#include <WiFi.h>
#include <ThingsBoard.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// Configurables
constexpr char WIFI_SSID[] = "";
constexpr char WIFI_PASSWORD[] = "";
constexpr char TOKEN[] = "";
constexpr char THINGSBOARD_SERVER[] = "";
const int GMT_OFFSET = -14400;  // Eastern Daylight Time (EDT) offset: UTC-4

// Constants
constexpr uint16_t THINGSBOARD_PORT = 1883;
constexpr uint32_t MAX_MESSAGE_SIZE = 256U;
const char* ntpServer = "pool.ntp.org";
const int MOTOR_TRIGGER_DURATION = 6;      // Seconds
const int TRIGGER_COOLDOWN_PERIOD = 1380;  // seconds
const int MAX_ATTEMPTS = 10;

// Pin Definitions
#define ONBOARD_LED 2
#define MOTOR_GPIO 19
#define BATTERY_GPIO 34

WiFiClient espClient;
WiFiUDP udp;
NTPClient timeClient(udp, ntpServer, GMT_OFFSET);
ThingsBoard tb(espClient, MAX_MESSAGE_SIZE);

// Variables
RTC_DATA_ATTR int Timer[7] = { 23400, 36000, 46800, 54000, 64800, 82800, 109800 };  // Failsafe Timers
RTC_DATA_ATTR int currentTime = 0;
RTC_DATA_ATTR int sleepTime = 0;
RTC_DATA_ATTR bool restartAttempt = false;

bool requestedShared = false;
constexpr std::array<const char*, 6U> REQUESTED_SHARED_ATTRIBUTES = {
  "Timer0",
  "Timer1",
  "Timer2",
  "Timer3",
  "Timer4",
  "Timer5",
};

// Function to handle shared attribute request from ThingsBoard
void processSharedAttributeRequest(const Shared_Attribute_Data& data) {
	int jsonSize = JSON_STRING_SIZE(measureJson(data));
	char buffer[jsonSize];
	serializeJson(data, buffer, jsonSize);
	StaticJsonDocument<JSON_OBJECT_SIZE(6)> doc;
	DeserializationError error = deserializeJson(doc, buffer);
	for (int i = 0; i < 6; i++) {
		const char* timerKey = REQUESTED_SHARED_ATTRIBUTES[i];
		if (doc.containsKey(timerKey)) {
			Timer[i] = doc[timerKey].as<int>() - TRIGGER_COOLDOWN_PERIOD / 2;
		}
	}
	Timer[6] = 86400 + Timer[0];  // First Timer for Next Day
	requestedShared = true;
}
// Callback for handling shared attribute requests
const Attribute_Request_Callback sharedCallback(REQUESTED_SHARED_ATTRIBUTES.cbegin(), REQUESTED_SHARED_ATTRIBUTES.cend(), &processSharedAttributeRequest);


// Initialize Wifi Connection and Set Time
bool connectWifi() {
	bool isConnected = false;
	Serial.println("Connecting to WiFi...");
	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
	for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
		if (WiFi.status() != WL_CONNECTED) {
			delay(500);
			Serial.print(".");
		}
		else {
			isConnected = true;
			Serial.println("Connected to WiFi.");
			break;
		}
	}

	if (!isConnected) {
		currentTime = (currentTime + sleepTime) % 86400;
		Serial.println("Warning Code 101: No Network Conneceted");
		return false;
	}

	Serial.println("Waiting for NTP synchronization...");
	for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
		if (!timeClient.update()) {
			delay(500);
			Serial.print(".");
		}
		else {
			int networkTime = timeClient.getSeconds() + timeClient.getMinutes() * 60 + timeClient.getHours() * 3600;
			if (networkTime != 0) {
				currentTime = networkTime;
				Serial.println("NTP synchronized.");
				return true;
			}
		}
	}
	Serial.println("Warning Code 102: Network Time Not Set");
	currentTime = (currentTime + sleepTime) % 86400;
	return true;
}

// Connect to Thingsboard Server and Get Time Table
bool connectServer() {
	bool isConnected = false;
	requestedShared = false;
	Serial.println("Connecting to ThingsBoard server");
	for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
		if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
			delay(500);
		}
		else {
			Serial.println("Connected to Thingsboard.");
			isConnected = true;
			attempt = MAX_ATTEMPTS;
		}
	}

	tb.Shared_Attributes_Request(sharedCallback);
	delay(500);
	freeCache();
	if (requestedShared) {
		Serial.println("Received Time Table.");
	}
	return isConnected;
}

// Restart Device
void restartIfNeeded(bool isConnected) {
	if (!isConnected) {
		if (restartAttempt == false) {
			Serial.println("Error Code 201: Network Communication Failure (Restarting)");
			restartAttempt = true;
			delay(100);
			ESP.restart();
		}
		currentTime += sleepTime % 86400;
	}
	else {
		if (restartAttempt == true) {
			restartAttempt = false;
			Serial.println("Error Code 202: Network Connection Lost (Reconnected)");
		}
	}
}

// Clears Thingsboard Cache
void freeCache() {
	for (int i = 0; i < 10; i++) {
		tb.loop();
		delay(100);
	}
}

// Function to connect to WiFi and ThingsBoard
void connect() {
	digitalWrite(ONBOARD_LED, HIGH);
	bool isConnected = connectWifi();
	if (isConnected) {
		isConnected = connectServer();
	}

	//restartIfNeeded(isConnected);
	digitalWrite(ONBOARD_LED, LOW);
}

// Function to put the microcontroller into deep sleep mode for a specified duration.
void deepSleep(uint64_t sleepTimeMicroseconds) {
    Serial.flush();
    WiFi.disconnect(true);
    esp_sleep_enable_timer_wakeup(sleepTimeMicroseconds);
    esp_deep_sleep_start();
}

void setup() {

	Serial.begin(115200);
	Serial.println("Booting...");

	pinMode(ONBOARD_LED, OUTPUT);
	pinMode(MOTOR_GPIO, OUTPUT);
	pinMode(BATTERY_GPIO, INPUT);

	//esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);

	connect();

	int nextFeedTime = Timer[0];
	int lastFeedTime = Timer[5];
	for (int i = 1; i < 7; i++) {
		if (currentTime < Timer[i]) {
			nextFeedTime = Timer[i];
			lastFeedTime = Timer[i - 1];
			break;  // Exit the loop after finding the nextFeedTime
		}
	}

	int timeToNextFeed = nextFeedTime - currentTime;
	if (timeToNextFeed < TRIGGER_COOLDOWN_PERIOD) {
		sleepTime = timeToNextFeed + TRIGGER_COOLDOWN_PERIOD / 2;
	}
	else {
		sleepTime = TRIGGER_COOLDOWN_PERIOD;
	}

	int timeSinceLastFeed = (currentTime - lastFeedTime + 86400) % 86400;
	if (timeSinceLastFeed < TRIGGER_COOLDOWN_PERIOD) {
		Serial.println("Running Motor..");
		tb.sendTelemetryData("TimeLastFeed", currentTime);
		digitalWrite(MOTOR_GPIO, HIGH);
		delay(MOTOR_TRIGGER_DURATION * 1000);
		digitalWrite(MOTOR_GPIO, LOW);
		sleepTime = TRIGGER_COOLDOWN_PERIOD * 2;
	}
	else {
		float batteryVoltage = analogRead(BATTERY_GPIO) * (3.3 / 4095.0);
		int rssi = WiFi.RSSI();
		const uint8_t data_items = 2U;
		Telemetry data[data_items] = {
		  { "BatteryVoltage", batteryVoltage },
		  { "RSSI", rssi },
		};
		tb.sendTelemetry(data, data_items);
	}

	freeCache();

	// Debug messages
	Serial.print("Last Feed Time: ");
	Serial.println(lastFeedTime);
	Serial.print("Time Since Last Feed: ");
	Serial.println(timeSinceLastFeed);
	Serial.print("Current Time: ");
	Serial.println(currentTime);
	Serial.print("Next Time to Next Feed: ");
	Serial.println(timeToNextFeed);
	Serial.print("Next Feed Time: ");
	Serial.println(nextFeedTime % 86400);
	Serial.print("Sleep Time: ");
	Serial.println(sleepTime);

	uint64_t sleepTimeMicroseconds = static_cast<uint64_t>(sleepTime) * 1000000ULL;
	deepSleep(sleepTimeMicroseconds);
}

void loop() {
	// Will Not Execute
}
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

// Constants
constexpr uint16_t THINGSBOARD_PORT = 1883;
constexpr uint32_t MAX_MESSAGE_SIZE = 256U;
const int MOTOR_TIME = 6;      // Seconds
const int SLEEP_TIME = 4;      // Seconds
const int MAX_ATTEMPTS = 10;

// Pin Definitions
#define ONBOARD_LED 2
#define MOTOR_GPIO 19
#define BATTERY_GPIO 34


// Variables
bool isFeed = false;
bool requestedShared = false;

WiFiClient espClient;
ThingsBoard tb(espClient, MAX_MESSAGE_SIZE);

constexpr std::array<const char*, 1U> REQUESTED_SHARED_ATTRIBUTES = {
  "isFeed",
};


void processSharedAttributeRequest(const Shared_Attribute_Data& data) {
	const size_t jsonSize = Helper::Measure_Json(data);
	char buffer[jsonSize];
	serializeJson(data, buffer, jsonSize);

	StaticJsonDocument<JSON_OBJECT_SIZE(1)> doc;
	DeserializationError error = deserializeJson(doc, buffer);
	isFeed = doc["isFeed"].as<bool>();
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
			Serial.println("Connected to WiFi.");
			return true;
			break;
		}
	}
	return false;
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
	if (isConnected) {
		tb.Shared_Attributes_Request(sharedCallback);
		delay(500);
		freeCache();
		if (requestedShared) {
			Serial.println("Received Time Table.");
			return true;
		}
	}
	return false;
}


// Clears Thingsboard Cache
void freeCache() {
	for (int i = 0; i < 10; i++) {
		tb.loop();
		delay(100);
	}
}

// Function to connect to WiFi and ThingsBoard
bool connect() {
	digitalWrite(ONBOARD_LED, HIGH);
	bool isConnected = connectWifi();
	if (isConnected) isConnected = connectServer();
	digitalWrite(ONBOARD_LED, LOW);
	return isConnected;
}

// Function to put the microcontroller into deep sleep mode for a specified duration.
void deepSleep(uint64_t sleepTimeMicroseconds) {
	Serial.println("Sleeping...");
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

	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);

}

void loop() {
	if (connect()) {

		if (isFeed) {
			Serial.println("Feeding....");
			const uint8_t data_items = 1U;
			Telemetry data[data_items] = {
			  { "feed", true },
			};
			tb.sendTelemetry(data, data_items);
      digitalWrite(MOTOR_GPIO, HIGH);
			delay(MOTOR_TIME * 1000);
			digitalWrite(MOTOR_GPIO, LOW);
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

		deepSleep(SLEEP_TIME * 1000000);
	}
}
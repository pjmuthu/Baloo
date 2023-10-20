# Baloo - Zoo Enrichment and Feeder Control Firmware
Baloo is a project that provides firmware for a series of devices designed to control feeders and enrichment activities in zoos. These devices are connected to a Google Cloud Platform (GCP) instance, enabling zoo staff to remotely manage and monitor the feeding and enrichment schedules for animals. Baloo's firmware is designed for optimal performance and minimal power consumption to ensure efficient operation.

## Firmware Modules

### 1. `esp32-sdrrc.c`

This module is responsible for controlling enrichment activities for the animals. The device is connected to a web-enabled application that allows users to turn a generic enrichment on or off. The code in this module is optimized for low power consumption, as the system follows a cycle of turning off and then powering on. It operates with a 10% duty cycle to minimize any potential lag in the enrichment activities.

Key Features:
- Real-time control of enrichment activities.
- Web-enabled application for user interaction.
- Low power optimization with a 10% duty cycle.

### 2. `esp32-snzf.c`

This module is designed to control the feeding operations for the animals. It connects to a server that hosts a schedule of feed times, which can be modified server-side. The system will periodically wake up at optimized intervals to perform feed operations at the specified times. The device is highly optimized for ultra-low power consumption, allowing the system as a whole to operate at an astonishingly low 100 nA.

Key Features:
- Control of feeding operations based on a server-defined schedule.
- Ultra-low power consumption, operating at 100 nA.
- Power optimization to minimize power usage during sleep periods.

## Getting Started

To use Baloo's firmware for your zoo's feeders and enrichment devices, follow these steps:

1. Clone this repository to your development environment.

2. Configure the `esp32-sdrrc.c` and `esp32-snzf.c` modules according to your specific requirements, such as network settings and server details.

3. Compile the firmware for your target ESP32 devices using your preferred toolchain.

4. Flash the compiled firmware onto the ESP32 devices.

5. Connect the devices to your GCP instance to enable remote management and monitoring.

6. Ensure that the server-side feed schedule is configured to match your zoo's feeding requirements.

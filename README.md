# ESP32 ThingsBoard Project

This project implements an ESP32-based IoT device that connects to ThingsBoard platform. It includes:

- Servo motor control via RPC
- Sensor data monitoring (temperature, humidity, light)
- Real-time data visualization on ThingsBoard dashboard
- WiFi connectivity
- MQTT communication

## Features

- Remote servo control through ThingsBoard RPC
- Continuous sensor data monitoring
- Automatic data upload to ThingsBoard
- Configurable update intervals
- Error handling and recovery

## Hardware Requirements

- ESP32 development board
- Servo motor
- Temperature and humidity sensor (DHT11/DHT22)
- Light sensor
- Jumper wires
- Breadboard (optional)

## Software Requirements

- ESP-IDF framework
- ThingsBoard platform
- ArduinoJson library
- ThingsBoard library

## Setup Instructions

1. Clone this repository
2. Install ESP-IDF framework
3. Configure WiFi credentials in `config.h`
4. Configure ThingsBoard credentials in `config.h`
5. Build and flash the project

## Usage

1. Power on the ESP32
2. The device will connect to WiFi and ThingsBoard
3. Use ThingsBoard dashboard to:
   - Monitor sensor data
   - Control servo motor
   - View device status

## License

This project is licensed under the MIT License - see the LICENSE file for details.

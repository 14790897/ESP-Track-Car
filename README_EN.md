# ESP32 Intelligent Tracking Car

[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-blue)](https://platformio.org/)
[![Framework](https://img.shields.io/badge/Framework-Arduino-green)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

> English Version | [中文版本](README.md)

An ESP32-based intelligent tracking car project with integrated radar sensor for target detection and automatic tracking.

## 🚗 Project Overview

This project implements a four-wheel intelligent car with the following features:

- 🎯 Radar target detection and tracking
- 📱 Web interface remote control
- 🔄 Automatic tracking mode
- 📡 WiFi connectivity and mDNS support
- 🎮 Real-time control response

## 🛠️ Hardware Requirements

### Main Controller

- **ESP32-C3** (AirM2M Core ESP32-C3)

### Motor Drivers

- **2x DRV8833** dual motor driver modules
- **4x DC Motors**

### Sensors

- **LD2450** radar module (24GHz mmWave Radar)

### Pin Connections

#### DRV8833 #1 (Motors A & B)

```text
IN1 (Motor A Dir1) -> GPIO 2
IN2 (Motor A Dir2) -> GPIO 3
IN3 (Motor B Dir1) -> GPIO 10
IN4 (Motor B Dir2) -> GPIO 6
```

#### DRV8833 #2 (Motors C & D)

```text
IN5 (Motor C Dir1) -> GPIO 12
IN6 (Motor C Dir2) -> GPIO 18
IN7 (Motor D Dir1) -> GPIO 19
IN8 (Motor D Dir2) -> GPIO 13
```

#### LD2450 Radar Module

```text
TX -> GPIO 4 (UART1 RX)
RX -> GPIO 5 (UART1 TX)
VCC -> 3.3V
GND -> GND
```

## 📋 Software Dependencies

### PlatformIO Libraries

```ini
lib_deps = 
    ESP32Async/ESPAsyncWebServer
    ESP32Async/AsyncTCP
    ArduinoJson
```

### Framework

- **Arduino Framework**
- **LittleFS** filesystem

## 🚀 Quick Start

### 1. Environment Setup

1. Install [PlatformIO](https://platformio.org/)
2. Clone the project:

```bash
git clone <repository-url>
cd ESP-Track-Car
```

### 2. WiFi Configuration

Edit the `include/secrets.h` file:

```cpp
const char* WIFI_SSID = "Your_WiFi_Name";
const char* WIFI_PASSWORD = "Your_WiFi_Password";
```

### 3. Build and Upload

```bash
# Build project
pio run

# Upload firmware
pio run --target upload

# Upload filesystem
pio run --target uploadfs

# Monitor serial
pio device monitor
```

### 4. Access Control Interface

1. Find the ESP32 IP address in the serial monitor
2. Access via browser: `http://ESP32_IP_ADDRESS` or `http://esp32car.local`

## 🎮 Usage Instructions

### Web Control Interface

The control interface provides:

- **Direction Control**: Forward, backward, left turn, right turn, stop
- **Motor Speed Control**: Independent adjustment of four motor speeds
- **Auto-tracking Mode**: Enable/disable automatic target tracking
- **Radar Data Display**: Real-time display of detected target information

### Auto-tracking Mode

When auto-tracking mode is enabled, the car will:

1. Automatically detect targets within 0.3-3.0 meters range
2. Track the nearest valid target
3. Automatically adjust direction and speed based on target position

## 📊 Technical Features

### Radar Detection

- **Detection Range**: 0.3-3.0 meters
- **Angle Threshold**: ±15 degrees
- **Max Targets**: 3 simultaneous tracking
- **Update Rate**: 20Hz

### Motor Control

- **PWM Frequency**: 5kHz
- **Resolution**: 8-bit (0-255)
- **Independent Speed Control**: Supports independent adjustment of four motors

### Network Features

- **Web Server**: Asynchronous HTTP server
- **mDNS**: Support for esp32car.local domain access
- **Real-time Communication**: WebSocket real-time data transmission
- **File System**: LittleFS for storing web interface files

## 🔧 Development Notes

### Project Structure

```text
ESP-Track-Car/
├── src/
│   └── main.cpp              # Main program code
├── include/
│   ├── README
│   └── secrets.h             # WiFi configuration file
├── data/
│   └── index.html            # Web interface file
├── lib/                      # Custom library directory
├── test/                     # Test code directory
└── platformio.ini            # PlatformIO configuration file
```

### Main Function Modules

1. **Motor Control Module**
2. **Radar Data Processing Module**
3. **Web Server Module**
4. **Auto-tracking Algorithm Module**
5. **WiFi Connection Management Module**

### Debug Information

Serial output includes:

- WiFi connection status
- Radar detected target information
- Auto-tracking status
- System running heartbeat information

## 🛡️ Safety Notes

⚠️ **Important Reminders:**

1. Ensure the car operates in a safe environment
2. Maintain supervision when in auto-tracking mode
3. Regularly check motor and circuit connections
4. Avoid prolonged continuous operation to prevent overheating

## 📝 Changelog

### v1.0.0

- ✅ Basic four-wheel motor control
- ✅ LD2450 radar integration
- ✅ Web interface control
- ✅ Auto-tracking functionality
- ✅ WiFi connection management
- ✅ mDNS domain support

## 🤝 Contributing

Welcome to submit Issues and Pull Requests to improve this project!

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Contact

For questions or suggestions, please contact via:

- Submit an Issue
- Send Email
- Project Discussions

---

⭐ If this project helps you, please give it a star for support!

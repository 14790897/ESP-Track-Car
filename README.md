# ESP32 智能跟踪小车

[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-blue)](https://platformio.org/)
[![Framework](https://img.shields.io/badge/Framework-Arduino-green)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

> [English Version](README_EN.md) | 中文版本

一个基于ESP32的智能跟踪小车项目，集成雷达传感器进行目标检测和自动跟踪。

## 🚗 项目概述

这个项目实现了一个四轮智能小车，具备以下功能：

- 🎯 雷达目标检测与跟踪
- 📱 Web界面远程控制
- 🔄 自动跟踪模式
- 📡 WiFi连接和mDNS支持
- 🎮 实时控制响应

## 🛠️ 硬件要求

### 核心控制器

- **ESP32-C3** (AirM2M Core ESP32-C3)

### 电机驱动

- **2x DRV8833** 双电机驱动模块
- **4x 直流电机** (DC Motors)

### 传感器

- **LD2450** 雷达模块 (24GHz mmWave Radar)

### 连接图

#### DRV8833 #1 (电机 A & B)

```text
IN1 (Motor A Dir1) -> GPIO 2
IN2 (Motor A Dir2) -> GPIO 3
IN3 (Motor B Dir1) -> GPIO 10
IN4 (Motor B Dir2) -> GPIO 6
```

#### DRV8833 #2 (电机 C & D)

```text
IN5 (Motor C Dir1) -> GPIO 12
IN6 (Motor C Dir2) -> GPIO 18
IN7 (Motor D Dir1) -> GPIO 19
IN8 (Motor D Dir2) -> GPIO 13
```

#### LD2450 雷达模块

```text
TX -> GPIO 4 (UART1 RX)
RX -> GPIO 5 (UART1 TX)
VCC -> 3.3V
GND -> GND
```

## 📋 软件依赖

### PlatformIO 库

```ini
lib_deps = 
    ESP32Async/ESPAsyncWebServer
    ESP32Async/AsyncTCP
    ArduinoJson
```

### 框架

- **Arduino Framework**
- **LittleFS** 文件系统

## 🚀 快速开始

### 1. 环境准备

1. 安装 [PlatformIO](https://platformio.org/)
2. 克隆项目：

```bash
git clone <repository-url>
cd ESP-Track-Car
```

### 2. 配置WiFi

编辑 `include/secrets.h` 文件：

```cpp
const char* WIFI_SSID = "您的WiFi名称";          // Your WiFi SSID
const char* WIFI_PASSWORD = "您的WiFi密码";      // Your WiFi Password
```

### 3. 编译上传

```bash
# 编译项目
pio run

# 上传固件
pio run --target upload

# 上传文件系统
pio run --target uploadfs

# 监控串口
pio device monitor
```

### 4. 访问控制界面

1. 串口监控器中查找ESP32的IP地址
2. 浏览器访问: `http://ESP32_IP_ADDRESS` 或 `http://esp32car.local`

## 🎮 使用说明

### Web控制界面

控制界面提供以下功能：

- **方向控制**: 前进、后退、左转、右转、停止
- **电机速度调节**: 独立调节四个电机速度
- **自动跟踪模式**: 开启/关闭自动目标跟踪
- **雷达数据显示**: 实时显示检测到的目标信息

### 自动跟踪模式

启用自动跟踪模式后，小车将：

1. 自动检测0.3-3.0米范围内的目标
2. 跟踪最近的有效目标
3. 根据目标位置自动调整方向和速度

## 📊 技术特性

### 雷达检测

- **检测范围**: 0.3-3.0米
- **角度阈值**: ±15度
- **最大目标数**: 3个同时跟踪
- **更新频率**: 20Hz

### 电机控制

- **PWM频率**: 5kHz
- **分辨率**: 8位 (0-255)
- **独立速度控制**: 支持四个电机独立调速

### 网络功能

- **Web服务器**: 异步HTTP服务器
- **mDNS**: 支持esp32car.local域名访问
- **实时通信**: WebSocket实时数据传输
- **文件系统**: LittleFS存储Web界面文件

## 🔧 开发说明

### 项目结构

```text
ESP-Track-Car/
├── src/
│   └── main.cpp              # 主程序代码
├── include/
│   ├── README
│   └── secrets.h             # WiFi配置文件
├── data/
│   └── index.html            # Web界面文件
├── lib/                      # 自定义库目录
├── test/                     # 测试代码目录
└── platformio.ini            # PlatformIO配置文件
```

### 主要功能模块

1. **电机控制模块**
2. **雷达数据处理模块**
3. **Web服务器模块**
4. **自动跟踪算法模块**
5. **WiFi连接管理模块**

### 调试信息

串口输出包含：

- WiFi连接状态
- 雷达检测目标信息
- 自动跟踪状态
- 系统运行心跳信息

## 🛡️ 安全注意事项

⚠️ **重要提醒:**

1. 确保小车在安全的环境中运行
2. 自动跟踪模式下请保持监控
3. 定期检查电机和电路连接
4. 避免长时间连续运行导致过热

## 📝 更新日志

### v1.0.0

- ✅ 基础四轮电机控制
- ✅ LD2450雷达集成
- ✅ Web界面控制
- ✅ 自动跟踪功能
- ✅ WiFi连接管理
- ✅ mDNS域名支持

## 🤝 贡献

欢迎提交Issues和Pull Requests来改进这个项目！

## 📄 许可证

本项目采用MIT许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 📞 联系

如有问题或建议，请通过以下方式联系：

- 提交Issue
- 发送邮件
- 项目讨论区

---

⭐ 如果这个项目对您有帮助，请给个Star支持一下！

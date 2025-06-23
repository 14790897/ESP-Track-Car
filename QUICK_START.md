# ESP32智能跟踪车 - 快速启动指南

## 🚀 5分钟快速开始

### 第一步：硬件准备
确保你有以下硬件：
- ESP32-C3开发板 × 1
- DRV8833电机驱动模块 × 2  
- 直流电机 × 4
- LD2450雷达模块 × 1
- 面包板和杜邦线若干

### 第二步：接线连接
```
LD2450雷达:
TX -> GPIO4 (ESP32 RX)
RX -> GPIO5 (ESP32 TX)
VCC -> 3.3V
GND -> GND

电机驱动DRV8833 #1:
IN1 -> GPIO2  (电机A+)
IN2 -> GPIO3  (电机A-)
IN3 -> GPIO10 (电机B+)
IN4 -> GPIO6  (电机B-)

电机驱动DRV8833 #2:
IN5 -> GPIO12 (电机C+)
IN6 -> GPIO18 (电机C-)
IN7 -> GPIO19 (电机D+)
IN8 -> GPIO13 (电机D-)
```

### 第三步：配置WiFi
1. 打开 `include/secrets.h`
2. 修改WiFi设置：
```cpp
const char* ssid = "你的WiFi名称";
const char* password = "你的WiFi密码";
```

### 第四步：上传代码
```bash
# 使用PlatformIO CLI
pio run -t upload

# 或者在VS Code中按Ctrl+Alt+U
```

### 第五步：测试验证
1. **打开串口监视器** (115200波特率)
2. **查看IP地址** - 在串口输出中找到类似 `WiFi连接成功！IP地址: 192.168.1.xxx`
3. **访问Web界面** - 在浏览器中输入该IP地址
4. **测试基本功能**：
   - 点击"前进"按钮，观察车辆是否向前移动
   - 点击"自动跟踪"按钮，在车前放置物体测试跟踪

## 🔧 常见问题解决

### 问题1：WiFi连接失败
```
检查清单:
☐ WiFi名称和密码是否正确
☐ 是否使用2.4GHz网络（ESP32不支持5GHz）
☐ 网络是否有访问限制
```

### 问题2：电机不转动
```
检查清单:
☐ 电机驱动模块是否正确供电
☐ 接线是否正确牢固
☐ 电机本身是否正常工作
```

### 问题3：雷达无数据
```
检查清单:
☐ LD2450的TX/RX是否接反
☐ 波特率是否为256000
☐ 供电是否稳定（3.3V）
```

## 📖 下一步学习

1. **查看详细示例** → `examples/` 目录
2. **了解参数调优** → `examples/README.md`
3. **自定义功能** → 修改 `src/main.cpp`

## 🆘 获取帮助

- 查看完整文档：`examples/README.md`
- 测试代码示例：`examples/test_scenarios/`
- 配置参考：`examples/configuration/`

**恭喜！🎉 你的ESP32智能跟踪车现在应该可以正常工作了！**

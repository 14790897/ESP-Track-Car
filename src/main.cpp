#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <math.h>
#include <esp_task_wdt.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include "secrets.h"

// --- Pin Definitions ---
// DRV8833 #1 - Controls Motor A and B
#define IN1_PIN 2  // Motor A Direction 1
#define IN2_PIN 3  // Motor A Direction 2
#define IN3_PIN 10 // Motor B Direction 1
#define IN4_PIN 6  // Motor B Direction 2
// DRV8833 #2 - Controls Motor C and D
#define IN5_PIN 12 // Motor C Direction 1
#define IN6_PIN 18 // Motor C Direction 2
#define IN7_PIN 19 // Motor D Direction 1
#define IN8_PIN 13 // Motor D Direction 2

// --- PWM Properties ---
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;
const int MAX_DUTY_CYCLE = (1 << PWM_RESOLUTION) - 1; // 255 for 8-bit

// --- Global Variables ---
AsyncWebServer server(80);
int motorASpeed = 150;
int motorBSpeed = 150;
int motorCSpeed = 150;
int motorDSpeed = 150;

// --- Radar Variables ---
HardwareSerial ld2450Serial(1); // 使用UART1

struct TagInfoBean {
    double x;
    double y;
    double speed;
    double resolution;
    double distance;
    double angle;
    unsigned long timestamp;
};

TagInfoBean currentTargets[3];
int currentTargetCount = 0;
unsigned long lastDetectionTime = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastTrackingOutput = 0; // 上次输出跟踪数据的时间
bool isDetecting = true;
bool autoTrackingMode = false; // 自动跟踪模式

// Radar Protocol Constants
const String notTargetDataStr = "0000000000000000";
const int singleTargetDataLength = 16;
const int totalFrameLength = 60;

// Tracking Parameters
const double MIN_DISTANCE = 0.3; // 最小跟踪距离(米)
const double MAX_DISTANCE = 3.0; // 最大跟踪距离(米)
const double ANGLE_THRESHOLD = 30.0; // 角度阈值(度)

// --- Helper Functions ---
String getContentType(String filename) {
    if (filename.endsWith(".html")) return "text/html";
    else if (filename.endsWith(".css")) return "text/css";
    else if (filename.endsWith(".js")) return "application/javascript";
    else if (filename.endsWith(".svg")) return "image/svg+xml";
    else if (filename.endsWith(".png")) return "image/png";
    else if (filename.endsWith(".jpg")) return "image/jpeg";
    else if (filename.endsWith(".gif")) return "image/gif";
    else if (filename.endsWith(".ico")) return "image/x-icon";
    else if (filename.endsWith(".xml")) return "text/xml";
    else if (filename.endsWith(".pdf")) return "application/x-pdf";
    else if (filename.endsWith(".zip")) return "application/x-zip";
    else if (filename.endsWith(".gz")) return "application/x-gzip";
    return "text/plain";
}

// --- Motor Control Functions ---
void motorA_Forward(int speed) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
}

void motorA_Backward(int speed) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
}

void motorA_Stop() {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
}

void motorB_Forward(int speed) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
}

void motorB_Backward(int speed) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
}

void motorB_Stop() {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
}

void motorC_Forward(int speed) {
    digitalWrite(IN5_PIN, HIGH);
    digitalWrite(IN6_PIN, LOW);
}

void motorC_Backward(int speed) {
    digitalWrite(IN5_PIN, LOW);
    digitalWrite(IN6_PIN, HIGH);
}

void motorC_Stop() {
    digitalWrite(IN5_PIN, LOW);
    digitalWrite(IN6_PIN, LOW);
}

void motorD_Forward(int speed) {
    digitalWrite(IN7_PIN, HIGH);
    digitalWrite(IN8_PIN, LOW);
}

void motorD_Backward(int speed) {
    digitalWrite(IN7_PIN, LOW);
    digitalWrite(IN8_PIN, HIGH);
}

void motorD_Stop() {
    digitalWrite(IN7_PIN, LOW);
    digitalWrite(IN8_PIN, LOW);
}

void motorForward() {
    motorA_Forward(motorASpeed);
    motorB_Forward(motorBSpeed);
    motorC_Forward(motorCSpeed);
    motorD_Forward(motorDSpeed);
    Serial.println("Moving Forward");
}

void motorBackward() {
    motorA_Backward(motorASpeed);
    motorB_Backward(motorBSpeed);
    motorC_Backward(motorCSpeed);
    motorD_Backward(motorDSpeed);
    Serial.println("Moving Backward");
}

void motorLeft() {
    motorA_Forward(motorASpeed);
    motorC_Forward(motorCSpeed);
    motorB_Backward(motorBSpeed);
    motorD_Backward(motorDSpeed);
    Serial.println("Turning Left");
}

void motorRight() {
    motorA_Backward(motorASpeed);
    motorC_Backward(motorCSpeed);
    motorB_Forward(motorBSpeed);
    motorD_Forward(motorDSpeed);
    Serial.println("Turning Right");
}

void motorStop() {
    motorA_Stop();
    motorB_Stop();
    motorC_Stop();
    motorD_Stop();
    Serial.println("All Motors Stopped");
}

void setMotorSpeed(int speed) {
    if (speed < 0) speed = 0;
    if (speed > MAX_DUTY_CYCLE) speed = MAX_DUTY_CYCLE;
    motorASpeed = speed;
    motorBSpeed = speed;
    motorCSpeed = speed;
    motorDSpeed = speed;
    Serial.print("Global speed set to: ");
    Serial.println(speed);
}

// --- Radar Functions ---
int hexToInt(String hex)
{
    return (int)strtol(hex.c_str(), NULL, 16);
}

// 根据LD2450协议解析X坐标
double calculateXValue(String str)
{
    int byteLow = hexToInt(str.substring(0, 2));
    int byteHigh = hexToInt(str.substring(2, 4));
    int rawValue = byteLow + byteHigh * 256; // 小端字节序

    // 按协议：高位字节最高位为1表示正，0表示负
    if (byteHigh >= 128)
    {
        // 正坐标：修正范围，去掉符号偏移（减去32768）
        return (double)(rawValue - 32768);
    }
    else
    {
        // 负坐标：直接是负值
        return (double)(-rawValue);
    }
}

// 根据LD2450协议解析Y坐标
double calculateYValue(String str)
{
    int byteLow = hexToInt(str.substring(4, 6));
    int byteHigh = hexToInt(str.substring(6, 8));
    int value = byteLow + byteHigh * 256; // 小端字节序

    // signed int16处理：最高位1=正坐标，0=负坐标
    if (byteHigh >= 128)
    {
        // 正坐标：value - 2^15 (因为示例显示34481 - 32768 = 1713)
        return (double)(value - 32768);
    }
    else
    {
        // 负坐标：0 - value
        return (double)(0 - value);
    }
}

// 根据LD2450协议解析速度
double calculateSpeed(String str)
{
    int byteLow = hexToInt(str.substring(8, 10));
    int byteHigh = hexToInt(str.substring(10, 12));
    int value = byteLow + byteHigh * 256; // 小端字节序

    // signed int16处理：最高位1=正向速度，0=负向速度
    if (byteHigh >= 128)
    {
        // 正向速度：取低15位
        return (double)(value & 0x7FFF); // 保留低15位
    }
    else
    {
        // 负向速度：取相反数
        return (double)(-value);
    }
}

// 根据LD2450协议解析距离分辨率
double calculateResolution(String str)
{
    int byteLow = hexToInt(str.substring(12, 14));
    int byteHigh = hexToInt(str.substring(14, 16));
    int value = byteLow + byteHigh * 256; // 小端字节序

    // uint16类型，直接返回
    return (double)value;
}

double calculateDistance(double x, double y)
{
    return sqrt(x * x + y * y);
}

double calculateAngle(double x, double y)
{
    double angle = atan2(y, x) * (180.0 / PI);
    return -(90.0 - angle);
}

void analysisAreaCoordinate(String strData, TagInfoBean results[], int &resultCount)
{
    resultCount = 0;

    if (strData.length() < totalFrameLength)
    {
        Serial.printf("Data too short: %d < %d\n", strData.length(), totalFrameLength);
        return; // 数据不完整，直接返回
    }

    // 验证帧头
    String frameHeader = strData.substring(0, 8);
    if (!frameHeader.equalsIgnoreCase("AAFF0300"))
    {
        Serial.printf("Invalid frame header: %s\n", frameHeader.c_str());
        return;
    }

    // 从帧头后开始解析目标数据（跳过4字节帧头 = 8个十六进制字符）
    int dataStartIndex = 8;
    for (int i = 0; i < 3; i++)
    {
        int segmentStart = dataStartIndex + (i * singleTargetDataLength);
        String segment = strData.substring(segmentStart, segmentStart + singleTargetDataLength);
        if (segment != notTargetDataStr && segment.length() == singleTargetDataLength)
        {
            double x = calculateXValue(segment);
            double y = calculateYValue(segment);
            double speed = calculateSpeed(segment);
            double resolution = calculateResolution(segment);
            double distance = calculateDistance(x, y) / 1000.0; // 单位mm转为m
            double angle = calculateAngle(x, y);

            // 添加有效性检查
            if (distance < 0.01 || distance > 10.0)
            {
                Serial.printf("  Distance out of range (%.3fm), skipping target\n", distance);
                continue;
            }

            results[resultCount].x = x;
            results[resultCount].y = y;
            results[resultCount].speed = speed;
            results[resultCount].resolution = resolution;
            results[resultCount].distance = distance;
            results[resultCount].angle = angle;
            results[resultCount].timestamp = millis();
            resultCount++;

            // Serial.print("Valid target found - X: ");
            // Serial.print(x);
            // Serial.print("mm, Y: ");
            // Serial.print(y);
            // Serial.print("mm, Speed: ");
            // Serial.print(speed);
            // Serial.print("cm/s, Resolution: ");
            // Serial.print(resolution);
            // Serial.println("mm");
        }
    }
}

String readSerialData()
{
    // 帧头: AA FF 03 00
    byte frameHeader[4] = {0xAA, 0xFF, 0x03, 0x00};
    byte buffer[4] = {0}; // 用于检测帧头的循环缓冲区
    int bufferIndex = 0;
    bool frameFound = false;
    unsigned long startTime = millis();
    int bytesChecked = 0;

    // 第一阶段：寻找帧头
    while ((millis() - startTime < 100) && !frameFound && ld2450Serial.available())
    {
        byte c = ld2450Serial.read();
        buffer[bufferIndex] = c;
        bytesChecked++;

        // 只有读取了至少4个字节后才开始检查帧头
        if (bytesChecked >= 4)
        {
            // 检查是否找到帧头 - 修正索引计算
            bool headerMatch = true;
            for (int i = 0; i < 4; i++)
            {
                int checkIndex = (bufferIndex + 1 + i) % 4; // 从下一个位置开始按顺序检查
                if (buffer[checkIndex] != frameHeader[i])
                {
                    headerMatch = false;
                    break;
                }
            }
            if (headerMatch)
            {
                frameFound = true;
                // Serial.println("Frame header found!"); // 减少输出
            }
        }

        bufferIndex = (bufferIndex + 1) % 4;
        // 重置看门狗
    }
    if (!frameFound)
    {
        // Serial.println("No frame header found"); // 减少输出
        return "";
    }

    // 第二阶段：从帧头开始读取完整帧(30字节)
    char hexBuffer[61]; // 30字节 = 60字符 + 1结尾
    int index = 0;

    // 先写入已找到的帧头
    sprintf(&hexBuffer[index], "AAFF0300");
    index += 8;

    // 继续读取剩余的26字节
    startTime = millis();
    int remainingBytes = 26;

    while ((millis() - startTime < 50) && remainingBytes > 0 && ld2450Serial.available())
    {
        byte c = ld2450Serial.read();
        sprintf(&hexBuffer[index], "%02X", c);
        index += 2;
        remainingBytes--;
    }

    hexBuffer[index] = '\0';
    String data = String(hexBuffer);

    return data;
}

// --- Auto Tracking Functions ---
void autoTrackTarget()
{
    if (!autoTrackingMode || currentTargetCount == 0)
    {
        return;
    }

    // 数据收集和滤波变量
    static double angleHistory[5] = {0};    // 存储最近5次角度数据
    static double distanceHistory[5] = {0}; // 存储最近5次距离数据
    static int historyIndex = 0;
    static int validDataCount = 0;
    static unsigned long lastUpdateTime = 0;
    const unsigned long DATA_COLLECT_INTERVAL = 200; // 200ms收集一次数据
    const int MIN_DATA_COUNT = 3;                    // 至少需要3次稳定数据才执行动作

    // 找到最近的目标
    TagInfoBean *nearestTarget = nullptr;
    double minDistance = MAX_DISTANCE;

    for (int i = 0; i < currentTargetCount; i++)
    {
        if (currentTargets[i].distance >= MIN_DISTANCE &&
            currentTargets[i].distance <= MAX_DISTANCE)
        {
            if (currentTargets[i].distance < minDistance)
            {
                minDistance = currentTargets[i].distance;
                nearestTarget = &currentTargets[i];
            }
        }
    } // 静态变量用于跟踪目标丢失状态
    static unsigned long lastTargetLostTime = 0;
    static bool isSearching = false;
    static int searchDirection = 1; // 1为右转，-1为左转
    static unsigned long searchStartTime = 0;

    if (nearestTarget == nullptr)
    {
        // 没有找到有效目标的处理策略
        if (lastTargetLostTime == 0)
        {
            lastTargetLostTime = millis();
            Serial.println("Target lost - Starting search mode");
        }

        unsigned long timeSinceTargetLost = millis() - lastTargetLostTime;

        if (timeSinceTargetLost < 2000)
        {
            // 前2秒：停止并等待目标重新出现
            motorStop();
            Serial.println("Waiting for target to reappear...");
        }
        else if (timeSinceTargetLost < 20000)
        {
            // 2-20秒：开始搜索模式，左右转动寻找目标
            if (!isSearching)
            {
                isSearching = true;
                searchStartTime = millis();
                Serial.printf("Starting search pattern, direction: %s\n",
                              searchDirection > 0 ? "RIGHT" : "LEFT");
            }

            // 每3秒改变搜索方向
            if (millis() - searchStartTime > 3000)
            {
                searchDirection *= -1;
                searchStartTime = millis();
                Serial.printf("Changing search direction to: %s\n",
                              searchDirection > 0 ? "RIGHT" : "LEFT");
            }

            // 执行搜索转动
            if (searchDirection > 0)
            {
                motorRight();
            }
            else
            {
                motorLeft();
            }
            Serial.println("Searching for target...");
        }
        else
        {
            // 超过20秒：停止搜索，等待手动干预
            motorStop();
            isSearching = false;
            Serial.println("Search timeout - Stopping auto tracking");
            // 可选：自动关闭跟踪模式
            // autoTrackingMode = false;
        }

        return;
    }
    else
    {
        // 找到目标，重置搜索状态
        if (lastTargetLostTime != 0)
        {
            Serial.println("Target found - Resuming normal tracking");
            lastTargetLostTime = 0;
            isSearching = false;
            // 重置数据收集
            validDataCount = 0;
            historyIndex = 0;
        }
    }

    double angle = nearestTarget->angle;
    double distance = nearestTarget->distance;

    // 数据收集和滤波处理
    if (millis() - lastUpdateTime >= DATA_COLLECT_INTERVAL)
    {
        // 将新数据添加到历史记录
        angleHistory[historyIndex] = angle;
        distanceHistory[historyIndex] = distance;
        historyIndex = (historyIndex + 1) % 5;

        if (validDataCount < 5)
        {
            validDataCount++;
        }

        lastUpdateTime = millis();

        Serial.printf("Raw data: angle=%.1f°, distance=%.2fm (sample %d/5)\n",
                      angle, distance, validDataCount);
    }

    // 需要收集足够的数据才能执行动作
    if (validDataCount < MIN_DATA_COUNT)
    {
        Serial.println("Collecting data for stable tracking...");
        motorStop(); // 收集数据期间停止运动
        return;
    }

    // 计算滤波后的角度和距离（使用平均值）
    double avgAngle = 0;
    double avgDistance = 0;
    for (int i = 0; i < validDataCount; i++)
    {
        avgAngle += angleHistory[i];
        avgDistance += distanceHistory[i];
    }
    avgAngle /= validDataCount;
    avgDistance /= validDataCount;

    // 计算数据稳定性（标准差）
    double angleVariance = 0;
    double distanceVariance = 0;
    for (int i = 0; i < validDataCount; i++)
    {
        double angleDiff = angleHistory[i] - avgAngle;
        double distDiff = distanceHistory[i] - avgDistance;
        angleVariance += angleDiff * angleDiff;
        distanceVariance += distDiff * distDiff;
    }
    angleVariance /= validDataCount;
    distanceVariance /= validDataCount;

    double angleStdDev = sqrt(angleVariance);
    double distanceStdDev = sqrt(distanceVariance);

    // 检查数据稳定性
    const double MAX_ANGLE_STD_DEV = 5.0;    // 角度标准差阈值（度）
    const double MAX_DISTANCE_STD_DEV = 0.2; // 距离标准差阈值（米）

    if (angleStdDev > MAX_ANGLE_STD_DEV || distanceStdDev > MAX_DISTANCE_STD_DEV)
    {
        Serial.printf("Data unstable: angle_std=%.1f°, dist_std=%.3fm - Collecting more data\n",
                      angleStdDev, distanceStdDev);
        motorStop(); // 数据不稳定时停止
        return;
    }
    Serial.printf("Stable tracking: avg_angle=%.1f°, avg_distance=%.2fm (std: %.1f°/%.3fm)\n",
                  avgAngle, avgDistance, angleStdDev, distanceStdDev);

    // 转向控制变量
    static unsigned long lastTurnTime = 0;
    static bool isTurning = false;
    static unsigned long turnDuration = 0;

    // 根据滤波后的角度调整方向
    if (abs(avgAngle) > ANGLE_THRESHOLD)
    {
        // 计算转向时间：角度越大，转向时间越长
        // 基础转向时间 + 角度比例时间
        unsigned long baseTurnTime = 200;                                  // 基础转向时间200ms
        unsigned long angleTurnTime = (unsigned long)(abs(avgAngle) * 10); // 每度10ms
        unsigned long totalTurnTime = baseTurnTime + angleTurnTime;

        // 限制最大转向时间
        if (totalTurnTime > 1000)
            totalTurnTime = 1000; // 最大1秒

        if (!isTurning)
        {
            // 开始新的转向
            isTurning = true;
            lastTurnTime = millis();
            turnDuration = totalTurnTime;

            Serial.printf("Starting turn: angle=%.1f°, duration=%lums\n", avgAngle, turnDuration);

            if (avgAngle > 0)
            {
                motorRight(); // 目标在右侧，向右转
            }
            else
            {
                motorLeft(); // 目标在左侧，向左转
            }
        }
        else
        {
            // 检查转向是否完成
            if (millis() - lastTurnTime >= turnDuration)
            {
                isTurning = false;
                motorStop(); // 停止转向
                Serial.println("Turn completed, stopping");
                return;
            }
            // 转向进行中，继续当前转向动作
            Serial.printf("Turning... %lums/%lums\n", millis() - lastTurnTime, turnDuration);
        }
    }
    else
    {
        // 角度在阈值内，停止转向
        if (isTurning)
        {
            isTurning = false;
            motorStop();
            Serial.println("Target centered, stopping turn");
        }
        // 根据滤波后的距离控制前进/后退（仅在不转向时执行）
        if (!isTurning)
        {
            if (avgDistance > 1.0)
            {
                motorForward(); // 距离较远，前进
                Serial.println("Moving forward - target too far");
            }
            else if (avgDistance < 0.5)
            {
                motorBackward(); // 距离太近，后退
                Serial.println("Moving backward - target too close");
            }
            else
            {
                motorStop(); // 距离合适，停止
                Serial.println("Distance optimal - stopping");
            }
        }
    }
}

// --- Web Server Setup ---
void setupWebServer()
{
    // 运动控制API
    server.on("/forward", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        if (!autoTrackingMode) {
            motorForward();
        }
        request->send(200, "text/plain", "Moving Forward"); });

    server.on("/backward", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        if (!autoTrackingMode) {
            motorBackward();
        }
        request->send(200, "text/plain", "Moving Backward"); });

    server.on("/left", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        if (!autoTrackingMode) {
            motorLeft();
        }
        request->send(200, "text/plain", "Turning Left"); });

    server.on("/right", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        if (!autoTrackingMode) {
            motorRight();
        }
        request->send(200, "text/plain", "Turning Right"); });

    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        motorStop();
        request->send(200, "text/plain", "Stopped"); });

    server.on("/speed", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        if (request->hasParam("value")) {
            int speed = request->getParam("value")->value().toInt();
            setMotorSpeed(speed);
            request->send(200, "text/plain", "Speed set to " + String(speed));
        } else {
            request->send(400, "text/plain", "Missing speed parameter");
        } }); // 雷达数据API
    server.on("/radar-data", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        JsonDocument doc;
        JsonArray targets = doc["targets"].to<JsonArray>();
        
        for (int i = 0; i < currentTargetCount; i++) {
            JsonObject target = targets.add<JsonObject>();
            target["id"] = i;
            target["x"] = currentTargets[i].x;
            target["y"] = currentTargets[i].y;
            target["distance"] = currentTargets[i].distance;
            target["angle"] = currentTargets[i].angle;
            target["speed"] = currentTargets[i].speed;
        }
        
        doc["count"] = currentTargetCount;
        doc["timestamp"] = millis();
        doc["autoTracking"] = autoTrackingMode;
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response); });

    // 自动跟踪控制
    server.on("/auto-tracking", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        if (request->hasParam("enable")) {
            autoTrackingMode = request->getParam("enable")->value() == "true";
            if (!autoTrackingMode) {
                motorStop();
            }
            request->send(200, "text/plain", autoTrackingMode ? "Auto tracking enabled" : "Auto tracking disabled");
        } else {
            request->send(200, "text/plain", autoTrackingMode ? "enabled" : "disabled");
        } });

    // OTA更新API
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 OTA Update</title>
    <meta charset="UTF-8">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 600px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        h1 { color: #333; text-align: center; }
        .upload-form { margin: 20px 0; }
        input[type="file"] { width: 100%; padding: 10px; margin: 10px 0; border: 1px solid #ddd; border-radius: 5px; }
        button { background: #4CAF50; color: white; padding: 15px 30px; border: none; border-radius: 5px; cursor: pointer; width: 100%; font-size: 16px; }
        button:hover { background: #45a049; }
        .info { background: #e7f3ff; padding: 15px; border-radius: 5px; margin: 10px 0; }
        .warning { background: #fff3cd; padding: 15px; border-radius: 5px; margin: 10px 0; color: #856404; }
        .progress { width: 100%; background-color: #f0f0f0; border-radius: 5px; margin: 10px 0; }
        .progress-bar { width: 0%; height: 30px; background-color: #4CAF50; border-radius: 5px; text-align: center; line-height: 30px; color: white; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚗 ESP32跟踪车 OTA更新</h1>
        <div class="info">
            <strong>固件信息:</strong><br>
            版本: 1.0.0<br>
            编译时间: )" + String(__DATE__) + " " + String(__TIME__) + R"(<br>
            芯片型号: )" + String(ESP.getChipModel()) + R"(<br>
            可用内存: )" + String(ESP.getFreeHeap()) + R"( bytes
        </div>
        <div class="warning">
            <strong>注意:</strong> 更新过程中请勿断电或重启设备！
        </div>
        <div class="upload-form">
            <form method='POST' action='/update' enctype='multipart/form-data'>
                <input type='file' name='update' accept='.bin' required>
                <button type='submit'>开始更新固件</button>
            </form>
        </div>
        <div id="progress" class="progress" style="display:none;">
            <div id="progress-bar" class="progress-bar">0%</div>
        </div>
        <div id="status"></div>
    </div>
    <script>
        document.querySelector('form').addEventListener('submit', function() {
            document.getElementById('progress').style.display = 'block';
            document.getElementById('status').innerHTML = '<p style="color: blue;">正在上传固件...</p>';
        });
    </script>
</body>
</html>
        )";
        request->send(200, "text/html", html); });

    // OTA固件上传处理
    server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request)
              {
        bool updateSuccess = !Update.hasError();
        AsyncWebServerResponse *response = request->beginResponse(200, "text/html", 
            updateSuccess ? 
            "<html><body><h1>✅ 更新成功！</h1><p>设备将在3秒后重启...</p><script>setTimeout(function(){window.location.href='/';}, 3000);</script></body></html>" :
            "<html><body><h1>❌ 更新失败！</h1><p>错误: " + String(Update.errorString()) + "</p><a href='/update'>返回重试</a></body></html>"
        );
        response->addHeader("Connection", "close");
        request->send(response);
        
        if (updateSuccess) {
            Serial.println("OTA Update Success! Rebooting...");
            delay(1000);
            ESP.restart();
        } }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
              {
        if (!index) {
            Serial.printf("OTA Update Start: %s\n", filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                Update.printError(Serial);
                return;
            }
        }
        if (Update.write(data, len) != len) {
            Update.printError(Serial);
            return;
        }
        if (final) {
            if (Update.end(true)) {
                Serial.printf("OTA Update Success: %uB\n", index + len);
            } else {
                Update.printError(Serial);
            }
        }
        
        // 打印进度
        if (index % 10240 == 0) { // 每10KB打印一次进度
            Serial.printf("OTA Progress: %u bytes\n", index + len);
        } });

    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    server.begin();
    Serial.println("Web server started");
}

// 初始化OTA
void initOTA()
{
    // 设置OTA主机名
    ArduinoOTA.setHostname("ESP32-TrackCar");
    Serial.println("OTA hostname set to: ESP32-TrackCar");

    // 设置OTA密码（可选）
    // ArduinoOTA.setPassword("your_password_here");

    // OTA开始回调
    ArduinoOTA.onStart([]()
                       {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        // 停止所有电机，确保安全
        motorStop();
        Serial.println("Start updating " + type); });

    // OTA结束回调
    ArduinoOTA.onEnd([]()
                     { Serial.println("\nOTA Update completed successfully!"); });

    // OTA进度回调
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { 
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 1000) { // 每秒打印一次进度
            Serial.printf("OTA Progress: %u%% (%u/%u bytes)\n", (progress / (total / 100)), progress, total);
            lastPrint = millis();
        } });

    // OTA错误回调
    ArduinoOTA.onError([](ota_error_t error)
                       {
        Serial.printf("OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
        } });

    ArduinoOTA.begin();
    Serial.println("ArduinoOTA started successfully!");
    Serial.println("OTA Ready - Listening for updates...");
    Serial.print("Device IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Device hostname: ESP32-TrackCar.local");
    Serial.println();
    Serial.println("You can now upload via:");
    Serial.println("1. Arduino IDE -> Tools -> Port -> ESP32-TrackCar");
    Serial.println("2. PlatformIO -> airm2m_core_esp32c3_ota environment");
    Serial.printf("3. Web interface -> http://%s/update\n", WiFi.localIP().toString().c_str());
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Car with Radar Tracking starting...");
    
    // 初始化电机引脚
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
    pinMode(IN5_PIN, OUTPUT);
    pinMode(IN6_PIN, OUTPUT);
    pinMode(IN7_PIN, OUTPUT);
    pinMode(IN8_PIN, OUTPUT);
    
    // 停止所有电机
    motorStop();
    // 初始化LittleFS
    if (!LittleFS.begin()) {
        Serial.println("LittleFS initialization failed - using built-in web interface");
    }
    else
    {
        Serial.println("LittleFS initialized");
    }

    // 初始化WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
    // 初始化mDNS
    if (MDNS.begin(MDNS_HOST_NAME))
    {
        Serial.printf("mDNS started. Access at: http://%s.local\n", MDNS_HOST_NAME);
    }

    // 初始化OTA
    initOTA();

    // 初始化雷达串口
    ld2450Serial.begin(256000, SERIAL_8N1, 1, 0); // RX=1, TX=0
    Serial.println("LD2450 serial initialized");
    
    // 设置Web服务器
    setupWebServer();
    
    Serial.println("System ready!");
}

void loop() {
    // 处理OTA更新请求
    ArduinoOTA.handle();

    // 读取雷达数据
    String radarData = readSerialData();
    if (radarData.length() > 0) {
        analysisAreaCoordinate(radarData, currentTargets, currentTargetCount);
        lastDetectionTime = millis();
    }
    else
    {
        // 每5秒输出一次无数据的状态
        static unsigned long lastNoDataReport = 0;
        if (millis() - lastNoDataReport > 5000)
        {
            Serial.println("No radar data received");
            Serial.printf("Serial available: %d bytes\n", ld2450Serial.available());
            lastNoDataReport = millis();
        }
    }

    // 检查数据超时
    if (millis() - lastDetectionTime > 5000) {
        currentTargetCount = 0; // 清空过时的目标数据
    }
    // 执行自动跟踪
    if (autoTrackingMode) {
        autoTrackTarget();
    }

    // 定期输出跟踪数据（每2秒）
    if (millis() - lastTrackingOutput > 2000)
    {
        if (currentTargetCount > 0)
        {
            Serial.println("=== 跟踪数据报告 ===");
            Serial.printf("检测到目标数量: %d\n", currentTargetCount);
            for (int i = 0; i < currentTargetCount; i++)
            {
                Serial.printf("目标 %d:\n", i + 1);
                Serial.printf("  坐标: X=%.1fmm, Y=%.1fmm\n", currentTargets[i].x, currentTargets[i].y);
                Serial.printf("  距离: %.2fm\n", currentTargets[i].distance);
                Serial.printf("  角度: %.1f°\n", currentTargets[i].angle);
                Serial.printf("  速度: %.1fcm/s\n", currentTargets[i].speed);
                Serial.printf("  分辨率: %.0fmm\n", currentTargets[i].resolution);
                Serial.printf("  检测时间: %lums前\n", millis() - currentTargets[i].timestamp);
            }
            if (autoTrackingMode)
            {
                Serial.printf("自动跟踪模式: 开启\n");
            }
            else
            {
                Serial.printf("自动跟踪模式: 关闭\n");
            }
            Serial.println("==================");
        }
        else
        {
            Serial.println("=== 跟踪数据报告 ===");
            Serial.println("当前无目标检测");
            Serial.printf("原始雷达数据: %s\n", radarData.c_str());
            Serial.printf("自动跟踪模式: %s\n", autoTrackingMode ? "开启" : "关闭");
            Serial.printf("上次检测时间: %lums前\n", millis() - lastDetectionTime);
            Serial.println("==================");
        }
        lastTrackingOutput = millis();
    }

    // 心跳信号
    if (millis() - lastHeartbeat > 100000)
    {
        Serial.printf("System running - WiFi: %s, Targets: %d, Auto-tracking: %s\n", 
            WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
            currentTargetCount,
            autoTrackingMode ? "ON" : "OFF");
        lastHeartbeat = millis();
    }

    delay(50); // 20Hz主循环
}

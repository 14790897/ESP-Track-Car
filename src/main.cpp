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
const char *mdnsHostname = "esp32car";
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
bool isDetecting = true;
bool autoTrackingMode = false; // 自动跟踪模式

// Radar Protocol Constants
const String notTargetDataStr = "0000000000000000";
const int singleTargetDataLength = 16;
const int totalFrameLength = 60;

// Tracking Parameters
const double MIN_DISTANCE = 0.3; // 最小跟踪距离(米)
const double MAX_DISTANCE = 3.0; // 最大跟踪距离(米)
const double ANGLE_THRESHOLD = 15.0; // 角度阈值(度)

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
int hexToInt(String hex) {
    return (int)strtol(hex.c_str(), NULL, 16);
}

double calculateXValue(String str) {
    int byteLow = hexToInt(str.substring(0, 2));
    int byteHigh = hexToInt(str.substring(2, 4));
    int rawValue = byteLow + byteHigh * 256;

    if (byteHigh >= 128) {
        return (double)(rawValue - 32768);
    } else {
        return (double)(-rawValue);
    }
}

double calculateYValue(String str) {
    int byteLow = hexToInt(str.substring(4, 6));
    int byteHigh = hexToInt(str.substring(6, 8));
    int value = byteLow + byteHigh * 256;

    if (byteHigh >= 128) {
        return (double)(value - 32768);
    } else {
        return (double)(0 - value);
    }
}

double calculateSpeed(String str) {
    int byteLow = hexToInt(str.substring(8, 10));
    int byteHigh = hexToInt(str.substring(10, 12));
    int value = byteLow + byteHigh * 256;

    if (byteHigh >= 128) {
        return (double)(value & 0x7FFF);
    } else {
        return (double)(-value);
    }
}

double calculateResolution(String str) {
    int byteLow = hexToInt(str.substring(12, 14));
    int byteHigh = hexToInt(str.substring(14, 16));
    int value = byteLow + byteHigh * 256;
    return (double)value;
}

double calculateDistance(double x, double y) {
    return sqrt(x * x + y * y) / 1000.0; // 转换为米
}

double calculateAngle(double x, double y) {
    double angle = atan2(y, x) * (180.0 / PI);
    return -(90.0 - angle);
}

int findFrameStart(String data) {
    String frameHeader = "aaff0300";
    for (int i = 0; i <= (int)data.length() - 8; i += 2) {
        String possibleHeader = data.substring(i, i + 8);
        if (possibleHeader.equalsIgnoreCase(frameHeader)) {
            return i;
        }
    }
    return -1;
}

bool validateFrameTail(String data, int frameStart) {
    String frameTail = "55cc";
    int tailPosition = frameStart + 56;
    
    if (tailPosition + 4 > data.length()) {
        return false;
    }
    
    String actualTail = data.substring(tailPosition, tailPosition + 4);
    return actualTail.equalsIgnoreCase(frameTail);
}

void analysisAreaCoordinate(String strData, TagInfoBean results[], int &resultCount) {
    resultCount = 0;
    
    if (strData.length() < totalFrameLength) {
        return;
    }
    
    int dataStartIndex = 8;
    
    for (int i = 0; i < 3; i++) {
        int segmentStart = dataStartIndex + (i * singleTargetDataLength);
        String segment = strData.substring(segmentStart, segmentStart + singleTargetDataLength);
        
        if (segment != notTargetDataStr && segment.length() == singleTargetDataLength) {
            double x = calculateXValue(segment);
            double y = calculateYValue(segment);
            double speed = calculateSpeed(segment);
            double resolution = calculateResolution(segment);
            double distance = calculateDistance(x, y);
            double angle = calculateAngle(x, y);
            
            if (distance > 0.1 && distance < 10.0) { // 合理的距离范围
                results[resultCount].x = x;
                results[resultCount].y = y;
                results[resultCount].speed = speed;
                results[resultCount].resolution = resolution;
                results[resultCount].distance = distance;
                results[resultCount].angle = angle;
                results[resultCount].timestamp = millis();
                resultCount++;
            }
        }
    }
}

String readSerialData() {
    static String buffer = "";
    static unsigned long lastDataTime = 0;
    const unsigned long TIMEOUT_MS = 100;
    
    while (ld2450Serial.available()) {
        char c = ld2450Serial.read();
        buffer += String(c, HEX);
        lastDataTime = millis();
        
        if (buffer.length() >= totalFrameLength) {
            int frameStart = findFrameStart(buffer);
            if (frameStart >= 0) {
                String frame = buffer.substring(frameStart, frameStart + totalFrameLength);
                if (frame.length() == totalFrameLength && validateFrameTail(frame, 0)) {
                    buffer = buffer.substring(frameStart + totalFrameLength);
                    return frame;
                }
            }
            // 如果缓冲区太长但没有找到有效帧，清理旧数据
            if (buffer.length() > totalFrameLength * 2) {
                buffer = buffer.substring(totalFrameLength);
            }
        }
    }
    
    // 超时清理缓冲区
    if (millis() - lastDataTime > TIMEOUT_MS && buffer.length() > 0) {
        buffer = "";
    }
    
    return "";
}

// --- Auto Tracking Functions ---
void autoTrackTarget() {
    if (!autoTrackingMode || currentTargetCount == 0) {
        return;
    }
    
    // 找到最近的目标
    TagInfoBean* nearestTarget = nullptr;
    double minDistance = MAX_DISTANCE;
    
    for (int i = 0; i < currentTargetCount; i++) {
        if (currentTargets[i].distance >= MIN_DISTANCE && 
            currentTargets[i].distance <= MAX_DISTANCE) {
            if (currentTargets[i].distance < minDistance) {
                minDistance = currentTargets[i].distance;
                nearestTarget = &currentTargets[i];
            }
        }
    }
    
    if (nearestTarget == nullptr) {
        motorStop();
        return;
    }
    
    double angle = nearestTarget->angle;
    double distance = nearestTarget->distance;
    
    Serial.printf("Tracking target: angle=%.1f°, distance=%.2fm\n", angle, distance);
    
    // 根据角度调整方向
    if (abs(angle) > ANGLE_THRESHOLD) {
        if (angle > 0) {
            motorRight(); // 目标在右侧，向右转
        } else {
            motorLeft();  // 目标在左侧，向左转
        }
    } else {
        // 根据距离控制前进/后退
        if (distance > 1.0) {
            motorForward(); // 距离较远，前进
        } else if (distance < 0.5) {
            motorBackward(); // 距离太近，后退
        } else {
            motorStop(); // 距离合适，停止
        }
    }
}

// --- Web Server Setup ---
void setupWebServer() {
    // 静态文件服务
    server.serveStatic("/", LittleFS, "/").setDefaultFile("car.html");
    
    // 运动控制API
    server.on("/forward", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!autoTrackingMode) {
            motorForward();
        }
        request->send(200, "text/plain", "Moving Forward");
    });
    
    server.on("/backward", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!autoTrackingMode) {
            motorBackward();
        }
        request->send(200, "text/plain", "Moving Backward");
    });
    
    server.on("/left", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!autoTrackingMode) {
            motorLeft();
        }
        request->send(200, "text/plain", "Turning Left");
    });
    
    server.on("/right", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!autoTrackingMode) {
            motorRight();
        }
        request->send(200, "text/plain", "Turning Right");
    });
    
    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request) {
        motorStop();
        request->send(200, "text/plain", "Stopped");
    });
    
    server.on("/speed", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("value")) {
            int speed = request->getParam("value")->value().toInt();
            setMotorSpeed(speed);
            request->send(200, "text/plain", "Speed set to " + String(speed));
        } else {
            request->send(400, "text/plain", "Missing speed parameter");
        }
    });
    
    // 雷达数据API
    server.on("/radar-data", HTTP_GET, [](AsyncWebServerRequest *request) {
        DynamicJsonDocument doc(1024);
        JsonArray targets = doc.createNestedArray("targets");
        
        for (int i = 0; i < currentTargetCount; i++) {
            JsonObject target = targets.createNestedObject();
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
        request->send(200, "application/json", response);
    });
    
    // 自动跟踪控制
    server.on("/auto-tracking", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("enable")) {
            autoTrackingMode = request->getParam("enable")->value() == "true";
            if (!autoTrackingMode) {
                motorStop();
            }
            request->send(200, "text/plain", autoTrackingMode ? "Auto tracking enabled" : "Auto tracking disabled");
        } else {
            request->send(200, "text/plain", autoTrackingMode ? "enabled" : "disabled");
        }
    });
    
    server.begin();
    Serial.println("Web server started");
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
        Serial.println("LittleFS initialization failed");
        return;
    }
    Serial.println("LittleFS initialized");
    
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
    if (MDNS.begin(mdnsHostname)) {
        Serial.printf("mDNS started. Access at: http://%s.local\n", mdnsHostname);
    }
    
    // 初始化雷达串口
    ld2450Serial.begin(256000, SERIAL_8N1, 0, 1); // RX=0, TX=1
    Serial.println("LD2450 serial initialized");
    
    // 设置Web服务器
    setupWebServer();
    
    Serial.println("System ready!");
}

void loop() {
    // 读取雷达数据
    String radarData = readSerialData();
    if (radarData.length() > 0) {
        analysisAreaCoordinate(radarData, currentTargets, currentTargetCount);
        lastDetectionTime = millis();
        
        if (currentTargetCount > 0) {
            Serial.printf("Detected %d targets\n", currentTargetCount);
            for (int i = 0; i < currentTargetCount; i++) {
                Serial.printf("Target %d: x=%.1f, y=%.1f, distance=%.2fm, angle=%.1f°\n", 
                    i, currentTargets[i].x, currentTargets[i].y, 
                    currentTargets[i].distance, currentTargets[i].angle);
            }
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
    
    // 心跳信号
    if (millis() - lastHeartbeat > 10000) {
        Serial.printf("System running - WiFi: %s, Targets: %d, Auto-tracking: %s\n", 
            WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
            currentTargetCount,
            autoTrackingMode ? "ON" : "OFF");
        lastHeartbeat = millis();
    }
    
    delay(50); // 20Hz主循环
}

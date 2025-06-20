#include <Arduino.h>
#include <math.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <ESPmDNS.h>
#include "secrets.h" // 引入WiFi配置文件

// 创建Web服务器
AsyncWebServer server(WEB_SERVER_PORT);

// 创建第二个串口用于与LD2450通信
HardwareSerial ld2450Serial(1); // 使用UART1

struct TagInfoBean
{
  double x;           // X坐标，单位mm
  double y;           // Y坐标，单位mm
  double speed;       // 速度，单位cm/s
  double resolution;  // 距离分辨率，单位mm
  double distance;    // 计算得出的距离，单位m
  double angle;       // 计算得出的角度，单位度
  unsigned long timestamp;
};

// 全局变量
TagInfoBean currentTargets[3];
int currentTargetCount = 0;
unsigned long lastDetectionTime = 0;
unsigned long lastHeartbeat = 0; // 心跳计时器
bool isDetecting = true;

const String notTargetDataStr = "0000000000000000";
const int singleTargetDataLength = 16; // 每个目标8字节 = 16个十六进制字符
const int totalFrameLength = 60; // 30字节 = 60个十六进制字符 (帧头4 + 数据24 + 帧尾2)

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
  if (byteHigh >= 128) {
    // 正坐标：value - 2^15 (因为示例显示34481 - 32768 = 1713)
    return (double)(value - 32768);
  } else {
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
  if (byteHigh >= 128) {
    // 正向速度：取低15位
    return (double)(value & 0x7FFF); // 保留低15位
  } else {
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

// 验证帧尾
bool validateFrameTail(String data, int frameStart)
{
  // LD2450的帧尾: 55 CC (4个十六进制字符)
  String frameTail = "55cc";
  
  // 计算帧尾位置：帧头(8) + 目标数据(48) = 56个字符后
  int tailPosition = frameStart + 56;
  
  if (tailPosition + 4 > data.length())
  {
    Serial.println("Data too short for complete frame with tail");
    return false;
  }
  
  String actualTail = data.substring(tailPosition, tailPosition + 4);
  bool isValid = actualTail.equalsIgnoreCase(frameTail);
  
  Serial.print("Frame tail at position ");
  Serial.print(tailPosition);
  Serial.print(": ");
  Serial.print(actualTail);
  Serial.print(" (expected: ");
  Serial.print(frameTail);
  Serial.print(") - ");
  Serial.println(isValid ? "VALID" : "INVALID");
  
  return isValid;
}

void analysisAreaCoordinate(String strData, TagInfoBean results[], int &resultCount)
{
  resultCount = 0;
  
  // 由于readSerialData已经确保返回完整的30字节帧数据（从AAFF0300开始）
  // 不需要再次查找帧头，直接从第8个字符开始解析目标数据
  if (strData.length() < totalFrameLength) {
    return; // 数据不完整，直接返回
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

      results[resultCount].x = x;
      results[resultCount].y = y;
      results[resultCount].speed = speed;
      results[resultCount].resolution = resolution;
      results[resultCount].distance = distance;
      results[resultCount].angle = angle;
      results[resultCount].timestamp = millis();
      resultCount++;
      
      Serial.print("Valid target found - X: ");
      Serial.print(x);
      Serial.print("mm, Y: ");
      Serial.print(y);
      Serial.print("mm, Speed: ");
      Serial.print(speed);
      Serial.print("cm/s, Resolution: ");
      Serial.print(resolution);
      Serial.println("mm");
    }
    else
    {
      Serial.print("No target at position ");
      Serial.println(i + 1);
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
  while ((millis() - startTime < 100) && !frameFound && ld2450Serial.available()) {
    byte c = ld2450Serial.read();
    buffer[bufferIndex] = c;
    bytesChecked++;
    
    // 只有读取了至少4个字节后才开始检查帧头
    if (bytesChecked >= 4) {
      // 检查是否找到帧头 - 修正索引计算
      bool headerMatch = true;
      for (int i = 0; i < 4; i++) {
        int checkIndex = (bufferIndex + 1 + i) % 4; // 从下一个位置开始按顺序检查
        if (buffer[checkIndex] != frameHeader[i]) {
          headerMatch = false;
          break;
        }
      }
        if (headerMatch) {
        frameFound = true;
        // Serial.println("Frame header found!"); // 减少输出
      }
    }
    
    bufferIndex = (bufferIndex + 1) % 4;
      // 重置看门狗

  }
    if (!frameFound) {
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
  
  while ((millis() - startTime < 50) && remainingBytes > 0 && ld2450Serial.available()) {
    byte c = ld2450Serial.read();
    sprintf(&hexBuffer[index], "%02X", c);
    index += 2;
    remainingBytes--;
}
  
  hexBuffer[index] = '\0';
  String data = String(hexBuffer);
  if (data.length() > 0)
  {
    Serial.print("Frame: ");
    Serial.print((index / 2));
    Serial.println(" bytes");
    // Serial.println("Data: " + data); // 减少输出
  }

  return data;
}

// LD2450命令函数
void sendLD2450Command(String command)
{
  ld2450Serial.print(command);
  Serial.println("Sent command to LD2450: " + command);
}

void initLD2450()
{
  delay(1000); // 等待传感器稳定

  // 发送一些初始化命令（根据LD2450文档）
  sendLD2450Command("sensor\n"); // 请求传感器状态
  delay(100);
  sendLD2450Command("target\n"); // 请求目标检测数据
  delay(100);

  Serial.println("LD2450 initialization commands sent");
}

// 初始化WiFi
void initWiFi()
{
  if (USE_AP_MODE)
  {
    // 创建热点模式
    WiFi.softAP(RADAR_AP_SSID, RADAR_AP_PASSWORD);
    Serial.println("WiFi AP mode started");
    Serial.print("AP SSID: ");
    Serial.println(RADAR_AP_SSID);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
  }
  else
  {
    // 连接到现有WiFi网络
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println();
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
    }
    else
    {
      Serial.println();
      Serial.println("WiFi connection failed! Starting AP mode as fallback...");
      WiFi.softAP(RADAR_AP_SSID, RADAR_AP_PASSWORD);
      Serial.print("Fallback AP IP address: ");
      Serial.println(WiFi.softAPIP());
    }
  }
}

// 初始化LittleFS
void initLittleFS()
{
  if (!LittleFS.begin(true))
  {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("LittleFS mounted successfully");
}

// 初始化mDNS
void initMDNS()
{
  if (!MDNS.begin(MDNS_HOST_NAME))
  {
    Serial.println("Error setting up MDNS responder!");
    return;
  }
  
  // 添加服务
  MDNS.addService("http", "tcp", WEB_SERVER_PORT);
  MDNS.addServiceTxt("http", "tcp", "version", "1.0");
  MDNS.addServiceTxt("http", "tcp", "device", "LD2450-Radar");
  
  Serial.print("mDNS responder started: ");
  Serial.print(MDNS_HOST_NAME);
  Serial.println(".local");
}

// 保存目标数据到LittleFS
// void saveTargetData(TagInfoBean targets[], int count)
// {
//   if (count == 0)
//     return;

//   JsonDocument doc;
//   doc["timestamp"] = millis();
//   doc["count"] = count;
//   JsonArray targetsArray = doc["targets"].to<JsonArray>();
//   for (int i = 0; i < count; i++)
//   {
//     JsonObject target = targetsArray.add<JsonObject>();
//     target["x"] = targets[i].x;
//     target["y"] = targets[i].y;
//     target["speed"] = targets[i].speed;
//     target["resolution"] = targets[i].resolution;
//     target["distance"] = targets[i].distance;
//     target["angle"] = targets[i].angle;
//     target["timestamp"] = targets[i].timestamp;
//   }

//   // 打开文件追加写入
//   File file = LittleFS.open("/targets.json", "a");
//   if (file)
//   {
//     serializeJson(doc, file);
//     file.println();
//     file.close();
//   }
// }

// 获取存储的目标数据
String getStoredTargets()
{
  if (!LittleFS.exists("/targets.json"))
  {
    return "{\"success\":false,\"message\":\"No data found\"}";
  }

  File file = LittleFS.open("/targets.json", "r");
  if (!file)
  {
    return "{\"success\":false,\"message\":\"Failed to open file\"}";
  }

  String lastLine = "";
  while (file.available())
  {
    String line = file.readStringUntil('\n');
    if (line.length() > 0)
    {
      lastLine = line;
    }
  }
  file.close();

  if (lastLine.length() == 0)
  {
    return "{\"success\":false,\"message\":\"No valid data\"}";
  }
  // 解析最新的数据并重新格式化为API响应
  JsonDocument doc;
  deserializeJson(doc, lastLine);

  JsonDocument response;
  response["success"] = true;
  response["targets"] = doc["targets"];

  String result;
  serializeJson(response, result);
  return result;
}

// 清除存储的数据
void clearStoredData()
{
  if (LittleFS.exists("/targets.json"))
  {
    LittleFS.remove("/targets.json");
  }
}

// 获取所有历史数据用于下载
String getAllStoredData()
{
  if (!LittleFS.exists("/targets.json"))
  {
    return "[]";
  }

  File file = LittleFS.open("/targets.json", "r");
  if (!file)
  {
    return "[]";
  }

  String allData = "[";
  bool first = true;

  while (file.available())
  {
    String line = file.readStringUntil('\n');
    if (line.length() > 0)
    {
      if (!first)
      {
        allData += ",";
      }
      allData += line;
      first = false;
    }
  }
  file.close();

  allData += "]";
  return allData;
}

// 初始化Web服务器
void initWebServer()
{
  // 提供静态文件
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html", "text/html"); });
  // API: 获取当前目标数据 (实时)
  server.on("/api/targets", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    JsonDocument doc;
    doc["success"] = true;
    doc["timestamp"] = millis();
    doc["lastDetectionTime"] = lastDetectionTime;
    
    JsonArray targetsArray = doc["targets"].to<JsonArray>();
    for (int i = 0; i < currentTargetCount; i++)
    {
      JsonObject target = targetsArray.add<JsonObject>();
      target["x"] = currentTargets[i].x;
      target["y"] = currentTargets[i].y;
      target["speed"] = currentTargets[i].speed;
      target["resolution"] = currentTargets[i].resolution;
      target["distance"] = currentTargets[i].distance;
      target["angle"] = currentTargets[i].angle;
      target["timestamp"] = currentTargets[i].timestamp;
    }
    
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response); });

  // API: 清除历史数据
  server.on("/api/clear", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    clearStoredData();
    request->send(200, "application/json", "{\"success\":true}"); });

  // API: 下载所有数据
  server.on("/api/download", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String data = getAllStoredData();
    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", data);
    response->addHeader("Content-Disposition", "attachment; filename=radar_data.json");
    request->send(response); });

  // API: 开始/停止检测
  server.on("/api/detection/start", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    isDetecting = true;
    request->send(200, "application/json", "{\"success\":true,\"message\":\"Detection started\"}"); });
  server.on("/api/detection/stop", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    isDetecting = false;
    request->send(200, "application/json", "{\"success\":true,\"message\":\"Detection stopped\"}"); });
  // API: 获取检测状态
  server.on("/api/detection/status", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    JsonDocument doc;
    doc["isDetecting"] = isDetecting;
    doc["lastDetectionTime"] = lastDetectionTime;
    doc["currentTargetCount"] = currentTargetCount;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response); });

  // API: 获取实时系统状态
  server.on("/api/realtime/status", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    JsonDocument doc;
    doc["success"] = true;
    doc["uptime"] = millis();
    doc["isDetecting"] = isDetecting;
    doc["lastDetectionTime"] = lastDetectionTime;
    doc["currentTargetCount"] = currentTargetCount;
    doc["serialAvailable"] = ld2450Serial.available();
    doc["freeHeap"] = ESP.getFreeHeap();
    
    // 如果有目标，添加最近的目标信息
    if (currentTargetCount > 0) {
      JsonObject latestTarget = doc["latestTarget"].to<JsonObject>();
      latestTarget["x"] = currentTargets[0].x;
      latestTarget["y"] = currentTargets[0].y;
      latestTarget["distance"] = currentTargets[0].distance;
      latestTarget["angle"] = currentTargets[0].angle;
    }
    
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response); });
  // 启动服务器
  server.begin();
  Serial.print("Web server started on port: ");
  Serial.println(WEB_SERVER_PORT);
  Serial.print("mDNS hostname: ");
  Serial.print(MDNS_HOST_NAME);
  Serial.println(".local");
}

void setup()
{
  Serial.begin(115200);                         // 用于调试输出
  ld2450Serial.begin(256000, SERIAL_8N1, 1, 0); // LD2450: RX=GPIO1, TX=GPIO0

  // 禁用任务看门狗，避免与异步Web服务器冲突
  esp_task_wdt_deinit(); // 完全禁用看门狗

  Serial.println("=== ESP32-C3 LD2450 Radar Sensor Started ===");
  Serial.println("Debug output: Serial (115200 baud)");
  Serial.println("LD2450 connection: UART1 (256000 baud, 8N1)");
  Serial.println("Wiring: LD2450_TX -> ESP32_GPIO1, LD2450_RX -> ESP32_GPIO0");
  Serial.println("Power: LD2450 VCC=3.3V, GND=GND");
  Serial.println("Task watchdog disabled");
  Serial.println("==========================================");
  // 初始化LD2450传感器
  // initLD2450();


  // 初始化WiFi
  initWiFi();

  // 初始化文件系统
  initLittleFS();
  
  // 初始化mDNS (只在WiFi连接成功时)
  if (WiFi.status() == WL_CONNECTED || WiFi.getMode() == WIFI_AP) {
    initMDNS();
  }
  
  // 初始化Web服务器
  initWebServer();
  Serial.println("Waiting for sensor data...");
  Serial.println("==========================================");
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Web interface available at: http://" + WiFi.localIP().toString());
    Serial.println("Or access via: http://" + String(MDNS_HOST_NAME) + ".local");
  }
  else
  {
    Serial.println("Web interface available at: http://" + WiFi.softAPIP().toString());
    Serial.println("Connect to WiFi AP: " + String(RADAR_AP_SSID));
  }
  Serial.println("==========================================");
}

void loop()
{  
  // 简化心跳输出
  if (millis() - lastHeartbeat > 10000) // 每10秒输出一次心跳
  {
    Serial.print("Heartbeat - Detection: ");
    Serial.println(isDetecting ? "ON" : "OFF");
    lastHeartbeat = millis();
  }
  
  // 给其他任务执行时间
  yield();
    // 只有在检测启用时才处理数据
  if (isDetecting && ld2450Serial.available() > 0) // 检查LD2450串口
  {
    Serial.println("Data detected!");
    
    String serialData = readSerialData();

    if (serialData.length() < totalFrameLength) // LD2450完整帧需要60个十六进制字符（30字节）
    {
      Serial.print("Data too short: ");
      Serial.print(serialData.length());
      Serial.print(" (need minimum ");
      Serial.print(totalFrameLength);
      Serial.println(")");
      return; // 早期返回
    }    TagInfoBean results[3];
    int count;
    
    analysisAreaCoordinate(serialData, results, count);
    Serial.print("Parsing completed, found targets: ");
    Serial.println(count);
    
    if (count > 0)
    {
      // 更新当前目标数据
      currentTargetCount = count;
      for (int i = 0; i < count; i++)
      {
        currentTargets[i] = results[i];
      }
      lastDetectionTime = millis();      // 保存到LittleFS
      // saveTargetData(results, count);

      Serial.print("Found ");
      Serial.print(count);
      Serial.println(" targets");
      for (int i = 0; i < count; i++)
      {
        // 简化输出，减少串口阻塞时间
        Serial.print("Target ");
        Serial.print(i + 1);
        Serial.print(" - X:");
        Serial.print(results[i].x);
        Serial.print("mm Y:");
        Serial.print(results[i].y);
        Serial.print("mm Speed:");
        Serial.print(results[i].speed);
        Serial.print("cm/s Res:");
        Serial.print(results[i].resolution);
        Serial.println("mm");

      }
    }
    else
    {
      Serial.println("No valid targets detected in this frame");
      currentTargetCount = 0;
    }
    Serial.println("Processing completed.");
  }
    // 如果检测被禁用但有数据到达，快速清空缓冲区
  else if (!isDetecting && ld2450Serial.available() > 0)
  {
    int cleared = 0;
    while (ld2450Serial.available() && cleared < 50) // 限制清理数量，防止阻塞
    {
      ld2450Serial.read();
      cleared++;
    }
  }  
  // 强制延迟和看门狗重置，确保其他任务有执行时间
  esp_task_wdt_reset();
  delay(20); // 增加延迟，给Web服务器更多时间
}


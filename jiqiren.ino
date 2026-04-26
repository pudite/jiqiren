#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <FluxGarage_RoboEyes.h>
#include <Adafruit_VL53L0X.h>
// RoboEyes.h 定义了 N 和 E 宏，与 mbedtls/ArduinoJson 冲突，需要取消定义
#undef N
#undef E
#include "sntp.h"
#include <EEPROM.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool sensorOK = false;            // 传感器状态
uint16_t baseDistance = 0;   // 基准距离
uint16_t edgeThreshold = 0;   // 防跌落阈值
uint16_t obstacleThreshold = 0;      // 障碍阈值
bool avoidingObstacle = false;       // 避障动作进行中标志
unsigned long avoidStartTime = 0;    // 避障动作计时
int avoidStep = 0;                   // 避障状态机步骤：0=空闲,1=停止,2=后退,3=转向准备,4=转向,5=完成
unsigned long scanStartTime = 0;    // 旋转扫描开始时间
int scanDirection = 1;              // 旋转方向：0左转，1右转（默认右转）
int rotateStep = 0;                 // 当前旋转步数（0~12）
const int MAX_ROTATE_STEPS = 12;    // 最大旋转步数（30°×12=360°）
const unsigned long ROTATE_TIME_PER_STEP = 50; // 每30°旋转时间（ms），可根据实测调整
bool movingForward = false;               // 是否正在前进
bool sleepByObstacle = false; // 是否为避障超时导致的睡眠
const uint8_t randomMoods[] = {HAPPY, ANGRY, TIRED};
const int moodCount = 3;
const unsigned long DANCE_HALF_TURN_TIME = 200;   // 小半圈时间 (ms)
const unsigned long DANCE_FULL_TURN_TIME = 600;   // 一整圈时间 (ms)，可根据实际调整
const unsigned long RANDOM_STABLE_DELAY = 150; // 随机动作完成后等待惯性消失的时间（毫秒）

// 边缘逃避状态机
int safeSeekState = 0; // 0=旋转中, 1=检测中
bool seekingSafeDir = false;          // 是否正在寻找安全方向
int safeSeekStep = 0;                 // 当前旋转步数
unsigned long safeSeekStartTime = 0;  // 旋转开始时间
const int SAFE_SEEK_STEPS = 12;       // 最多旋转12步（30°×12=360°）
const unsigned long SAFE_ROTATE_TIME = 50; // 每步旋转时间（与避障一致）

// 随机动作状态机
bool randomActive = false;          // 是否正在执行随机动作
int randomStep = 0;                  // 0=空闲,1=动作中,2=停顿中
byte randomCmd = 0;                  // 当前动作指令
int randomT1 = 0;                     // 动作持续时间
int randomT2 = 0;                     // 停顿时间
int randomRepeat = 0;                 // 剩余循环次数
unsigned long randomActionTime = 0;   // 动作/停顿开始时间

// 防跌落动作状态机
// 防跌落动作模式：0=手动模式（仅后退停止），1=好奇模式（后退+转向）
uint8_t fallMode = 0;
// 新增状态
enum FallState { 
  FALL_IDLE, 
  FALL_STOP, 
  FALL_BACKWARD, 
  FALL_TURN,      // 转向准备
  FALL_TURNING,   // 转向中
  FALL_DONE 
};
bool fallLock = false;  // 防跌落锁定，用于阻止长按前进的反复
FallState fallState = FALL_IDLE;
unsigned long fallActionTime = 0;

// 滤波状态机变量（替代原来的 getFilteredDistance）
enum FilterState { FILTER_IDLE, FILTER_SAMPLING };
FilterState filterState = FILTER_IDLE;
int filterIndex = 0;
uint16_t filterValues[5];
unsigned long lastFilterTime = 0;
uint16_t latestDistance = 0;  // 最新滤波后的距离
// 快速校准基准距离（用于恢复场景）
// 采样期间检测稳定性，确保小车已放稳
// 返回 true 表示校准成功，false 表示数据不稳定
bool quickCalibrateBase() {
  if (!sensorOK) return false;
  const int samples = 10;
  uint32_t sumDist = 0;
  int validCount = 0;
  uint16_t minD = 65535, maxD = 0;
  for (int i = 0; i < samples; i++) {
    uint16_t d = getDistance();
    if (d < 2000) {
      sumDist += d;
      validCount++;
      if (d < minD) minD = d;
      if (d > maxD) maxD = d;
    }
    delay(30);
  }
  // 稳定性检查：最大最小差距不能超过20mm
  if (validCount >= samples - 1 && (maxD - minD) <= 20) {
    baseDistance = sumDist / validCount;
    edgeThreshold = baseDistance + 20;
    obstacleThreshold = baseDistance - 20;
    if (obstacleThreshold < 30) obstacleThreshold = 30;
    Serial.print("快速校准完成 - 新基准: ");
    Serial.print(baseDistance);
    Serial.print(" mm, 障碍阈值: ");
    Serial.println(obstacleThreshold);
    latestDistance = baseDistance;
    return true;
  } else {
    Serial.print("快速校准失败：数据不稳定(maxD-minD=");
    Serial.print(maxD - minD);
    Serial.print("mm, valid=");
    Serial.print(validCount);
    Serial.println(")，将在下次唤醒重试");
    return false;
  }
}

// 非阻塞滤波，需要在 loop 中频繁调用，有新数据时更新 latestDistance
void updateFilter() {
  const int samples = 5;
  
  if (filterState == FILTER_IDLE) {
    filterIndex = 0;
    filterState = FILTER_SAMPLING;
    lastFilterTime = millis();
    // 立即采第一个点
    filterValues[0] = getDistance();
    filterIndex = 1;
    return;
  }
  
  if (filterState == FILTER_SAMPLING) {
    if (millis() - lastFilterTime >= 10) { // 每10ms采样一次
      if (filterIndex < samples) {
        filterValues[filterIndex] = getDistance();
        filterIndex++;
        lastFilterTime = millis();
      }
      if (filterIndex >= samples) {
        // 采样完成，排序去极值平均
        for (int i = 0; i < samples-1; i++) {
          for (int j = i+1; j < samples; j++) {
            if (filterValues[i] > filterValues[j]) {
              uint16_t temp = filterValues[i];
              filterValues[i] = filterValues[j];
              filterValues[j] = temp;
            }
          }
        }
        uint32_t sum = 0;
        for (int i = 1; i < samples-1; i++) {
          sum += filterValues[i];
        }
        latestDistance = sum / (samples - 2);
        filterState = FILTER_IDLE;
      }
    }
  }
}

/* =================代码原作者为Huy Vector================= */
/* ================= 千秋我不见 仅转译优化================= */

/* ================= ASR Pro2 语音模块配置 ================= */
#define ASR_RX_PIN 4   // ESP32的 GPIO4 (RX1)，连接 ASR Pro2 的 PB5 (UART0_TX)
#define ASR_TX_PIN 5   // ESP32的 GPIO5 (TX1)，连接 ASR Pro2 的 PB6 (UART0_RX)（备用）
#define ASR_BAUDRATE 9600

HardwareSerial ASRSerial(1);
String voiceCommand = "";
bool voiceCommandComplete = false;

// 语音动作控制变量
bool voiceActionActive = false;
bool voiceSequenceLock = false;  // 新增：序列动作锁，防止中断
unsigned long voiceActionStartTime = 0;
unsigned long voiceActionDuration = 0;
byte voiceActionType = 0; // 1前进 2后退 3左转 4右转 5跳舞 6唱歌

// 序列动作状态
typedef struct {
  byte step;
  byte loopCount;
  unsigned long stepTime;
  bool inSequence;
  bool resetFlag;
} SequenceState;

SequenceState danceState = {0, 0, 0, false, false};
SequenceState singState = {0, 0, 0, false, false};

/* ================= OLED显示屏配置 ================= */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_SDA 8
#define OLED_SCL 9
#define OLED_ADDR 0x3C

// U8g2 统一显示驱动（替代 Adafruit_SSD1306）
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

class OledDisplayWrapper {
public:
  U8G2& u8g2_dev;

  OledDisplayWrapper() : u8g2_dev(u8g2) {}

  bool begin(uint8_t addr = 0x3C, bool reset = true) {
    Wire.begin(OLED_SDA, OLED_SCL);
    Wire.setClock(400000);
    u8g2_dev.setI2CAddress(addr * 2); // 7-bit to 8-bit
    bool result = u8g2_dev.begin();
    u8g2_dev.enableUTF8Print();
    u8g2_dev.setFont(u8g2_font_6x10_tf); // 默认字体，防止 print() 时空指针崩溃
    return result;
  }

  void clearDisplay()        { u8g2_dev.clearBuffer(); u8g2_dev.setDrawColor(1); }
  void setCursor(int16_t x, int16_t y) { u8g2_dev.setCursor(x, y); }
  void setTextSize(uint8_t s)  { /* U8g2 uses setFont for sizing; map sizes approximately */
    if (s == 1) u8g2_dev.setFont(u8g2_font_6x10_tf);
    else if (s >= 2) u8g2_dev.setFont(u8g2_font_logisoso24_tr);
  }
  void setTextColor(uint16_t c){ u8g2_dev.setDrawColor(c); }
  size_t print(const char* s)  { return u8g2_dev.print(s); }
  size_t print(const String& s){ return u8g2_dev.print(s); }
  template<typename T> size_t print(T v) { return u8g2_dev.print(v); }
  template<typename T> size_t println(T v) { return u8g2_dev.println(v); }
  size_t println() { return u8g2_dev.println(); }
  void display()               { u8g2_dev.sendBuffer(); }
  void drawPixel(int16_t x, int16_t y, uint16_t color) { u8g2_dev.setDrawColor(color); u8g2_dev.drawPixel(x, y); }
  void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color = 1) { u8g2_dev.setDrawColor(color); u8g2_dev.drawLine(x0, y0, x1, y1); }
  void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color = 1) { u8g2_dev.setDrawColor(color); u8g2_dev.drawCircle(x0, y0, r); }
  void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color = 1) { u8g2_dev.setDrawColor(color); u8g2_dev.drawDisc(x0, y0, r); }
  void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color = 1) {
    // U8g2 drawRBox 参数为 uint16_t，负值/下溢会导致图形异常
    // roboEyes 眨眼时 h 可接近0且 r>h/2，无符号减法 hh=h-r-r 会下溢为巨大值
    u8g2_dev.setDrawColor(color);
    if (w <= 0 || h <= 0) return;
    if (r < 0) r = 0;
    // 圆角半径不能超过宽高的一半，否则 U8g2 内部无符号减法下溢
    if (r > w / 2) r = w / 2;
    if (r > h / 2) r = h / 2;
    if (x < 0 || y < 0) {
      // 负坐标：用 drawBox 替代，避免传负值给 U8g2
      u8g2_dev.drawBox(x < 0 ? 0 : x, y < 0 ? 0 : y, w, h);
      return;
    }
    u8g2_dev.drawRBox(x, y, w, h, r);
  }
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color = 1) { u8g2_dev.setDrawColor(color); u8g2_dev.drawBox(x, y, w, h); }
  void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color = 1) {
    // U8g2 drawTriangle 只画边框，需手动扫描线填充
    u8g2_dev.setDrawColor(color);
    if (y0 > y1) { int16_t t; t=x0; x0=x1; x1=t; t=y0; y0=y1; y1=t; }
    if (y1 > y2) { int16_t t; t=x1; x1=x2; x2=t; t=y1; y1=y2; y2=t; }
    if (y0 > y1) { int16_t t; t=x0; x0=x1; x1=t; t=y0; y0=y1; y1=t; }
    if (y0 == y2) {
      int16_t mn = x0, mx = x0;
      if (x1 < mn) mn = x1; if (x1 > mx) mx = x1;
      if (x2 < mn) mn = x2; if (x2 > mx) mx = x2;
      u8g2_dev.drawHLine(mn, y0, mx - mn + 1);
      return;
    }
    for (int16_t y = y0; y <= y2; y++) {
      int16_t xa = (y2 != y0) ? x0 + (int32_t)(x2 - x0) * (y - y0) / (y2 - y0) : x0;
      int16_t xb;
      if (y <= y1) xb = (y1 != y0) ? x0 + (int32_t)(x1 - x0) * (y - y0) / (y1 - y0) : (y == y0 ? x0 : x1);
      else xb = (y2 != y1) ? x1 + (int32_t)(x2 - x1) * (y - y1) / (y2 - y1) : x1;
      if (xa > xb) { int16_t t = xa; xa = xb; xb = t; }
      u8g2_dev.drawHLine(xa, y, xb - xa + 1);
    }
  }
  int16_t width()  { return SCREEN_WIDTH; }
  int16_t height() { return SCREEN_HEIGHT; }
};

OledDisplayWrapper display;
RoboEyes<OledDisplayWrapper> roboEyes(display);

/* ================= 电机引脚定义 ================= */
#define LF   0
#define LB   1
#define RF   2
#define RB   3
#define STBY 10

/* ================= WiFi配置 ================= */
WebServer server(80);
DNSServer dnsServer;

/* ================= 状态变量 ================= */
volatile bool manualActive = false;  // 手动控制激活标志
volatile int currentCommand = 0;     // 当前执行的命令
unsigned long lastCommandTime = 0;   // 上次命令时间
const unsigned long COMMAND_TIMEOUT = 60000; // 命令超时时间(ms) - 60秒

/* ================= 随机模式 ================= */
enum RandomMode {
  RANDOM_OFF,     // 关闭随机模式
  RANDOM_SOFT,    // 柔和随机模式
  RANDOM_NORMAL,  // 正常随机模式
  RANDOM_CLOCK    // 时间模式
};

volatile RandomMode randomMode = RANDOM_NORMAL;  // 当前随机模式

/* ================= 时间模式配置 ================= */
// EEPROM 配置存储
#define EEPROM_SIZE 256
#define EEPROM_MAGIC 0xA5

typedef struct {
  char magic;
  char ssid[33];
  char password[65];
  char apiKey[65];
  char city[33];
} WifiConfig;

char routerSSID[33] = "";
char routerPassword[65] = "";
char weatherApiKey[65] = "";
char weatherCity[33] = "Beijing";

bool routerConnected = false;
bool clockModeInitialized = false;  // 标记时间模式是否已初始化WiFi连接
bool timeSynced = false;
struct tm timeinfo;
char timeStr[16] = "00:00:00";
char dateStr[24] = "----/--/--";
float currentTemp = 0;
char weatherDesc[32] = "";
char weatherIcon[4] = "";
unsigned long lastWeatherFetch = 0;
unsigned long lastClockUpdate = 0;
bool locationDetected = false;
float detectedLat = 0;
float detectedLon = 0;
char detectedCity[32] = "";  // IP定位获取的城市名
char ipBuf[48] = "";

const char* NTP_SERVERS[] = {"cn.pool.ntp.org", "pool.ntp.org"};
const int NTP_SERVER_COUNT = 2;
const long GMT_OFFSET_SEC = 8 * 3600;  // UTC+8
const int DAYLIGHT_OFFSET_SEC = 0;

/* ================= 时间模式函数 ================= */
void initSNTP() {
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVERS[0], NTP_SERVERS[1]);
  Serial.println("SNTP 初始化完成");
}

bool getTime() {
  if (!getLocalTime(&timeinfo)) {
    return false;
  }
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  snprintf(dateStr, sizeof(dateStr), "%04d/%02d/%02d", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
  return true;
}

void updateClockDisplay() {
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);

  if (!getTime()) {
    timeSynced = false;
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(0, 14);
    u8g2.print("Time --:--:--");
    if (!routerConnected) {
      u8g2.setCursor(0, 30);
      u8g2.print("WiFi: not connected");
    } else {
      u8g2.setCursor(0, 30);
      u8g2.print("Syncing time...");
    }
    u8g2.sendBuffer();
    return;
  }
  timeSynced = true;

  const char* cityName = (strlen(weatherCity) > 0) ? weatherCity : ((strlen(detectedCity) > 0) ? detectedCity : "");
  bool hasWeather = (strlen(weatherDesc) > 0);
  bool hasCity = (strlen(weatherCity) > 0 || strlen(detectedCity) > 0);

  // --- 时间（大字） ---
  u8g2.setFont(u8g2_font_logisoso24_tr);
  u8g2.setCursor(8, 26);
  u8g2.print(timeStr);

  // --- 日期 ---
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(0, 38);
  u8g2.print(dateStr);

  // --- 城市 + 温度 ---
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(0, 48);
  if (hasCity || hasWeather) {
    u8g2.print(cityName);
    if (hasWeather) {
      u8g2.print(" ");
      u8g2.print(currentTemp, 1);
      u8g2.print("°C");
    }
  }

  // --- 天气描述 - U8g2 中文直接显示 ---
  if (hasWeather) {
    u8g2.setFont(u8g2_font_wqy12_t_gb2312);
    u8g2.setCursor(0, 63);
    u8g2.print(weatherDesc);
  }

  u8g2.sendBuffer();
}

void detectLocation() {
  // 已移除IP定位，直接使用用户配置的城市
  Serial.println("IP定位已禁用，使用配置城市");
}

void fetchWeather() {
  if (!routerConnected) {
    Serial.println("天气获取跳过: 路由器未连接");
    return;
  }
  if (strlen(weatherApiKey) == 0) {
    Serial.println("天气获取跳过: API Key未设置");
    return;
  }
  if (strlen(weatherCity) == 0) {
    Serial.println("天气获取跳过: 未设置城市");
    return;
  }

  String url = "http://api.openweathermap.org/data/2.5/weather?";
  url += "q=" + String(weatherCity);
  Serial.print("使用城市名获取天气: ");
  Serial.println(weatherCity);
  url += "&appid=";
  url += weatherApiKey;
  url += "&units=metric&lang=zh_cn";

  Serial.print("天气请求: "); Serial.println(url);
  HTTPClient http;
  http.setTimeout(10000);
  http.begin(url);
  int httpCode = http.GET();
  Serial.print("天气HTTP返回: "); Serial.println(httpCode);

  if (httpCode == 200) {
    String payload = http.getString();
    Serial.print("天气返回: "); Serial.println(payload);
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      currentTemp = doc["main"]["temp"].as<float>();
      const char* desc = doc["weather"][0]["description"].as<const char*>();
      const char* icon = doc["weather"][0]["icon"].as<const char*>();
      if (desc) strncpy(weatherDesc, desc, sizeof(weatherDesc) - 1);
      if (icon) strncpy(weatherIcon, icon, sizeof(weatherIcon) - 1);
      Serial.print("天气: "); Serial.print(currentTemp, 1);
      Serial.print("C "); Serial.println(weatherDesc);
    } else {
      Serial.print("天气JSON解析失败: ");
      Serial.println(error.f_str());
    }
  } else {
    Serial.print("天气请求失败: ");
    Serial.println(httpCode);
  }
  http.end();
}

/* ================= EEPROM配置存取 ================= */
void saveConfigToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  WifiConfig cfg;
  cfg.magic = EEPROM_MAGIC;
  strncpy(cfg.ssid, routerSSID, sizeof(cfg.ssid) - 1);
  strncpy(cfg.password, routerPassword, sizeof(cfg.password) - 1);
  strncpy(cfg.apiKey, weatherApiKey, sizeof(cfg.apiKey) - 1);
  strncpy(cfg.city, weatherCity, sizeof(cfg.city) - 1);
  EEPROM.put(0, cfg);
  EEPROM.commit();
  EEPROM.end();
  Serial.println("配置已保存到EEPROM");
}

void loadConfigFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  WifiConfig cfg;
  EEPROM.get(0, cfg);
  EEPROM.end();

  if (cfg.magic == EEPROM_MAGIC) {
    strncpy(routerSSID, cfg.ssid, sizeof(routerSSID) - 1);
    strncpy(routerPassword, cfg.password, sizeof(routerPassword) - 1);
    strncpy(weatherApiKey, cfg.apiKey, sizeof(weatherApiKey) - 1);
    strncpy(weatherCity, cfg.city, sizeof(weatherCity) - 1);
    Serial.print("从EEPROM加载配置 - WiFi: ");
    Serial.println(routerSSID);
  } else {
    Serial.println("EEPROM无配置");
  }
}

void connectToRouter() {
  if (strlen(routerSSID) == 0) return;

  wl_status_t status = WiFi.status();

  // 如果已连接，直接返回
  if (status == WL_CONNECTED) {
    Serial.println("WiFi已连接，无需重连");
    routerConnected = true;
    return;
  }

  // 如果正在连接中，先断开，给一个干净的状态
  if (status != WL_DISCONNECTED) {
    Serial.print("WiFi状态异常(");
    Serial.print(status);
    Serial.println(")，先断开重连...");
    WiFi.disconnect(true);
    delay(500);
  }

  // 开始连接
  Serial.print("连接路由器WiFi: ");
  Serial.println(routerSSID);
  WiFi.begin(routerSSID, routerPassword);

  // 等待连接
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40) {
    delay(250);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    routerConnected = true;
    Serial.println("\n已连接路由器");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.println("等待网络栈稳定...");
    delay(2000);
  } else {
    routerConnected = false;
    Serial.println("\n路由器连接失败");
  }
}

/* ================= WiFi控制电机函数 ================= */
void motorWifi(byte c) {
   // 如果正在防跌落过程中，忽略前进命令
  if (c == 1 && fallLock) {
    // 强制停止电机
    digitalWrite(LF, LOW); digitalWrite(LB, LOW);
    digitalWrite(RF, LOW); digitalWrite(RB, LOW);
    movingForward = false;
    return;
  }
  
  digitalWrite(STBY, HIGH);
  switch (c) {
    case 0:
      digitalWrite(LF, LOW); digitalWrite(LB, LOW);
      digitalWrite(RF, LOW); digitalWrite(RB, LOW);
      movingForward = false;
      break;
    case 1:
      digitalWrite(LF, HIGH); digitalWrite(LB, LOW);
      digitalWrite(RF, LOW);  digitalWrite(RB, HIGH);
      movingForward = true;
      break;
    case 2:
      digitalWrite(LF, LOW);  digitalWrite(LB, HIGH);
      digitalWrite(RF, HIGH); digitalWrite(RB, LOW);
      movingForward = false;
      break;
    case 3:
      digitalWrite(LF, LOW);  digitalWrite(LB, HIGH);
      digitalWrite(RF, LOW);  digitalWrite(RB, HIGH);
      movingForward = false;
      break;
    case 4:
      digitalWrite(LF, HIGH); digitalWrite(LB, LOW);
      digitalWrite(RF, HIGH); digitalWrite(RB, LOW);
      movingForward = false;
      break;
  }
}

/* ================= 停止所有电机 ================= */
void stopAllMotors() {
  motorWifi(0);
  manualActive = false;
  currentCommand = 0;
  fallLock = false;  // 用户主动停止，解除防跌落锁定
}

/* ================= 随机控制电机函数 ================= */
void MOTOR(byte c, int t1, int t2, int Time) {
  for (int i = 0; i < Time; i++) {
    switch (c) {
      case 0: digitalWrite(LF, LOW); digitalWrite(LB, LOW); digitalWrite(RF, LOW); digitalWrite(RB, LOW); break;  // 停止
      case 1: digitalWrite(LF, LOW); digitalWrite(LB, HIGH); digitalWrite(RF, LOW); digitalWrite(RB, HIGH); break; // 后退
      case 2: digitalWrite(LF, HIGH); digitalWrite(LB, LOW); digitalWrite(RF, HIGH); digitalWrite(RB, LOW); break; // 前进
      case 3: digitalWrite(LF, LOW); digitalWrite(LB, HIGH); digitalWrite(RF, HIGH); digitalWrite(RB, LOW); break; // 左转
      case 4: digitalWrite(LF, HIGH); digitalWrite(LB, LOW); digitalWrite(RF, LOW); digitalWrite(RB, HIGH); break; // 右转
      case 5: digitalWrite(LF, LOW); digitalWrite(LB, HIGH); digitalWrite(RF, LOW); digitalWrite(RB, LOW); break; // 左轮后退
      case 6: digitalWrite(LF, LOW); digitalWrite(LB, LOW); digitalWrite(RF, LOW); digitalWrite(RB, HIGH); break; // 右轮后退
      case 7: digitalWrite(LF, HIGH); digitalWrite(LB, LOW); digitalWrite(RF, LOW); digitalWrite(RB, LOW); break; // 左轮前进
      case 8: digitalWrite(LF, LOW); digitalWrite(LB, LOW); digitalWrite(RF, HIGH); digitalWrite(RB, LOW); break; // 右轮前进
    }

    delay(t1);  // 动作持续时间

    // 停止电机
    digitalWrite(LF, LOW); digitalWrite(LB, LOW);
    digitalWrite(RF, LOW); digitalWrite(RB, LOW);

    delay(t2);  // 停止时间
  }
}

/* ================= 检查并停止超时命令 ================= */
void checkCommandTimeout() {
  if (manualActive && (millis() - lastCommandTime > COMMAND_TIMEOUT)) {
    motorWifi(0);  // 停止电机
    manualActive = false;
    fallLock = false;  // 命令超时，解除锁定
    Serial.println("命令超时，已停止");
  }
}

/* ================= 处理语音动作序列 ================= */
void handleVoiceActionSequence() {
  if (!voiceActionActive) return;
  
  unsigned long currentTime = millis();
  
  // 检查序列动作锁
  if (voiceSequenceLock) {
    return; // 如果序列动作被锁定，等待解锁
  }
  
  // 检查重置标志
  if (voiceActionType == 5 && danceState.resetFlag) {
    voiceSequenceLock = true; // 锁定序列动作
    delay(50); // 短暂延迟确保稳定
    roboEyes.setMood(DEFAULT);             // 恢复默认表情
    danceState = {0, 0, 0, false, false};
    voiceActionActive = false;
    manualActive = false;
    voiceSequenceLock = false; // 解锁
    Serial.println("跳舞动作被重置");
    return;
  }
  
  if (voiceActionType == 6 && singState.resetFlag) {
    voiceSequenceLock = true; // 锁定序列动作
    delay(50); // 短暂延迟确保稳定
    singState = {0, 0, 0, false, false};
    voiceActionActive = false;
    manualActive = false;
    voiceSequenceLock = false; // 解锁
    Serial.println("唱歌动作被重置");
    return;
  }
  
  if (voiceActionType == 5 && danceState.inSequence) { // 跳舞
    voiceSequenceLock = true; // 锁定序列动作
    
    switch (danceState.step) {
        case 0: // 第一步：左转小半圈
            motorWifi(3); // 左转
            danceState.stepTime = currentTime;
            danceState.step = 1;
            Serial.println("跳舞：左转小半圈");
            break;
            
        case 1: // 左转持续 DANCE_HALF_TURN_TIME
            if (currentTime - danceState.stepTime >= DANCE_HALF_TURN_TIME) {
                motorWifi(0); // 立即停止
                danceState.stepTime = currentTime;
                danceState.step = 2;
            }
            break;
            
        case 2: // 右转小半圈
            motorWifi(4); // 右转
            danceState.stepTime = currentTime;
            danceState.step = 3;
            Serial.println("跳舞：右转小半圈");
            break;
            
        case 3: // 右转持续 DANCE_HALF_TURN_TIME
            if (currentTime - danceState.stepTime >= DANCE_HALF_TURN_TIME) {
                motorWifi(0); // 立即停止
                danceState.stepTime = currentTime;
                danceState.step = 4; // 进入整圈旋转
            }
            break;
            
        case 4: // 整圈旋转（这里选择右转一整圈，也可改为左转）
            motorWifi(4); // 右转一整圈
            danceState.stepTime = currentTime;
            danceState.step = 5;
            Serial.println("跳舞：整圈旋转");
            break;
            
        case 5: // 整圈旋转持续 DANCE_FULL_TURN_TIME
            if (currentTime - danceState.stepTime >= DANCE_FULL_TURN_TIME) {
                motorWifi(0); // 停止
                danceState.stepTime = currentTime;
                danceState.loopCount++; // 增加循环计数
                Serial.print("跳舞：完成第");
                Serial.print(danceState.loopCount);
                Serial.println("次循环");
                danceState.step = 6; // 进入循环检查
            }
            break;
            
        case 6: // 检查是否完成循环
            if (danceState.loopCount < 2) { // 需要循环2次
                danceState.step = 0; // 重新开始左转
            } else {
                roboEyes.setMood(DEFAULT);         // 恢复默认表情
                // 跳舞完成
                delay(100); // 完成延迟
                voiceActionActive = false;
                danceState = {0, 0, 0, false, false};
                manualActive = false;
                Serial.println("跳舞完成");
            }
            break;
    }
    
    voiceSequenceLock = false; // 解锁序列动作
}
  
  else if (voiceActionType == 6 && singState.inSequence) { // 唱歌
    voiceSequenceLock = true; // 锁定序列动作
    
    switch (singState.step) {
      case 0: // 第一步：左转一圈（速度减半）
        motorWifi(3); // 左转
        singState.stepTime = currentTime;
        singState.step = 1;
        Serial.println("唱歌：左转一圈（慢速）");
        break;
        
      case 1: // 左转持续1600ms
        if (currentTime - singState.stepTime >= 1600) {
          motorWifi(0); // 立即停止
          singState.step = 2;
          singState.stepTime = currentTime;
        }
        break;
        
      case 2: // 右转一圈（速度减半）
        motorWifi(4); // 右转
        singState.stepTime = currentTime;
        singState.step = 3;
        Serial.println("唱歌：右转一圈（慢速）");
        break;
        
      case 3: // 右转持续1600ms
        if (currentTime - singState.stepTime >= 1600) {
          motorWifi(0); // 立即停止
          singState.step = 4;
          singState.loopCount++;
          Serial.print("唱歌：完成第");
          Serial.print(singState.loopCount);
          Serial.println("次循环");
        }
        break;
        
      case 4: // 检查是否完成循环
        if (singState.loopCount < 1) { // 需要循环1次
          singState.step = 0; // 重新开始左转
        } else {
          roboEyes.setMood(DEFAULT);         // 恢复默认表情
          // 唱歌完成
          delay(100); // 完成延迟
          voiceActionActive = false;
          singState = {0, 0, 0, false, false};
          manualActive = false;
          Serial.println("唱歌完成");
        }
        break;
    }
    
    voiceSequenceLock = false; // 解锁序列动作
  }
}

/* ================= 检查并停止语音动作 ================= */
void checkVoiceAction() {
  if (!voiceActionActive) return;
  
  unsigned long currentTime = millis();
  
  if (danceState.inSequence || singState.inSequence) {
    handleVoiceActionSequence();
  } else {
    // 单次短时动作
    if (currentTime - voiceActionStartTime >= voiceActionDuration) {
      motorWifi(0); // 立即停止电机
      voiceActionActive = false;
      manualActive = false; // 允许恢复随机模式
      fallLock = false;  // 语音动作完成，解除锁定
      Serial.println("语音动作完成");
    }
  }
}

/* ================= 处理来自ASR Pro2的语音指令 ================= */
void handleVoiceCommand() {
  // 从串口读取数据
  while (ASRSerial.available()) {
    char inChar = (char)ASRSerial.read();
    
    if (inChar == '\n') {
      if (voiceCommand.length() > 0) {
        voiceCommandComplete = true;
      }
    } else {
      voiceCommand += inChar;
    }
  }

  // 处理完整指令
  if (voiceCommandComplete) {
    voiceCommand.trim();
    Serial.print("[语音指令] 收到: \"");
    Serial.print(voiceCommand);
    Serial.println("\"");
    
    // 等待序列动作锁释放（最多等待200ms）
    unsigned long waitStart = millis();
    while (voiceSequenceLock && (millis() - waitStart < 200)) {
      delay(10);
      handleVoiceActionSequence(); // 继续处理当前序列
    }
    
    // 如果序列动作仍在进行，忽略新指令
    if (voiceSequenceLock) {
      Serial.println("序列动作进行中，忽略新指令");
      voiceCommand = "";
      voiceCommandComplete = false;
      return;
    }
    
    // 重要：立即停止当前任何动作，为新指令让路
    motorWifi(0);
    
    // 设置重置标志，确保序列动作能正确中断
    if (danceState.inSequence) {
      danceState.resetFlag = true;
      delay(50); // 增加延迟确保稳定
    }
    if (singState.inSequence) {
      singState.resetFlag = true;
      delay(50); // 增加延迟确保稳定
    }
    
    // 激活手动控制标志，暂停随机模式
    manualActive = true;
    lastCommandTime = millis();
    
    // 指令映射
    if (voiceCommand == "STOP") {
      wakeupFromSleep();//脱困后唤醒
      stopAllMotors();                // 停止电机，清除手动标志，解除防跌落锁
      voiceActionActive = false;// 停止语音动作
      danceState = {0, 0, 0, false, false};
      singState = {0, 0, 0, false, false};
      Serial.println("执行：停止");
      
    } else if (voiceCommand == "CMD_FWD") {
      // 中止边缘逃避
      if (seekingSafeDir) {
          motorWifi(0);
          seekingSafeDir = false;
          safeSeekState = 0;
          safeSeekStep = 0;
      }
      // 新增：中止随机动作，恢复默认表情
      randomActive = false;
      roboEyes.setMood(DEFAULT);

      wakeupFromSleep();
      // 前进一小段
      motorWifi(1);
      voiceActionActive = true;
      voiceActionStartTime = millis();
      voiceActionDuration = 800;
      voiceActionType = 1;
      Serial.println("执行：前进一小段");
      
    } else if (voiceCommand == "CMD_BACK") {
      // 中止边缘逃避
      if (seekingSafeDir) {
          motorWifi(0);
          seekingSafeDir = false;
          safeSeekState = 0;
          safeSeekStep = 0;
      }
      // 新增：中止随机动作，恢复默认表情
      randomActive = false;
      roboEyes.setMood(DEFAULT);
      
      wakeupFromSleep();
      // 后退一小段
      motorWifi(2);
      voiceActionActive = true;
      voiceActionStartTime = millis();
      voiceActionDuration = 800;
      voiceActionType = 2;
      Serial.println("执行：后退一小段");
      
    } else if (voiceCommand == "CMD_LEFT") {
      // 中止边缘逃避
      if (seekingSafeDir) {
          motorWifi(0);
          seekingSafeDir = false;
          safeSeekState = 0;
          safeSeekStep = 0;
      }
      // 新增：中止随机动作，恢复默认表情
      randomActive = false;
      roboEyes.setMood(DEFAULT);
      
      wakeupFromSleep();
      // 左转一小段（100ms）
      motorWifi(3);
      voiceActionActive = true;
      voiceActionStartTime = millis();
      voiceActionDuration = 100;
      voiceActionType = 3;
      Serial.println("执行：左转一小段（幅度减半）");
      
    } else if (voiceCommand == "CMD_RIGHT") {
      // 中止边缘逃避
      if (seekingSafeDir) {
          motorWifi(0);
          seekingSafeDir = false;
          safeSeekState = 0;
          safeSeekStep = 0;
      }
      // 新增：中止随机动作，恢复默认表情
      randomActive = false;
      roboEyes.setMood(DEFAULT);
      
      wakeupFromSleep();
      // 右转一小段（幅100ms）
      motorWifi(4);
      voiceActionActive = true;
      voiceActionStartTime = millis();
      voiceActionDuration = 100;
      voiceActionType = 4;
      Serial.println("执行：右转一小段（幅度减半）");
      
    } else if (voiceCommand == "DANCE") {
      // 跳舞：左转小半圈，右转小半圈，循环2次后停止
      roboEyes.setMood(HAPPY);               // <-- 新增：设置开心表情
      voiceActionActive = true;
      danceState = {0, 0, millis(), true, false};
      voiceActionType = 5;
      Serial.println("执行：跳舞（2次循环）");
      
    } else if (voiceCommand == "SING") {
      // 唱歌：左转一圈，右转一圈，循环1次后停止（速度减半）
      roboEyes.setMood(HAPPY);               // <-- 新增：设置开心表情
      voiceActionActive = true;
      singState = {0, 0, millis(), true, false};
      voiceActionType = 6;
      Serial.println("执行：唱歌（1次循环，速度减半）");
      
    } else {
      Serial.println("未知指令");
      manualActive = false; // 未知指令不打断随机模式
    }

    // 清空变量等待下一条指令
    voiceCommand = "";
    voiceCommandComplete = false;
    
    // 重要：立即开始执行新动作
    checkVoiceAction();
  }
}

/* ================= 网页界面 ================= */
void handleRoot() {
  String page = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>桌面机器人控制</title>
<style>
* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}

body {
  margin: 0;
  height: 100vh;
  background: radial-gradient(circle at top, #0f2027, #000);
  color: #00ffe1;
  font-family: "Microsoft YaHei", Arial, sans-serif;
  display: flex;
  align-items: center;
  justify-content: center;
  touch-action: manipulation;
  -webkit-user-select: none;
  user-select: none;
  overflow: hidden;
}

.panel {
  width: 95vw;
  max-width: 320px;
  padding: 15px;
  border-radius: 20px;
  background: rgba(0, 255, 225, 0.05);
  border: 1px solid rgba(0, 255, 225, 0.4);
  box-shadow: 0 0 30px rgba(0, 255, 225, 0.3);
  backdrop-filter: blur(10px);
  box-sizing: border-box;
  overflow: hidden;
}

h2 {
  text-align: center;
  margin: 0 0 20px 0;
  letter-spacing: 2px;
  font-size: 26px;
  color: #00ffe1;
  text-shadow: 0 0 10px rgba(0, 255, 225, 0.5);
}

.control-area {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  grid-template-rows: 90px 90px 90px;
  gap: 15px;
  margin-bottom: 25px;
}

.control-btn {
  border: none;
  border-radius: 20px;
  font-size: 20px;
  font-weight: bold;
  cursor: pointer;
  transition: all 0.15s ease;
  display: flex;
  align-items: center;
  justify-content: center;
  touch-action: manipulation;
  position: relative;
  overflow: hidden;
}

.control-btn::after {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(255, 255, 255, 0.1);
  opacity: 0;
  transition: opacity 0.2s;
}

.control-btn:active::after {
  opacity: 1;
}

.dir-btn {
  background: linear-gradient(145deg, #00e6ff, #0099cc);
  color: white;
  box-shadow: 
    0 6px 0 #0077aa,
    0 12px 20px rgba(0, 230, 255, 0.4);
  border: 2px solid rgba(0, 255, 255, 0.3);
}

.dir-btn:active {
  transform: translateY(6px);
  box-shadow: 
    0 2px 0 #0077aa,
    0 4px 10px rgba(0, 230, 255, 0.3);
}

.center-space {
  background: transparent;
  border: 2px solid transparent;
  border-radius: 16px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 13px;
  color: transparent;
  opacity: 0;
  padding: 10px;
  text-align: center;
}

.mode {
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 8px;
  margin-bottom: 20px;
}

.mode button {
  font-size: 13px;
  padding: 10px 4px;
  border: none;
  border-radius: 12px;
  background: rgba(0, 255, 225, 0.1);
  color: #00ffe1;
  cursor: pointer;
  transition: all 0.3s ease;
  border: 1px solid rgba(0, 255, 225, 0.2);
}

.mode button:hover {
  background: rgba(0, 255, 225, 0.2);
}

.mode button.active {
  background: linear-gradient(145deg, #00ff9c, #00c46a);
  color: #000;
  box-shadow: 0 0 15px rgba(0, 255, 180, 0.8);
  font-weight: bold;
  border: 1px solid rgba(0, 255, 180, 0.5);
}

/* 网络配置区 */
.config-section {
  margin-top: 15px;
  padding: 12px;
  background: rgba(0, 0, 0, 0.25);
  border-radius: 10px;
  border: 1px solid rgba(0, 255, 225, 0.2);
}

.config-section h4 {
  margin: 0 0 8px 0;
  font-size: 12px;
  color: #00ffe1;
  opacity: 0.8;
}

.config-row {
  display: flex;
  gap: 4px;
  margin-bottom: 6px;
  align-items: center;
  width: 100%;
  box-sizing: border-box;
}

.config-row label {
  font-size: 11px;
  width: 36px;
  flex-shrink: 0;
}

.config-row input {
  flex: 1;
  min-width: 0; /* 重要：防止flex子项溢出 */
  padding: 6px 8px;
  border-radius: 6px;
  border: 1px solid rgba(0, 255, 225, 0.3);
  background: rgba(0, 0, 0, 0.3);
  color: #00ffe1;
  font-size: 16px; /* 字体≥16px防止iOS自动缩放 */
  -webkit-text-size-adjust: 100%;
  -moz-text-size-adjust: 100%;
  -ms-text-size-adjust: 100%;
  touch-action: manipulation;
  box-sizing: border-box;
}

.config-row select {
  flex: 1;
  min-width: 0; /* 重要：防止flex子项溢出 */
  padding: 6px 8px;
  border-radius: 6px;
  border: 1px solid rgba(0, 255, 225, 0.3);
  background: rgba(0, 0, 0, 0.3);
  color: #00ffe1;
  font-size: 16px; /* 字体≥16px防止iOS自动缩放 */
  -webkit-text-size-adjust: 100%;
  box-sizing: border-box;
}

.config-row button {
  flex-shrink: 0; /* 防止按钮被压缩 */
  padding: 6px 10px;
  border-radius: 6px;
  border: none;
  background: linear-gradient(145deg, #00e6ff, #0099cc);
  color: white;
  font-size: 11px;
  cursor: pointer;
  white-space: nowrap;
}

.config-row select option {
  background: #111;
  color: #00ffe1;
}

.net-status {
  font-size: 11px;
  color: #00ffe1;
  opacity: 0.6;
  text-align: center;
  margin-top: 6px;
}

.status {
  text-align: center;
  font-size: 13px;
  color: #00ffe1;
  opacity: 0.8;
  margin-top: 20px;
  padding: 12px;
  background: rgba(0, 0, 0, 0.25);
  border-radius: 10px;
  border: 1px solid rgba(0, 255, 225, 0.2);
}

.footer {
  margin-top: 20px;
  text-align: center;
  font-size: 13px;
  opacity: 0.6;
  letter-spacing: 1px;
}

.led {
  display: inline-block;
  width: 10px;
  height: 10px;
  border-radius: 50%;
  margin-right: 8px;
  background: #0f0;
  box-shadow: 0 0 10px #0f0;
  animation: pulse 1.5s infinite;
}

@keyframes pulse {
  0%, 100% { 
    opacity: 1;
    box-shadow: 0 0 10px #0f0, 0 0 20px #0f0;
  }
  50% { 
    opacity: 0.7;
    box-shadow: 0 0 5px #0f0, 0 0 10px #0f0;
  }
}

.connection-info {
  font-size: 11px;
  color: #00ffe1;
  opacity: 0.6;
  text-align: center;
  margin-top: 10px;
  padding: 8px;
  background: rgba(0, 0, 0, 0.2);
  border-radius: 8px;
}

/* 响应式调整 */
@media (max-width: 360px) {
  .panel {
    width: 95vw;
    padding: 15px;
  }
  
  .control-area {
    grid-template-rows: 80px 80px 80px;
    gap: 12px;
  }
  
  h2 {
    font-size: 22px;
  }
}
</style>
</head>
<body>

<div class="panel">
<h2>🤖 桌面机器人</h2>

<div class="control-area">
  <div class="center-space"></div>
  <button class="control-btn dir-btn" 
          id="btnForward"
          ontouchstart="startCommand('/f')" 
          ontouchend="stopCommand()"
          ontouchcancel="stopCommand()"
          onmousedown="startCommand('/f')" 
          onmouseup="stopCommand()"
          onmouseleave="stopCommand()">
    前进 ▲
  </button>
  <div class="center-space"></div>

  <button class="control-btn dir-btn" 
          id="btnLeft"
          ontouchstart="startCommand('/l')" 
          ontouchend="stopCommand()"
          ontouchcancel="stopCommand()"
          onmousedown="startCommand('/l')" 
          onmouseup="stopCommand()"
          onmouseleave="stopCommand()">
    左转 ◀
  </button>
  <div class="center-space"></div>
  <button class="control-btn dir-btn" 
          id="btnRight"
          ontouchstart="startCommand('/r')" 
          ontouchend="stopCommand()"
          ontouchcancel="stopCommand()"
          onmousedown="startCommand('/r')" 
          onmouseup="stopCommand()"
          onmouseleave="stopCommand()">
    右转 ▶
  </button>

  <div class="center-space"></div>
  <button class="control-btn dir-btn" 
          id="btnBackward"
          ontouchstart="startCommand('/b')" 
          ontouchend="stopCommand()"
          ontouchcancel="stopCommand()"
          onmousedown="startCommand('/b')" 
          onmouseup="stopCommand()"
          onmouseleave="stopCommand()">
    后退 ▼
  </button>
  <div class="center-space"></div>
</div>

<div class="mode">
  <button id="btn_sleep" onclick="setMode('off')">睡眠</button>
  <button id="btn_wiggle" onclick="setMode('soft')">轻柔</button>
  <button id="btn_curious" class="active" onclick="setMode('normal')">好奇</button>
  <button id="btn_clock" onclick="setMode('clock')">时钟</button>
</div>

<div class="status">
  <span class="led"></span>
  已连接 | 长按方向键控制移动，松开即停
</div>

<div id="netStatus" class="net-status"></div>

<div class="config-section">
  <h4>WiFi 路由器配置</h4>
  <div class="config-row">
    <label>WiFi</label>
    <select id="cfg_ssid" onclick="scanWifiIfEmpty()">
      <option value="">-- 点击扫描 --</option>
    </select>
    <button onclick="scanWifi()" id="btn_scan">扫描</button>
  </div>
  <div class="config-row">
    <label>密码</label>
    <input type="password" id="cfg_password" placeholder="WiFi密码">
  </div>
  <h4>天气配置</h4>
  <div class="config-row">
    <label>API Key</label>
    <input type="password" id="cfg_apikey" placeholder="OpenWeatherMap Key" title="点击输入框可显示内容">
  </div>
  <div class="config-row">
    <label>城市</label>
    <input type="text" id="cfg_city" placeholder="输入城市名(拼音)" autocomplete="off" autocorrect="off" autocapitalize="off" spellcheck="false">
    <button onclick="refreshWeather()" style="flex:0 0 auto;min-width:80px;padding:8px;margin-left:8px">刷新天气</button>
  </div>
  <div class="config-row">
    <button onclick="saveConfig()" style="flex:1">保存配置</button>
  </div>
  <div class="config-row">
    <button onclick="testConnect()" style="flex:1">测试连接</button>
  </div>
</div>

<div class="connection-info">
  热点: 桌面机器人 | IP: 192.168.4.1<br>
  语音控制已启用
</div>

<div class="footer">Pudite</div>
</div>

<script>
let activeCommand = null;
let commandTimer = null;
let currentMode = 'normal'; // 记录当前模式

// 阻止默认触摸行为
document.addEventListener('touchstart', function(e) {
  if (e.target.classList.contains('dir-btn')) {
    e.preventDefault();
  }
}, { passive: false });

// 开始命令函数
function startCommand(cmd) {
  if (activeCommand !== cmd) {
    stopCommand(); // 先停止当前命令
    
    activeCommand = cmd;
    sendCommand(cmd);
    
    // 设置定时器持续发送命令
    commandTimer = setInterval(() => {
      sendCommand(cmd);
    }, 200);
    
    console.log('开始持续命令:', cmd);
  }
}

// 停止命令函数
function stopCommand() {
  if (activeCommand) {
    clearInterval(commandTimer);
    sendCommand('/s');
    activeCommand = null;
    console.log('发送停止命令');
  }
}

// 命令发送函数（增强版）
function sendCommand(cmd) {
  fetch(cmd)
    .then(response => {
      if (!response.ok) {
        console.error('命令发送失败:', response.status);
      }
      return response.text();
    })
    .then(text => {
      // 检查是否是模式切换响应，如果是则更新按钮状态
      if (text.includes('已切换到睡眠模式')) {
        updateModeButtons('off');
      } else if (text.includes('已切换到轻柔模式')) {
        updateModeButtons('soft');
      } else if (text.includes('已切换到好奇模式')) {
        updateModeButtons('normal');
      }
    })
    .catch(error => {
      console.error('网络错误:', error);
    });
}

// 更新模式按钮状态
function updateModeButtons(mode) {
  clearActive();
  currentMode = mode;

  if(mode === 'off') {
    document.getElementById('btn_sleep').classList.add('active');
  } else if(mode === 'soft') {
    document.getElementById('btn_wiggle').classList.add('active');
  } else if(mode === 'normal') {
    document.getElementById('btn_curious').classList.add('active');
  } else if(mode === 'clock') {
    document.getElementById('btn_clock').classList.add('active');
  }
}

// 模式切换函数
function clearActive(){
  document.getElementById('btn_sleep').classList.remove('active');
  document.getElementById('btn_wiggle').classList.remove('active');
  document.getElementById('btn_curious').classList.remove('active');
  document.getElementById('btn_clock').classList.remove('active');
}

let wifiScanned = false;

// 扫描WiFi网络
function scanWifi() {
  const sel = document.getElementById('cfg_ssid');
  const btn = document.getElementById('btn_scan');
  btn.textContent = '扫描中...';
  btn.disabled = true;
  sel.innerHTML = '<option value="">扫描中...</option>';
  fetch('/scanWifi')
    .then(r => r.json())
    .then(d => {
      sel.innerHTML = '<option value="">选择WiFi</option>';
      for (let i = 0; i < d.length; i++) {
        const opt = document.createElement('option');
        opt.value = d[i].ssid;
        opt.textContent = d[i].ssid;
        sel.appendChild(opt);
      }
      wifiScanned = true;
      btn.textContent = '刷新';
      btn.disabled = false;
      // 恢复之前保存的SSID选中状态
      loadConfig();
    })
    .catch(e => {
      sel.innerHTML = '<option value="">扫描失败</option>';
      btn.textContent = '扫描';
      btn.disabled = false;
    });
}

// 首次点击下拉框时自动扫描
function scanWifiIfEmpty() {
  if (!wifiScanned) {
    scanWifi();
  }
}

// 保存配置
function saveConfig() {
  const ssid = document.getElementById('cfg_ssid').value;
  const password = document.getElementById('cfg_password').value;
  const apiKey = document.getElementById('cfg_apikey').value;
  const city = document.getElementById('cfg_city').value;
  fetch('/saveWifiConfig?ssid=' + encodeURIComponent(ssid) +
    '&password=' + encodeURIComponent(password) +
    '&apiKey=' + encodeURIComponent(apiKey) +
    '&city=' + encodeURIComponent(city))
    .then(r => r.text())
    .then(t => { alert(t); loadConfig(); })
    .catch(e => { alert('保存失败: ' + e); });
}

// 测试路由器连接
function testConnect() {
  const status = document.getElementById('netStatus');
  status.textContent = '连接中...';
  fetch('/testConnect')
    .then(r => r.text())
    .then(t => { status.textContent = t; })
    .catch(e => { status.textContent = '连接失败'; });
}

function refreshWeather() {
  const status = document.getElementById('netStatus');
  status.textContent = '刷新天气中...';
  fetch('/refreshWeather')
    .then(r => r.text())
    .then(t => { status.textContent = t; })
    .catch(e => { status.textContent = '刷新失败'; });
}

// 加载配置到表单
function loadConfig() {
  fetch('/loadConfig')
    .then(r => r.json())
    .then(d => {
      const ssidSel = document.getElementById('cfg_ssid');

      // 如果有保存的SSID，确保它在下拉列表中
      if (d.ssid && d.ssid.length > 0) {
        let exists = false;
        for (let i = 0; i < ssidSel.options.length; i++) {
          if (ssidSel.options[i].value === d.ssid) {
            exists = true;
            break;
          }
        }
        if (!exists) {
          const opt = document.createElement('option');
          opt.value = d.ssid;
          opt.textContent = d.ssid + ' (已保存)';
          ssidSel.insertBefore(opt, ssidSel.firstChild);
        }
      }

      ssidSel.value = d.ssid || '';
      document.getElementById('cfg_password').value = d.password || '';
      document.getElementById('cfg_apikey').value = d.apiKey || '';
      const cityInput = document.getElementById('cfg_city');
      if (d.city) {
        cityInput.blur();
        cityInput.value = d.city;
        cityInput.blur();
      }

      // 提示是否已有配置
      const status = document.getElementById('netStatus');
      if (status) {
        let text = d.connected ? '路由器: 已连接' : '路由器: 未连接';
        if (d.connected) {
          if (d.city) {
            text += ' | 城市: ' + d.city;
          } else {
            text += ' | 天气: 需填城市';
          }
        } else {
          // 未连接时提示用户填写WiFi信息
          if (!d.ssid || d.ssid.length === 0) {
            text += ' | 请填写WiFi信息';
          }
        }
        status.textContent = text;
      }
    })
    .catch(() => {});
}

function setMode(mode){
  sendCommand('/mode_' + mode);
  updateModeButtons(mode);
  
  // 切换模式时自动停止
  stopCommand();
}

// 页面加载完成后的初始化
document.addEventListener('DOMContentLoaded', function() {
  console.log('桌面机器人控制页面已加载');

  // 立即移除所有输入框的焦点
  setTimeout(function() {
    const inputs = document.querySelectorAll('input, select');
    inputs.forEach(function(el) {
      el.blur();
    });
    window.scrollTo(0, 0);
    document.activeElement && document.activeElement.blur();
  }, 10);

  // 防止页面滚动
  document.body.addEventListener('touchmove', function(e) {
    if (e.target.classList.contains('dir-btn')) {
      e.preventDefault();
    }
  }, { passive: false });

  // 鼠标移出窗口时停止
  document.addEventListener('mouseleave', stopCommand);

  // 处理页面可见性变化（切换到其他标签页时停止）
  document.addEventListener('visibilitychange', function() {
    if (document.hidden) {
      stopCommand();
    }
  });

  // 防止右键菜单
  document.addEventListener('contextmenu', function(e) {
    if (e.target.classList.contains('dir-btn')) {
      e.preventDefault();
    }
  });

  // 防止任何输入框自动聚焦
  document.addEventListener('focusin', function(e) {
    // 只有用户真正点击时才允许聚焦
    if (!e.isTrusted) {
      e.preventDefault();
      e.target.blur();
    }
  }, true);

  // 定期检查模式状态（每3秒一次）
  setInterval(checkModeStatus, 3000);

  // 延迟加载配置，避免触发自动聚焦
  setTimeout(function() {
    loadConfig();
    // 再次移除焦点
    const inputs = document.querySelectorAll('input, select');
    inputs.forEach(function(el) {
      el.blur();
    });
    window.scrollTo(0, 0);
  }, 100);

  // 加载配置后自动扫描WiFi（扫描结果会补充到下拉列表，如果扫描到已保存的SSID则更新显示）
  setTimeout(function() {
    scanWifi();
  }, 300);
});

// 检查模式状态
function checkModeStatus() {
  fetch('/checkMode')
    .then(response => response.text())
    .then(mode => {
      if (mode && mode !== currentMode) {
        updateModeButtons(mode);
      }
    })
    .catch(error => {
      console.error('检查模式状态失败:', error);
    });
  // 更新网络状态
  fetch('/loadConfig')
    .then(r => r.json())
    .then(d => {
      const status = document.getElementById('netStatus');
      if (status) {
        let text = d.connected ? '路由器: 已连接' : '路由器: 未连接';
        if (d.connected) {
          if (d.city) {
            text += ' | 城市: ' + d.city;
          } else {
            text += ' | 天气: 需填城市';
          }
        }
        status.textContent = text;
      }
    })
    .catch(() => {});
}

// 键盘快捷键支持（可选）
document.addEventListener('keydown', function(e) {
  switch(e.key) {
    case 'ArrowUp':
    case 'w':
    case 'W':
      startCommand('/f');
      break;
    case 'ArrowDown':
    case 's':
    case 'S':
      startCommand('/b');
      break;
    case 'ArrowLeft':
    case 'a':
    case 'A':
      startCommand('/l');
      break;
    case 'ArrowRight':
    case 'd':
    case 'D':
      startCommand('/r');
      break;
  }
});

document.addEventListener('keyup', function(e) {
  switch(e.key) {
    case 'ArrowUp':
    case 'ArrowDown':
    case 'ArrowLeft':
    case 'ArrowRight':
    case 'w':
    case 'W':
    case 's':
    case 'S':
    case 'a':
    case 'A':
    case 'd':
    case 'D':
      stopCommand();
      break;
  }
});
</script>

</body>
</html>
)rawliteral";

  server.sendHeader("Content-Type", "text/html; charset=utf-8");
  server.send(200, "text/html", page);
}

/* ================= 服务器路由设置 ================= */
void setupServer() {
  server.on("/", handleRoot);

  // 检查模式状态路由
  server.on("/checkMode", []() {
    String modeStr;
    switch (randomMode) {
      case RANDOM_OFF: modeStr = "off"; break;
      case RANDOM_SOFT: modeStr = "soft"; break;
      case RANDOM_NORMAL: modeStr = "normal"; break;
      case RANDOM_CLOCK: modeStr = "clock"; break;
      default: modeStr = "normal";
    }
    server.send(200, "text/plain", modeStr);
  });

  // 时间模式路由
  server.on("/mode_clock", []() {
    // 中止边缘逃避
    if (seekingSafeDir) {
        motorWifi(0);
        seekingSafeDir = false;
        safeSeekState = 0;
        safeSeekStep = 0;
    }
    randomMode = RANDOM_CLOCK;
    // 关闭眨眼和空闲动画
    roboEyes.setAutoblinker(OFF);
    roboEyes.setIdleMode(OFF);
    roboEyes.open();
    roboEyes.setSweat(false);
    roboEyes.setMood(DEFAULT);
    // 停止电机
    stopAllMotors();
    voiceActionActive = false;
    danceState = {0, 0, 0, false, false};
    singState = {0, 0, 0, false, false};
    // 显示时钟
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(1);
    display.setCursor(0, 14);

    // 如果已经在时间模式且WiFi已连接，只刷新天气，不用重连
    if (clockModeInitialized && routerConnected) {
      Serial.println("时间模式：已连接，只刷新天气");
      display.print("Refreshing...");
      display.display();
      fetchWeather();
      lastWeatherFetch = millis();
      Serial.println("切换到时间模式");
      server.sendHeader("Content-Type", "text/plain; charset=utf-8");
      server.send(200, "text/plain", "已刷新天气");
      return;
    }

    display.print("Connecting WiFi...");
    display.display();
    lastClockUpdate = millis();

    // 初次进入时间模式，连接WiFi
    Serial.println("时间模式：开始连接路由器...");

    if (!clockModeInitialized) {
      WiFi.mode(WIFI_AP_STA);
      delay(200);
    }

    connectToRouter();
    if (routerConnected) {
      if (!clockModeInitialized) {
        initSNTP();
      }
      clockModeInitialized = true;

      // 更新显示
      display.clearDisplay();
      display.setCursor(0, 14);
      display.print("Syncing time...");
      display.display();

      // 获取天气
      Serial.println("等待网络栈稳定...");
      delay(1000);
      fetchWeather();
      lastWeatherFetch = millis();
    } else {
      // WiFi连接失败
      display.clearDisplay();
      display.setCursor(0, 14);
      display.print("WiFi not config'd");
      display.display();
    }

    Serial.println("切换到时间模式");
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "已切换到时间模式");
  });

  // 保存配置路由
  server.on("/saveWifiConfig", []() {
    String ssid = server.arg("ssid");
    String password = server.arg("password");
    String apiKey = server.arg("apiKey");
    String city = server.arg("city");

    // 检查WiFi配置是否有变化
    bool wifiConfigChanged = (ssid != String(routerSSID) || password != String(routerPassword));
    bool cityChanged = (city != String(weatherCity));

    ssid.toCharArray(routerSSID, sizeof(routerSSID));
    password.toCharArray(routerPassword, sizeof(routerPassword));
    apiKey.toCharArray(weatherApiKey, sizeof(weatherApiKey));
    city.toCharArray(weatherCity, sizeof(weatherCity));

    saveConfigToEEPROM();

    // 只有WiFi配置变化时才重置WiFi连接
    if (wifiConfigChanged) {
      Serial.println("WiFi配置已更改，重置连接");
      clockModeInitialized = false;
      routerConnected = false;
      WiFi.disconnect(true);
      delay(200);
    }

    // 如果WiFi配置变化了，尝试连接
    if (wifiConfigChanged && ssid.length() > 0) {
      Serial.println("保存配置后测试连接...");
      connectToRouter();
      if (routerConnected) {
        initSNTP();
        // 同时刷新天气
        fetchWeather();
        lastWeatherFetch = millis();
        server.send(200, "text/plain", "配置已保存！路由器已连接成功！");
      } else {
        server.send(200, "text/plain", "配置已保存，但路由器连接失败，请检查密码和信号");
      }
    } else if (cityChanged && routerConnected) {
      // 只是城市变了，只刷新天气
      Serial.println("城市已更改，刷新天气...");
      fetchWeather();
      lastWeatherFetch = millis();
      server.send(200, "text/plain", "配置已保存！天气已刷新！");
    } else {
      server.send(200, "text/plain", "配置已保存");
    }
  });

  // 刷新天气路由
  server.on("/refreshWeather", []() {
    if (!routerConnected) {
      server.send(200, "text/plain", "错误：WiFi未连接");
      return;
    }
    Serial.println("手动刷新天气...");
    fetchWeather();
    lastWeatherFetch = millis();
    server.send(200, "text/plain", "天气已刷新");
  });

  // 扫描WiFi网络
  server.on("/scanWifi", []() {
    int n = WiFi.scanNetworks();
    String json = "[";
    for (int i = 0; i < n && i < 20; i++) {  // 最多返回20个
      if (i > 0) json += ",";
      json += "{\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + WiFi.RSSI(i) + "}";
    }
    json += "]";
    WiFi.scanDelete();
    server.send(200, "application/json", json);
  });

  // 测试连接路由
  server.on("/testConnect", []() {
    Serial.println("测试连接按钮被点击");
    // 先检查是否已连接
    if (routerConnected && WiFi.status() == WL_CONNECTED) {
      server.send(200, "text/plain", "✓ 路由器已连接 | IP: " + WiFi.localIP().toString());
      return;
    }
    // 尝试连接
    if (strlen(routerSSID) > 0) {
      connectToRouter();
      if (routerConnected) {
        initSNTP();
        server.send(200, "text/plain", "✓ 连接成功！IP: " + WiFi.localIP().toString());
      } else {
        server.send(200, "text/plain", "✗ 连接失败，请检查密码和信号");
      }
    } else {
      server.send(200, "text/plain", "请先配置WiFi名称和密码");
    }
  });

  // 加载配置路由
  server.on("/loadConfig", []() {
    String json = "{";
    json += "\"ssid\":\"" + String(routerSSID) + "\",";
    json += "\"password\":\"" + String(routerPassword) + "\",";
    json += "\"apiKey\":\"" + String(weatherApiKey) + "\",";
    json += "\"city\":\"" + String(weatherCity) + "\",";
    json += "\"detectedCity\":\"" + String(detectedCity) + "\",";
    json += "\"connected\":" + String(routerConnected ? "true" : "false") + ",";
    json += "\"located\":" + String(locationDetected ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  });

  // 控制路由 - 改为持续动作直到收到停止命令
  server.on("/f", []() {
    // 中止边缘逃避
    if (seekingSafeDir) {
        motorWifi(0);
        seekingSafeDir = false;
        safeSeekState = 0;
        safeSeekStep = 0;
    }
    // 新增：中止随机动作，恢复默认表情
    randomActive = false;
    roboEyes.setMood(DEFAULT);

    wakeupFromSleep();//脱困后唤醒
    manualActive = true;
    currentCommand = 1;
    motorWifi(currentCommand);
    lastCommandTime = millis();
    Serial.println("前进 - 持续动作");
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "前进");
  });

  server.on("/b", []() {
    // 中止边缘逃避
    if (seekingSafeDir) {
        motorWifi(0);
        seekingSafeDir = false;
        safeSeekState = 0;
        safeSeekStep = 0;
    }
    // 新增：中止随机动作，恢复默认表情
    randomActive = false;
    roboEyes.setMood(DEFAULT);
    
    wakeupFromSleep();//脱困后唤醒
    manualActive = true;
    currentCommand = 2;
    motorWifi(currentCommand);
    lastCommandTime = millis();
    Serial.println("后退 - 持续动作");
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "后退");
  });

  server.on("/l", []() {
    // 中止边缘逃避
    if (seekingSafeDir) {
        motorWifi(0);
        seekingSafeDir = false;
        safeSeekState = 0;
        safeSeekStep = 0;
    }
    // 新增：中止随机动作，恢复默认表情
    randomActive = false;
    roboEyes.setMood(DEFAULT);
    
    wakeupFromSleep();//脱困后唤醒
    manualActive = true;
    currentCommand = 3;
    motorWifi(currentCommand);
    lastCommandTime = millis();
    Serial.println("左转 - 持续动作");
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "左转");
  });

  server.on("/r", []() {
    // 中止边缘逃避
    if (seekingSafeDir) {
        motorWifi(0);
        seekingSafeDir = false;
        safeSeekState = 0;
        safeSeekStep = 0;
    }
    // 新增：中止随机动作，恢复默认表情
    randomActive = false;
    roboEyes.setMood(DEFAULT);
    
    wakeupFromSleep();//脱困后唤醒
    manualActive = true;
    currentCommand = 4;
    motorWifi(currentCommand);
    lastCommandTime = millis();
    Serial.println("右转 - 持续动作");
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "右转");
  });

  server.on("/s", []() {
    stopAllMotors();
    Serial.println("停止所有动作");
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "停止");
  });

  // 模式切换路由
  server.on("/mode_off", []() {
    // 中止边缘逃避
    if (seekingSafeDir) {
        motorWifi(0);
        seekingSafeDir = false;
        safeSeekState = 0;
        safeSeekStep = 0;
    }
    randomMode = RANDOM_OFF;
    sleepByObstacle = true;          // 标记为手动睡眠，避免自动唤醒
    // 关闭自动眨眼和空闲模式
    roboEyes.setAutoblinker(OFF);
    roboEyes.setIdleMode(OFF);
    // 设置睡眠表情
    roboEyes.close();
    roboEyes.setSweat(true);
    stopAllMotors();
    voiceActionActive = false;
    danceState = {0, 0, 0, false, false};
    singState = {0, 0, 0, false, false};
    Serial.println("切换到睡眠模式");
    ASRSerial.print("mode_off");
    Serial.println("mode_off 已发送");                 // 添加日志输出确认发送
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "已切换到睡眠模式");
  });

  server.on("/mode_soft", []() {
    // 中止边缘逃避
    if (seekingSafeDir) {
        motorWifi(0);
        seekingSafeDir = false;
        safeSeekState = 0;
        safeSeekStep = 0;
    }
    randomMode = RANDOM_SOFT;
    // 重置眼睛表情到默认
    roboEyes.setAutoblinker(ON, 3, 2);
    roboEyes.setIdleMode(ON, 2, 2);
    roboEyes.open();
    roboEyes.setSweat(false);
    roboEyes.setMood(DEFAULT);
    stopAllMotors();
    voiceActionActive = false;
    danceState = {0, 0, 0, false, false};
    singState = {0, 0, 0, false, false};
    Serial.println("切换到轻柔模式");
    ASRSerial.print("mode_soft");
    Serial.println("mode_soft 已发送");                 // 添加日志输出确认发送
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "已切换到轻柔模式");
  });

  server.on("/mode_normal", []() {
    // 中止边缘逃避
    if (seekingSafeDir) {
        motorWifi(0);
        seekingSafeDir = false;
        safeSeekState = 0;
        safeSeekStep = 0;
    }
    randomMode = RANDOM_NORMAL;
    // 重置眼睛表情到默认
    roboEyes.setAutoblinker(ON, 3, 2);
    roboEyes.setIdleMode(ON, 2, 2);
    roboEyes.open();
    roboEyes.setSweat(false);
    roboEyes.setMood(DEFAULT);
    stopAllMotors();
    voiceActionActive = false;
    danceState = {0, 0, 0, false, false};
    singState = {0, 0, 0, false, false};
    Serial.println("切换到好奇模式");
    ASRSerial.print("mode_normal");
    Serial.println("mode_normal 已发送");                 // 添加日志输出确认发送
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "已切换到好奇模式");
  });

  server.onNotFound(handleRoot);
  server.begin();
}
// 将 getDistance 函数移到全局作用域
uint16_t getDistance() {
  if (!sensorOK) return 2000; // 传感器不可用返回安全值
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // false 表示不打印调试信息
  if (measure.RangeStatus != 4) {   // 4 表示无效数据
    return measure.RangeMilliMeter;
  } else {
    return 2000; // 无效数据返回远距离
  }
}
/* ================= 初始化设置 ================= */
void setup() {
    // 初始化电机引脚
  pinMode(STBY, OUTPUT); digitalWrite(STBY, LOW);
  pinMode(LF, OUTPUT); pinMode(LB, OUTPUT);
  pinMode(RF, OUTPUT); pinMode(RB, OUTPUT);
  Serial.println("电机引脚初始化完成");

  // 增加启动延迟
  delay(2000);

  // 初始化串口通信
  Serial.begin(115200);
  Serial.println("===== 桌面机器人启动 =====");

  // 初始化OLED显示屏
  Serial.println("初始化OLED...");
  if (!display.begin()) {
    Serial.println("OLED初始化失败！");
  } else {
    Serial.println("OLED初始化完成");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Start");
    display.display();
  }

  // 初始化机器人眼睛动画
  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 100);
  roboEyes.setAutoblinker(ON, 3, 2);
  roboEyes.setIdleMode(ON, 2, 2);
  roboEyes.setMood(DEFAULT);
  Serial.println("机器人眼睛初始化完成");

  // 初始化随机种子
  randomSeed(esp_random());

  // ================= WiFi稳定性优化配置 =================
  // 设置WiFi为APSTA模式（AP热点 + STA连接路由器）
  WiFi.mode(WIFI_MODE_APSTA);

  // 加载EEPROM配置
  loadConfigFromEEPROM();

  // 注意：不再开机自动连接路由器，仅在进入时间模式时才连接
  Serial.println("路由器连接已延迟到时间模式启动时");

  // 降低发射功率以提高稳定性
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  
  // 给射频模块一点准备时间
  delay(100);
  
  // 启动WiFi热点
  Serial.println("正在启动WiFi热点...");
  WiFi.softAP("桌面机器人");
  
  Serial.print("热点名称: 桌面机器人");
  Serial.print("IP地址: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("MAC地址: ");
  Serial.println(WiFi.softAPmacAddress());

  // 启动DNS和Web服务器
  dnsServer.start(53, "*", WiFi.softAPIP());
  setupServer();
  Serial.println("DNS和Web服务器启动完成");
  
  // ================= 在WiFi启动后初始化ASR Pro2 =================
  // 增加延迟确保WiFi稳定
  delay(500);
  
  // 初始化ASR Pro2串口
  ASRSerial.begin(ASR_BAUDRATE, SERIAL_8N1, ASR_RX_PIN, ASR_TX_PIN);
  Wire.setClock(100000);
  // ================= 初始化VL53L0X传感器 =================
  if (!lox.begin()) {
    Serial.println("VL53L0X 初始化失败！");
    sensorOK = false;
  } else {
    Serial.println("VL53L0X 初始化成功");
    sensorOK = true;
    Wire.setClock(400000); // 传感器初始化后，立即恢复高速 I2C 速率以保证 OLED 刷新率
    // 动态校准：连续读取10次距离取平均
    uint32_t sumDist = 0;
    for (int i = 0; i < 10; i++) {
      sumDist += getDistance();  // 原来是 getRawDistance()
      delay(50);
    }
    baseDistance = sumDist / 10;
    edgeThreshold = baseDistance + 20; // 防跌落阈值 = 基准 + 20mm
    obstacleThreshold = baseDistance - 20; // 障碍阈值 = 基准 - 20mm
    if (obstacleThreshold < 30) obstacleThreshold = 30; // 避免低于盲区
    Serial.print("障碍阈值: "); Serial.println(obstacleThreshold);
    Serial.print("基准距离: "); Serial.print(baseDistance); Serial.println(" mm");
    Serial.print("防跌落阈值: "); Serial.print(edgeThreshold); Serial.println(" mm");
    latestDistance = baseDistance; // 防止启动时误触发避障
    
    // 在OLED上显示校准结果（2秒）
    display.clearDisplay();
    display.setTextColor(1);
    display.setCursor(0, 0);
    display.print("Calibration OK");
    display.setCursor(0, 12);
    display.print("Base: "); display.print(baseDistance); display.print("mm");
    display.display();
    delay(2000);
    // 读取一次距离测试
    uint16_t dist = getDistance();
    Serial.print("首次距离读取: ");
    Serial.print(dist);
    Serial.println(" mm");
  }
  Serial.println("ASR Pro2 串口初始化完成，等待语音指令...");
  // 使能电机驱动
  digitalWrite(STBY, HIGH);

  Serial.println("===== 系统启动完成 =====");
  Serial.println("请用手机搜索WiFi：桌面机器人");
  Serial.println("然后访问：192.168.4.1");
  Serial.println("语音控制已启用");

  // 在OLED上显示WiFi信息
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 14);
  display.print("WiFi AP Started");
  display.setCursor(0, 28);
  display.print("Name: Desk Robot");
  snprintf(ipBuf, sizeof(ipBuf), "IP: %s", WiFi.softAPIP().toString().c_str());
  display.setCursor(0, 42);
  display.print(ipBuf);
  display.display();
}

/* ================= 主循环 ================= */
void loop() {
  if (randomMode != RANDOM_CLOCK) {
    // 眼睛动画更新，限制帧率防止闪烁
    static unsigned long lastEyeUpdate = 0;
    if (millis() - lastEyeUpdate > 50) {
      lastEyeUpdate = millis();
      roboEyes.update();
    }
  }

  // 处理客户端请求
  server.handleClient();
  dnsServer.processNextRequest();

  // ========== 时间模式逻辑 ==========
  if (randomMode == RANDOM_CLOCK) {
    // 每秒更新时间显示
    if (millis() - lastClockUpdate >= 1000) {
      lastClockUpdate = millis();
      updateClockDisplay();
    }
    // 每15分钟刷新天气
    if (routerConnected && millis() - lastWeatherFetch >= 900000) {
      lastWeatherFetch = millis();
      fetchWeather();
    }
    return; // 时钟模式下不执行其他任何逻辑
  }

  // 检查并停止超时命令（安全保护）
  checkCommandTimeout();
  
  // 处理语音指令 - 优先级最高
  handleVoiceCommand();
  
  // 检查并执行语音动作
  checkVoiceAction();
  
  // ========== 新增：防跌落逻辑 ==========
  // 1. 非阻塞更新滤波距离
  updateFilter();

  // 2. 防跌落触发判断（使用最新距离）
uint16_t rawDistance = getDistance(); // 实时读取一次

  // 异常环境检测（玻璃桌面、悬空等）
static int abnormalCount = 0;
static int normalCount = 0;      // 用于连续正常计数，实现自动唤醒
const uint16_t ABNORMAL_THRESHOLD = 200; // 可根据实际调整
const int ABNORMAL_MAX = 5;        // 连续5次异常触发睡眠
const int NORMAL_MAX = 5;          // 连续5次正常唤醒

static unsigned long lastAbnormalCheck = 0;
if (millis() - lastAbnormalCheck >= 100) {  // 每100ms检查一次
    lastAbnormalCheck = millis();
    // 异常检测逻辑
if (randomMode != RANDOM_OFF && rawDistance > ABNORMAL_THRESHOLD) {
    // 如果正在寻找安全方向，继续旋转
    if (seekingSafeDir) {
        normalCount = 0;
    } else {
        // 不在寻找中，且不在随机动作中，且不在手动控制，则累计异常计数
        if (!randomActive && !manualActive && !voiceActionActive) {
            abnormalCount++;
            if (abnormalCount >= ABNORMAL_MAX) {
                Serial.println("检测到异常环境，进入睡眠模式");
                roboEyes.anim_confused();
                roboEyes.setMood(TIRED);
                randomMode = RANDOM_OFF;
                sleepByObstacle = false;
                motorWifi(0);
                sendSleepMultiple(3);  // 播放3次help
                Serial.println("HELP 已发送");                 // 添加日志输出确认发送
                abnormalCount = 0;
            }
        } else {
            // 如果在随机动作或手动控制中，不计数，避免误判
            abnormalCount = 0;
        }
    }
    normalCount = 0; // 只要距离异常，就清零正常计数
} else {
    // 距离正常
    if (seekingSafeDir) {
        // 如果正在寻找方向，则立即停止，恢复正常
        motorWifi(0);
        seekingSafeDir = false;
        Serial.println("找到安全方向，停止旋转");
    }
    // 自动唤醒逻辑
    abnormalCount = 0;
    // 仅当非避障睡眠时才尝试自动唤醒
    if (randomMode == RANDOM_OFF && !sleepByObstacle) {
        // 距离在合理的正常桌面范围内（如50mm~200mm）才累加唤醒计数
        if (rawDistance > 50 && rawDistance < ABNORMAL_THRESHOLD) {
            normalCount++;
            if (normalCount >= NORMAL_MAX) {
                Serial.println("环境恢复正常，退出睡眠模式");
                if (quickCalibrateBase()) {
                    randomMode = RANDOM_NORMAL;
                    roboEyes.setMood(DEFAULT);
                }
                normalCount = 0;
            }
        } else {
            normalCount = 0; // 距离不在正常范围，清零计数，避免误唤醒
        }
    } else {
        normalCount = 0;
    }
}
}

if (fallState == FALL_IDLE && movingForward && rawDistance > edgeThreshold) {
  roboEyes.setMood(DEFAULT);
  randomActive = false;  // 中止随机动作
  avoidingObstacle = false;  // 中止避障
  seekingSafeDir = false;  // 中止边缘逃避
  motorWifi(0); // 确保停止
  fallLock = true;  // 触发防跌落时锁定前进
  fallState = FALL_STOP;
  fallActionTime = millis();
  
  // 根据当前模式设置 fallMode
  if (manualActive) {
    fallMode = 0; // 手动模式
  } else if (randomMode == RANDOM_NORMAL) {
    fallMode = 1; // 好奇模式
  } else {
    fallMode = 0; // 其他模式默认手动
  }
}

  // 3. 防跌落动作状态机
  switch (fallState) {
  case FALL_STOP:
    if (millis() - fallActionTime >= 100) {
      motorWifi(2); // 后退
      fallActionTime = millis();
      fallState = FALL_BACKWARD;
    }
    break;
    
  case FALL_BACKWARD:
    if (millis() - fallActionTime >= 100) { // 后退100ms
      motorWifi(0);
      if (fallMode == 1) {
        // 好奇模式：准备转向
        fallState = FALL_TURN;
        fallActionTime = millis();
      } else {
        // 手动模式：结束
        fallState = FALL_DONE;
      }
    }
    break;
    
  case FALL_TURN:
    if (millis() - fallActionTime >= 100) { // 短暂停顿
      // 随机选择左转或右转
      if (random(2) == 0) motorWifi(3); else motorWifi(4);
      fallActionTime = millis();
      fallState = FALL_TURNING;
    }
    break;
    
  case FALL_TURNING:
    if (millis() - fallActionTime >= 150) { // 转向150ms
      motorWifi(0);
      fallState = FALL_DONE;
    }
    break;
    
  case FALL_DONE:
    // 防跌落动作完成
    fallState = FALL_IDLE;
    
    // 如果机器人不在边缘逃避中，且当前距离仍然过大，则启动边缘逃避
    if (!seekingSafeDir) {
        uint16_t currentDist = getDistance();
        if (currentDist > ABNORMAL_THRESHOLD+ 10) {// 加10mm余量
            abnormalCount = 0;               // 清零异常计数
            Serial.println("防跌落后仍处于危险区域，启动边缘逃避");
            seekingSafeDir = true;
            safeSeekStep = 0;
            safeSeekState = 0;
            motorWifi(4); // 开始右转
            safeSeekStartTime = millis();
        }
    }
    break;
}

// ========== 3. 避障逻辑（仅在好奇模式、非手动、非语音、未防跌落时） ==========
if (!manualActive && !voiceActionActive && randomMode == RANDOM_NORMAL && fallState == FALL_IDLE) {
  // 触发条件：最新距离小于障碍阈值且未在避障中
  if (!avoidingObstacle && rawDistance < obstacleThreshold) {
    roboEyes.setMood(DEFAULT);
    randomActive = false;  // 中止随机动作
    seekingSafeDir = false;  // 中止边缘逃避
    motorWifi(0); // 确保停止
    Serial.println("避障触发，开始步进旋转扫描");
    avoidingObstacle = true;
    rotateStep = 0;
    avoidStep = 1;                // 进入旋转状态
    motorWifi(4);                  // 开始右转
    avoidStartTime = millis();     // 记录旋转开始时间
  }
  
  // 避障状态机
 if (avoidingObstacle) {
    switch (avoidStep) {
        case 1: // 旋转中
            if (millis() - avoidStartTime >= ROTATE_TIME_PER_STEP) {
                motorWifi(0);           // 停止旋转
                avoidStartTime = millis();
                avoidStep = 2;           // 进入停顿检测状态
            }
            break;

        case 2: // 停顿检测
            // 停顿足够时间让电机停稳（例如100ms）
            if (millis() - avoidStartTime >= 100) {
                uint16_t currentDist = getDistance(); // 读取距离
                if (currentDist > obstacleThreshold + 5 && currentDist < ABNORMAL_THRESHOLD) {
                    // 找到出路，进入前进状态
                    Serial.println("扫描到出路，准备前进");
                    avoidStep = 3;
                    motorWifi(1);        // 开始前进
                    avoidStartTime = millis();
                } else {
                    rotateStep++;
                    if (rotateStep >= MAX_ROTATE_STEPS) {
                        // 旋转超时，进入睡眠
                        Serial.println("未找到出路，进入睡眠模式");
                        roboEyes.anim_confused();
                        roboEyes.setMood(TIRED);
                        motorWifi(0);
                        randomMode = RANDOM_OFF;
                        sleepByObstacle = true;
                        sendAvoidMultiple(3);  // 播放3次Avoid
                        Serial.println("Avoid 已发送");                 // 添加日志输出确认发送
                        avoidingObstacle = false;
                    } else {
                        // 继续下一次旋转
                        motorWifi(4);     // 继续右转
                        avoidStartTime = millis();
                        avoidStep = 1;     // 回到旋转状态
                    }
                }
            }
            break;

        case 3: // 前进一小段
            if (millis() - avoidStartTime >= 100) {
                motorWifi(0);
                avoidingObstacle = false; // 避障完成
            }
            break;

        default:
            avoidingObstacle = false;
            break;
    }
}
}

// 随机动作状态机
if (randomActive) {
    switch (randomStep) {
        case 1: // 动作进行中
            if (millis() - randomActionTime >= randomT1) {
                motorWifi(0); // 停止电机
                randomActionTime = millis();
                randomStep = 2;
            }
            break;
        case 2: // 停顿中
            if (millis() - randomActionTime >= randomT2) {
                randomRepeat--;
                if (randomRepeat > 0) {
                    // 继续下一个循环
                    motorWifi(randomCmd);
                    randomActionTime = millis();
                    randomStep = 1;
                } else {
                    // 所有循环结束，进入稳定等待状态
                    randomStep = 3;
                    randomActionTime = millis(); // 记录等待开始时间
                }
            }
            break;
          case 3: // 稳定等待（消除惯性）
            if (millis() - randomActionTime >= RANDOM_STABLE_DELAY) { // 等待时间ms，可根据需要调整
                // 等待结束，正式完成随机动作
                randomActive = false;
                roboEyes.setMood(DEFAULT);
                if (randomCmd == 3 || randomCmd == 4) {
                    uint16_t currentDist = getDistance();
                    if (currentDist > ABNORMAL_THRESHOLD) {
                        abnormalCount = 0;               // 清零异常计数
                        seekingSafeDir = true;
                        safeSeekStep = 0;
                        safeSeekState = 0;
                        motorWifi(4);
                        safeSeekStartTime = millis();
                    }
                }
            }
            break;
        default:
            randomActive = false;
            break;
    }
}

// 边缘逃避状态机
if (seekingSafeDir) {
    switch (safeSeekState) {
        case 0: // 旋转中
            if (millis() - safeSeekStartTime >= SAFE_ROTATE_TIME) {
                motorWifi(0);
                safeSeekStartTime = millis();
                safeSeekState = 1; // 进入检测
            }
            break;
        case 1: // 检测中（停顿）
            if (millis() - safeSeekStartTime >= 100) { // 停顿100ms
                uint16_t currentDist = getDistance();
                if (currentDist <= ABNORMAL_THRESHOLD) {
                    // 找到安全方向
                    Serial.println("边缘逃避成功");
                    if (!quickCalibrateBase()) {
                        Serial.println("边缘逃避后校准失败，保持旧基准");
                    }
                    seekingSafeDir = false;
                    safeSeekState = 0;
                    safeSeekStep = 0;   // 重置步数
                } else {
                    safeSeekStep++;
                    if (safeSeekStep >= SAFE_SEEK_STEPS) {
                        // 旋转超时，进入睡眠
                        Serial.println("边缘逃避失败，进入睡眠");
                        roboEyes.anim_confused();
                        roboEyes.setMood(TIRED);
                        randomMode = RANDOM_OFF;
                        sleepByObstacle = false;
                        motorWifi(0);
                        sendSafeSeekMultiple(3);  // 播放3次SAFE_SEEK
                        Serial.println("SAFE_SEEK 已发送");                 // 添加日志输出确认发送
                        seekingSafeDir = false;
                        safeSeekState = 0;
                    } else {
                        // 继续旋转
                        motorWifi(4); // 继续右转
                        safeSeekStartTime = millis();
                        safeSeekState = 0; // 回到旋转
                    }
                }
            }
            break;
    }
}

// ========== 4. 随机模式控制（非阻塞） ==========
static unsigned long lastTick = 0;
if (!manualActive && !voiceActionActive && !randomActive && millis() - lastTick > 40) {
    lastTick = millis();

    // 定义允许的动作列表（前进、左转、右转）
    const byte allowedMoves[] = {1, 1, 1, 3, 4};
    const int moveCount = 5;

if (randomMode == RANDOM_SOFT) {
    if (random(120) == 1) {
        randomCmd = allowedMoves[random(moveCount)];
        if (randomCmd == 1) { // 前进
            randomT1 = random(8, 15);   // 增加前进时间
        } else { // 左转或右转
            randomT1 = random(6, 18);     // 保持原转向时间
        }
        randomT2 = random(40, 90);
        randomRepeat = 1;
        Serial.print("随机动作启动: cmd="); Serial.print(randomCmd);
        Serial.print(" t1="); Serial.print(randomT1);
        Serial.print(" t2="); Serial.print(randomT2);
        Serial.print(" repeat="); Serial.println(randomRepeat);
        randomActive = true;
        randomStep = 1;
        roboEyes.setMood(randomMoods[random(moodCount)]);
        motorWifi(randomCmd);
        randomActionTime = millis();
    }
}
else if (randomMode == RANDOM_NORMAL) {
    if (random(100) == 1) {
        randomCmd = allowedMoves[random(moveCount)];
        if (randomCmd == 1) { // 前进
            randomT1 = random(10, 25);  // 前进时间更长
        } else { // 左转或右转
            randomT1 = random(5, 30);     // 转向时间短
        }
        randomT2 = random(10, 100);
        randomRepeat = random(20);
        Serial.print("随机动作启动: cmd="); Serial.print(randomCmd);
        Serial.print(" t1="); Serial.print(randomT1);
        Serial.print(" t2="); Serial.print(randomT2);
        Serial.print(" repeat="); Serial.println(randomRepeat);
        randomActive = true;
        randomStep = 1;
        roboEyes.setMood(randomMoods[random(moodCount)]);
        motorWifi(randomCmd);
        randomActionTime = millis();
    }
}
}

// 每5秒打印一次连接状态
static unsigned long lastStatus = 0;
if (millis() - lastStatus > 5000) {
  lastStatus = millis();
  int stations = WiFi.softAPgetStationNum();
  Serial.print("已连接设备数: ");
  Serial.println(stations);

  // 在OLED上更新连接状态（已禁用，避免干扰眼睛动画）
  /*
  if (stations > 0) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(1);
    display.setCursor(0, 14);
    display.print("Connected:");
    char staBuf[16];
    snprintf(staBuf, sizeof(staBuf), "%d devices", stations);
    display.setCursor(0, 30);
    display.print(staBuf);
    display.display();
  }
  */
}
  // 每2秒读取一次距离用于测试
  static unsigned long lastTestRead = 0;
  if (sensorOK && millis() - lastTestRead > 500) { // 加速到0.5s，确保日志可见
    lastTestRead = millis();
    uint16_t d = getDistance();
    Serial.print("[LOOP] 模式: ");
    switch(randomMode) {
      case RANDOM_OFF: Serial.print("睡眠"); break;
      case RANDOM_SOFT: Serial.print("轻柔"); break;
      case RANDOM_NORMAL: Serial.print("好奇"); break;
      case RANDOM_CLOCK: Serial.print("时钟"); break;
    }
    Serial.print(" | 距离: "); Serial.print(d); Serial.print(" | 前进:"); Serial.print(movingForward ? "是" : "否");
    Serial.print(" | 手动:"); Serial.print(manualActive ? "是" : "否");
    Serial.println(" | 滤波: " + String(latestDistance));
  }
}
void wakeupFromSleep() {
    if (randomMode == RANDOM_OFF) {
        randomMode = RANDOM_NORMAL;   // 恢复到好奇模式（可根据需要改为柔和模式）
        sleepByObstacle = false;       // 清除避障睡眠标志
        // 恢复自动眨眼和空闲模式（参数与初始化一致）
        roboEyes.setAutoblinker(ON, 3, 2);
        roboEyes.setIdleMode(ON, 2, 2);
        // 睁眼，关闭汗水，恢复默认表情
        roboEyes.open();
        roboEyes.setSweat(false);
        roboEyes.setMood(DEFAULT);     // 恢复默认表情
        Serial.println("手动唤醒");
    }
}
void sendSafeSeekMultiple(int times) {
    for (int i = 0; i < times; i++) {
        ASRSerial.print("SAFE_SEEK");
        delay(300);  // 每次间隔250ms，可根据需要调整
    }
}
void sendSleepMultiple(int times) {
    for (int i = 0; i < times; i++) {
        ASRSerial.print("HELP");
        delay(250);  // 每次间隔250ms，可根据需要调整
    }
}
void sendAvoidMultiple(int times) {
    for (int i = 0; i < times; i++) {
        ASRSerial.print("Avoid");
        delay(250);  // 每次间隔250ms，可根据需要调整
    }
}
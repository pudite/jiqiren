#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>
#include <Adafruit_VL53L0X.h>
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool sensorOK = false;            // 传感器状态
uint16_t baseDistance = 0;   // 基准距离
uint16_t edgeThreshold = 0;   // 防跌落阈值
bool movingForward = false;               // 是否正在前进

// 防跌落动作状态机
enum FallState { FALL_IDLE, FALL_STOP, FALL_BACKWARD, FALL_DONE };
FallState fallState = FALL_IDLE;
unsigned long fallActionTime = 0;

// 滤波状态机变量（替代原来的 getFilteredDistance）
enum FilterState { FILTER_IDLE, FILTER_SAMPLING };
FilterState filterState = FILTER_IDLE;
int filterIndex = 0;
uint16_t filterValues[5];
unsigned long lastFilterTime = 0;
uint16_t latestDistance = 0;  // 最新滤波后的距离
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

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RoboEyes<Adafruit_SSD1306> roboEyes(display);

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
  RANDOM_NORMAL   // 正常随机模式
};

volatile RandomMode randomMode = RANDOM_NORMAL;  // 当前随机模式

/* ================= WiFi控制电机函数 ================= */
void motorWifi(byte c) {
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
        
      case 1: // 左转持续400ms
        if (currentTime - danceState.stepTime >= 400) {
          motorWifi(0); // 立即停止
          danceState.step = 2;
          danceState.stepTime = currentTime;
        }
        break;
        
      case 2: // 右转小半圈
        motorWifi(4); // 右转
        danceState.stepTime = currentTime;
        danceState.step = 3;
        Serial.println("跳舞：右转小半圈");
        break;
        
      case 3: // 右转持续400ms
        if (currentTime - danceState.stepTime >= 400) {
          motorWifi(0); // 立即停止
          danceState.step = 4;
          danceState.loopCount++;
          Serial.print("跳舞：完成第");
          Serial.print(danceState.loopCount);
          Serial.println("次循环");
        }
        break;
        
      case 4: // 检查是否完成循环
        if (danceState.loopCount < 2) { // 需要循环2次
          danceState.step = 0; // 重新开始左转
        } else {
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
      // 进入睡眠模式
      randomMode = RANDOM_OFF;
      motorWifi(0);
      manualActive = false;
      voiceActionActive = false;
      danceState = {0, 0, 0, false, false};
      singState = {0, 0, 0, false, false};
      Serial.println("执行：睡眠模式");
      
    } else if (voiceCommand == "CMD_FWD") {
      // 前进一小段
      motorWifi(1);
      voiceActionActive = true;
      voiceActionStartTime = millis();
      voiceActionDuration = 800;
      voiceActionType = 1;
      Serial.println("执行：前进一小段");
      
    } else if (voiceCommand == "CMD_BACK") {
      // 后退一小段
      motorWifi(2);
      voiceActionActive = true;
      voiceActionStartTime = millis();
      voiceActionDuration = 800;
      voiceActionType = 2;
      Serial.println("执行：后退一小段");
      
    } else if (voiceCommand == "CMD_LEFT") {
      // 左转一小段（幅度减半：200ms）
      motorWifi(3);
      voiceActionActive = true;
      voiceActionStartTime = millis();
      voiceActionDuration = 200;
      voiceActionType = 3;
      Serial.println("执行：左转一小段（幅度减半）");
      
    } else if (voiceCommand == "CMD_RIGHT") {
      // 右转一小段（幅度减半：200ms）
      motorWifi(4);
      voiceActionActive = true;
      voiceActionStartTime = millis();
      voiceActionDuration = 200;
      voiceActionType = 4;
      Serial.println("执行：右转一小段（幅度减半）");
      
    } else if (voiceCommand == "DANCE") {
      // 跳舞：左转小半圈，右转小半圈，循环2次后停止
      voiceActionActive = true;
      danceState = {0, 0, millis(), true, false};
      voiceActionType = 5;
      Serial.println("执行：跳舞（2次循环）");
      
    } else if (voiceCommand == "SING") {
      // 唱歌：左转一圈，右转一圈，循环1次后停止（速度减半）
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
<meta name="viewport" content="width=device-width, initial-scale=1">
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
  width: 320px;
  padding: 20px;
  border-radius: 20px;
  background: rgba(0, 255, 225, 0.05);
  border: 1px solid rgba(0, 255, 225, 0.4);
  box-shadow: 0 0 30px rgba(0, 255, 225, 0.3);
  backdrop-filter: blur(10px);
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
  display: flex;
  gap: 10px;
  margin-bottom: 20px;
}

.mode button {
  flex: 1;
  font-size: 15px;
  padding: 12px 6px;
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
  <button id="btn_sleep" onclick="setMode('off')">睡眠模式</button>
  <button id="btn_wiggle" onclick="setMode('soft')">摆动模式</button>
  <button id="btn_curious" class="active" onclick="setMode('normal')">好奇模式</button>
</div>

<div class="status">
  <span class="led"></span> 
  已连接 | 长按方向键控制移动，松开即停
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
      } else if (text.includes('已切换到摆动模式')) {
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
  }
}

// 模式切换函数
function clearActive(){
  document.getElementById('btn_sleep').classList.remove('active');
  document.getElementById('btn_wiggle').classList.remove('active');
  document.getElementById('btn_curious').classList.remove('active');
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
  
  // 定期检查模式状态（每3秒一次）
  setInterval(checkModeStatus, 3000);
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
      default: modeStr = "normal";
    }
    server.send(200, "text/plain", modeStr);
  });

  // 控制路由 - 改为持续动作直到收到停止命令
  server.on("/f", []() {
    manualActive = true;
    currentCommand = 1;
    motorWifi(currentCommand);
    lastCommandTime = millis();
    Serial.println("前进 - 持续动作");
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "前进");
  });

  server.on("/b", []() {
    manualActive = true;
    currentCommand = 2;
    motorWifi(currentCommand);
    lastCommandTime = millis();
    Serial.println("后退 - 持续动作");
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "后退");
  });

  server.on("/l", []() {
    manualActive = true;
    currentCommand = 3;
    motorWifi(currentCommand);
    lastCommandTime = millis();
    Serial.println("左转 - 持续动作");
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "左转");
  });

  server.on("/r", []() {
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
    randomMode = RANDOM_OFF;
    stopAllMotors();
    voiceActionActive = false;
    danceState = {0, 0, 0, false, false};
    singState = {0, 0, 0, false, false};
    Serial.println("切换到睡眠模式");
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "已切换到睡眠模式");
  });

  server.on("/mode_soft", []() {
    randomMode = RANDOM_SOFT;
    stopAllMotors();
    voiceActionActive = false;
    danceState = {0, 0, 0, false, false};
    singState = {0, 0, 0, false, false};
    Serial.println("切换到摆动模式");
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(200, "text/plain", "已切换到摆动模式");
  });

  server.on("/mode_normal", []() {
    randomMode = RANDOM_NORMAL;
    stopAllMotors();
    voiceActionActive = false;
    danceState = {0, 0, 0, false, false};
    singState = {0, 0, 0, false, false};
    Serial.println("切换到好奇模式");
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
  // 增加启动延迟
  delay(2000);

  // 初始化串口通信
  Serial.begin(115200);
  Serial.println("===== 桌面机器人启动 =====");

  // 初始化电机引脚
  pinMode(STBY, OUTPUT); digitalWrite(STBY, LOW);
  pinMode(LF, OUTPUT); pinMode(LB, OUTPUT);
  pinMode(RF, OUTPUT); pinMode(RB, OUTPUT);
  Serial.println("电机引脚初始化完成");

  // 初始化OLED显示屏
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED初始化失败！");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("正在启动...");
    display.display();
    Serial.println("OLED初始化完成");
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
  // 设置WiFi为AP模式
  WiFi.mode(WIFI_MODE_AP);
  
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
  // ================= 初始化VL53L0X传感器 =================
  if (!lox.begin()) {
    Serial.println("VL53L0X 初始化失败！");
    sensorOK = false;
  } else {
    Serial.println("VL53L0X 初始化成功");
    sensorOK = true;
        // 动态校准：连续读取10次距离取平均
    uint32_t sumDist = 0;
    for (int i = 0; i < 10; i++) {
      sumDist += getDistance();  // 原来是 getRawDistance()
      delay(50);
    }
    baseDistance = sumDist / 10;
    edgeThreshold = baseDistance + 30; // 防跌落阈值 = 基准 + 30mm

    Serial.print("基准距离: "); Serial.print(baseDistance); Serial.println(" mm");
    Serial.print("防跌落阈值: "); Serial.print(edgeThreshold); Serial.println(" mm");

    // 在OLED上显示校准结果（2秒）
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Calibration OK");
    display.print("Base: "); display.print(baseDistance); display.println("mm");
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
  display.setCursor(0, 0);
  display.println("WiFi热点已启动");
  display.println("名称:桌面机器人");
  display.print("IP:");
  display.println(WiFi.softAPIP());
  display.println("手机连接后访问");
  display.println("192.168.4.1");
  display.display();
}

/* ================= 主循环 ================= */
void loop() {
  // 更新机器人眼睛动画
  roboEyes.update();

  // 处理客户端请求
  server.handleClient();
  dnsServer.processNextRequest();

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
  if (fallState == FALL_IDLE && movingForward && latestDistance > edgeThreshold) {
    Serial.println("防跌落触发！");
    fallState = FALL_STOP;
    motorWifi(0);
    fallActionTime = millis();
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
      if (millis() - fallActionTime >= 400) {
        motorWifi(0); // 停止
        fallState = FALL_DONE;
      }
      break;
    case FALL_DONE:
      fallState = FALL_IDLE;
      break;
    default:
      break;
  }

  // 随机模式控制 - 仅在非手动控制且非语音动作时执行
  static unsigned long lastTick = 0;
  if (!manualActive && !voiceActionActive && millis() - lastTick > 40) {
    lastTick = millis();

    if (randomMode == RANDOM_SOFT) {
      // 柔和随机模式：低频率小幅度动作
      if (random(120) == 1) {
        MOTOR(random(9), random(6, 18), random(40, 90), 1);
      }
    }
    else if (randomMode == RANDOM_NORMAL) {
      // 正常随机模式：较高频率和幅度动作
      if (random(100) == 1) {
        MOTOR(random(9), random(5, 50), random(10, 100), random(20));
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
    
    // 在OLED上更新连接状态
    if (stations > 0) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("已连接设备:");
      display.print(stations);
      display.println("台");
      display.println("控制中...");
      display.display();
    }
  }
  // 每2秒读取一次距离用于测试
  static unsigned long lastTestRead = 0;
  if (sensorOK && millis() - lastTestRead > 2000) {
    lastTestRead = millis();
    uint16_t d = getDistance();
    Serial.print("测试距离: ");
    Serial.println(d);
  }
}
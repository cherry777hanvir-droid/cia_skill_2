/*
 * 基础技能 2：CIA 物联网平台远程控制设备启停
 * 板子：ESP32-WROOM-32 DevKit（Type-C，CP2102，COM7）
 *
 * 三条通道：
 *   1) 串口    XCOM，9600 8N1
 *   2) 局域网  TCP Client → <板子IP>:8266
 *   3) CIA     TCP → 47.113.229.47:3007
 *
 * 本地命令：LED:1/0  RELAY:1/0  MOTOR:1/0  STATUS?  ALL:0
 *
 * CIA：连上后发设备密码完成认证；下行是 JSON directive；
 *      执行后必须回执，id 原样带回。
 *
 * 接线：LED R→GPIO13；继电器 IN→GPIO4、DC+→5V；
 *      12V→继电器COM、NO→电机+、电机−→GND。
 */

#include <WiFi.h>
#include <WiFiClient.h>

// ================================================================
//  你不用改这个 .ino！配置全部在同目录的 secrets.h 里：
//    - WiFi 名字和密码
//    - CIA_UUID（这台板子在平台的设备密码）
//  改完 secrets.h 直接上传即可。
// ================================================================

// ===== 1. 配置 =====

// 真密码放 secrets.h，不入库；没有也能编译，只是连不上。
#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID   "YOUR_WIFI_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS   "YOUR_WIFI_PASSWORD"
#endif
#ifndef CIA_UUID
#define CIA_UUID    "YOUR_DEVICE_PASSWORD"
#endif

#define CIA_HOST    "47.113.229.47"
#define CIA_PORT    3007

// 平台「指令设置」里的通道号：2=灯，3=电机
#define CIA_CH_LED     2
#define CIA_CH_MOTOR   3

// 平台若下发「指令内容」数字：3/4/5/6 依次是 灯开/灯关/电机开/电机关
#define CIA_NUM_LED_ON    3
#define CIA_NUM_LED_OFF   4
#define CIA_NUM_MOTOR_ON  5
#define CIA_NUM_MOTOR_OFF 6

#define TCP_PORT    8266
#define SERIAL_BAUD 9600

#define PIN_LED    13
#define PIN_RELAY   4
#define PIN_AIN1   25
#define PIN_AIN2   26
#define PIN_PWMA   27

#define LED_ACTIVE_LOW   0     // 共阴=0，共阳=1
#define RELAY_ACTIVE_LOW 1     // 跳线 LOW 时低电平吸合
#define MOTOR_FORWARD    1     // 转向反了改 0（仅接驱动）

#define USE_DRIVER    0        // 0=继电器直带电机，1=接 TB6612
#define PWM_FREQ      20000
#define PWM_RES       8
#define PWM_MAX       255
#define PWM_LIMIT     204      // 80%
#define DEFAULT_DUTY  153      // 60%
#define POWER_ON_MS   150      // 合闸后等待（仅接驱动）
#define RAMP_MS       300      // 软启软停（仅接驱动）
#define IDLE_FLUSH_MS 120      // 无换行时的分帧静默时间
#define RETRY_MS      5000     // 重连间隔
#define CIA_SNIFF     1        // 1=打印 CIA 原始字节

// ================================================================
// LEDC（PWM）兼容层：Arduino 核心 2.x 和 3.x 的 PWM 函数名不一样，
// 这段让代码在两个版本上都能编译，你不用管。
// ================================================================
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define LEDC_SETUP()      ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES)
  #define LEDC_WRITE(duty)  ledcWrite(PIN_PWMA, duty)
#else
  #define LEDC_CH           0
  #define LEDC_SETUP()      do { ledcSetup(LEDC_CH, PWM_FREQ, PWM_RES); \
                                 ledcAttachPin(PIN_PWMA, LEDC_CH); } while (0)
  #define LEDC_WRITE(duty)  ledcWrite(LEDC_CH, duty)
#endif

// ===== 2. 状态 =====
enum MotorState { M_IDLE = 0, M_POWER_ON, M_RAMP_UP, M_RUN, M_RAMP_DOWN, M_POWER_OFF };

MotorState mState = M_IDLE;
uint32_t   stateT0 = 0;
uint32_t   rampT0 = 0;
int        rampFrom = 0;
int        rampTo = 0;
int        dutyNow = 0;
int        dutyTarget = 0;

bool ledOn = false;
bool relayOn = false;

struct Channel {
  String   buf;
  uint32_t lastByte;
};
Channel chSerial, chLan;

WiFiClient cia;
WiFiClient lanClient;
WiFiServer server(TCP_PORT);

bool     wifiUp = false;
uint32_t wifiT0 = 0;
uint32_t ciaT0 = 0;
String   ciaRx;

// ===== 3. 输出 =====
static inline bool levelFor(bool on, bool activeLow) {
  return activeLow ? !on : on;
}

void ledSet(bool on) {
  ledOn = on;
  digitalWrite(PIN_LED, levelFor(on, LED_ACTIVE_LOW));
}

void pwmApply(int duty) {
  if (duty < 0) duty = 0;
  if (duty > PWM_LIMIT) duty = PWM_LIMIT;
  dutyNow = duty;
  LEDC_WRITE(duty);
}

// 继电器是电机总电源，断开时同时复位电机状态
void relaySet(bool on) {
  relayOn = on;
  digitalWrite(PIN_RELAY, levelFor(on, RELAY_ACTIVE_LOW));
  if (!on) {
    pwmApply(0);
    dutyTarget = 0;
    mState = M_IDLE;
  }
}

// ===== 4. 电机 =====
#if USE_DRIVER
void motorDirection() {
  bool fwd = (MOTOR_FORWARD != 0);
  digitalWrite(PIN_AIN1, fwd ? HIGH : LOW);
  digitalWrite(PIN_AIN2, fwd ? LOW : HIGH);
}
#endif

void motorRequest(uint8_t pct) {
  if (pct == 0) return;

#if USE_DRIVER
  int target = (pct <= 5) ? DEFAULT_DUTY : (int)pct * PWM_MAX / 100;
  if (target > PWM_LIMIT) target = PWM_LIMIT;
  dutyTarget = target;

  if (mState == M_IDLE || mState == M_POWER_OFF) {
    motorDirection();
    relaySet(true);
    mState  = M_POWER_ON;
    stateT0 = millis();
  } else {
    rampFrom = dutyNow;
    rampTo   = dutyTarget;
    rampT0   = millis();
    mState   = M_RAMP_UP;
  }
#else
  dutyTarget = 0;
  dutyNow    = 0;
  relaySet(true);
  mState     = M_RUN;
#endif
}

void motorStop() {
#if USE_DRIVER
  dutyTarget = 0;
  if (mState == M_IDLE || mState == M_POWER_OFF) return;
  rampFrom = dutyNow;
  rampTo   = 0;
  rampT0   = millis();
  mState   = M_RAMP_DOWN;
#else
  relaySet(false);
#endif
}

void allOff() {
  pwmApply(0);
  dutyTarget = 0;
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  relaySet(false);
  ledSet(false);
  mState = M_IDLE;
}

void updateMotor() {
  uint32_t now = millis();
  switch (mState) {
    case M_POWER_ON:
      if (now - stateT0 >= POWER_ON_MS) {
        rampFrom = 0;
        rampTo   = dutyTarget;
        rampT0   = now;
        mState   = M_RAMP_UP;
      }
      break;

    case M_RAMP_UP:
    case M_RAMP_DOWN: {
      uint32_t elapsed = now - rampT0;
      if (elapsed >= RAMP_MS) {
        pwmApply(rampTo);
        if (rampTo == 0) {
          mState  = M_POWER_OFF;
          stateT0 = now;
        } else {
          mState = M_RUN;
        }
      } else {
        pwmApply(rampFrom + (int)((long)(rampTo - rampFrom) * (long)elapsed / RAMP_MS));
      }
      break;
    }

    case M_RUN:
      if (dutyTarget != dutyNow) {
        rampFrom = dutyNow;
        rampTo   = dutyTarget;
        rampT0   = now;
        mState   = (rampTo > rampFrom) ? M_RAMP_UP : M_RAMP_DOWN;
      }
      break;

    case M_POWER_OFF:
      digitalWrite(PIN_AIN1, LOW);
      digitalWrite(PIN_AIN2, LOW);
      relaySet(false);
      pwmApply(0);
      mState = M_IDLE;
      break;

    default:
      break;
  }
}

// ===== 5. 状态与回复 =====
String stateReport() {
#if USE_DRIVER
  bool running = (dutyNow > 0);
  int  pct     = (dutyNow * 100) / PWM_MAX;
#else
  bool running = (mState == M_RUN) && relayOn;
  int  pct     = running ? 100 : 0;
#endif
  return String("STATE LED=") + (ledOn ? 1 : 0) +
         " RELAY=" + (relayOn ? 1 : 0) +
         " MOTOR=" + (running ? 1 : 0) +
         " DUTY=" + pct +
         " IP=" + WiFi.localIP().toString();
}

void reply(const char *src, const String &msg) {
  Serial.print("[");
  Serial.print(src);
  Serial.print("] ");
  Serial.println(msg);
  if (strcmp(src, "LAN") == 0 && lanClient && lanClient.connected()) {
    lanClient.println(msg);
  }
}

// ===== 6. 命令解析 =====
bool valueAfter(const String &s, const char *key, int &out) {
  int idx = s.indexOf(key);
  if (idx < 0) return false;

  int i = idx + (int)strlen(key);
  int n = (int)s.length();
  while (i < n && (s[i] == ' ' || s[i] == '\t')) i++;
  if (i < n && (s[i] == ':' || s[i] == '=')) i++;
  while (i < n && s[i] == ' ') i++;

  bool any = false;
  long v = 0;
  while (i < n && isDigit((unsigned char)s[i])) {
    v = v * 10 + (s[i] - '0');
    if (v > 999) break;
    i++;
    any = true;
  }
  if (!any) return false;
  out = (int)v;
  return true;
}

// 取 at 前面紧挨着的数字，如 @!2powerOn#% 里的 2
int digitsBefore(const String &s, int at) {
  int end = at - 1;
  int start = end;
  while (start >= 0 && isDigit((unsigned char)s[start])) start--;
  start++;
  if (start > end) return -1;
  return s.substring(start, end + 1).toInt();
}

bool allDigits(const String &s) {
  if (s.length() == 0) return false;
  for (unsigned int i = 0; i < s.length(); i++) {
    if (!isDigit((unsigned char)s[i])) return false;
  }
  return true;
}

void handleCommand(String raw, const char *src) {
  raw.trim();
  if (raw.length() == 0) return;

  // 剥掉 ~<设备密码>| 前缀
  String s = raw;
  int bar = s.lastIndexOf('|');
  if (bar >= 0) s = s.substring(bar + 1);
  s.trim();

  String u = s;
  u.toUpperCase();

  Serial.print("CMD<");
  Serial.print(src);
  Serial.print("> ");
  Serial.println(s);

  if (u.indexOf("STATUS") >= 0) {
    reply(src, stateReport());
    return;
  }

  // @!<通道>powerOn#% / @!<通道>powerOff#%
  int atOn  = u.indexOf("POWERON");
  int atOff = u.indexOf("POWEROFF");
  if (atOn >= 0 || atOff >= 0) {
    bool on = (atOn >= 0);
    int  ch = digitsBefore(u, on ? atOn : atOff);
    if (ch == CIA_CH_LED) {
      ledSet(on);
      reply(src, stateReport());
      return;
    }
    if (ch == CIA_CH_MOTOR) {
      if (on) motorRequest(1);
      else    motorStop();
      reply(src, stateReport());
      return;
    }
    reply(src, String("ERR unknown CIA channel: ") + ch);
    return;
  }

  // 整串就是一个数字：3/4/5/6
  if (s.length() <= 2 && allDigits(s)) {
    int n = s.toInt();
    if (n == CIA_NUM_LED_ON)    { ledSet(true);    reply(src, stateReport()); return; }
    if (n == CIA_NUM_LED_OFF)   { ledSet(false);   reply(src, stateReport()); return; }
    if (n == CIA_NUM_MOTOR_ON)  { motorRequest(1); reply(src, stateReport()); return; }
    if (n == CIA_NUM_MOTOR_OFF) { motorStop();     reply(src, stateReport()); return; }
  }

  int v = 0;
  if (valueAfter(u, "ALL", v)) {
    allOff();
    reply(src, stateReport());
    return;
  }
  if (valueAfter(u, "LED", v)) {
    ledSet(v > 0);
    reply(src, stateReport());
    return;
  }
  if (valueAfter(u, "RELAY", v)) {
    relaySet(v > 0);
    reply(src, stateReport());
    return;
  }
  if (valueAfter(u, "MOTOR", v)) {
    if (v <= 0) motorStop();
    else        motorRequest((uint8_t)v);
    reply(src, stateReport());
    return;
  }

  reply(src, String("ERR unknown cmd: ") + s);
}

// ===== 7. 分帧 =====
void feed(Channel &ch, uint8_t b, const char *src) {
  if (b == '\r') return;

  ch.lastByte = millis();
  if (b == '\n') {
    String line = ch.buf;
    ch.buf = "";
    if (line.length()) handleCommand(line, src);
    return;
  }
  if (ch.buf.length() > 120) ch.buf.remove(0, ch.buf.length() - 120);
  ch.buf += (char)b;
}

void flushIdle(Channel &ch, const char *src) {
  if (ch.buf.length() == 0) return;
  if (millis() - ch.lastByte < IDLE_FLUSH_MS) return;

  String line = ch.buf;
  ch.buf = "";
  handleCommand(line, src);
}

// ===== 8. CIA 通道 =====
#if CIA_SNIFF
void ciaSniff(uint8_t b) {
  static uint8_t buf[16];
  static char    asc[17];
  static uint8_t n = 0;

  buf[n] = b;
  asc[n] = (b >= 0x20 && b < 0x7F) ? (char)b : '.';
  n++;
  if (n < 16) return;

  Serial.print("CIA HEX");
  for (uint8_t i = 0; i < 16; i++) {
    Serial.print(' ');
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
  }
  asc[16] = 0;
  Serial.print("  |");
  Serial.print(asc);
  Serial.println('|');
  n = 0;
}
#endif

// 按大括号配平切出完整 JSON；字符串内的括号不算
int jsonObjectLength(const String &s) {
  int start = s.indexOf('{');
  if (start < 0) return -1;

  int depth = 0;
  bool inStr = false, esc = false;
  for (int i = start; i < (int)s.length(); i++) {
    char c = s[i];
    if (inStr) {
      if (esc)            esc = false;
      else if (c == '\\') esc = true;
      else if (c == '"')  inStr = false;
      continue;
    }
    if (c == '"')      inStr = true;
    else if (c == '{') depth++;
    else if (c == '}') {
      depth--;
      if (depth == 0) return i - start + 1;
    }
  }
  return 0;
}

bool jsonValue(const String &obj, const char *key, String &out) {
  int k = obj.indexOf(String("\"") + key + "\"");
  if (k < 0) return false;

  int c = obj.indexOf(':', k + (int)strlen(key) + 2);
  if (c < 0) return false;

  int i = c + 1;
  int n = (int)obj.length();
  while (i < n && (obj[i] == ' ' || obj[i] == '\t')) i++;

  if (i < n && obj[i] == '"') {
    String v;
    bool esc = false;
    for (i++; i < n; i++) {
      char ch = obj[i];
      if (esc)        { v += ch; esc = false; continue; }
      if (ch == '\\') { esc = true; continue; }
      if (ch == '"')  break;
      v += ch;
    }
    out = v;
    return true;
  }

  int start = i;
  while (i < n && obj[i] != ',' && obj[i] != '}') i++;
  out = obj.substring(start, i);
  out.trim();
  return out.length() > 0;
}

// 回执必须带原 id，否则平台一直显示「执行中」
void replyDirective(const String &id, bool ok, const char *msg) {
  String r = String("{\"type\":\"directive\",\"id\":\"") + id +
             "\",\"message\":\"" + msg +
             "\",\"status\":\"" + (ok ? "ok" : "error") + "\"}";
  cia.print(r);
  Serial.print("CIA TX ");
  Serial.println(r);
}

void handleDirective(const String &obj) {
  String type, id, code;
  jsonValue(obj, "type", type);
  jsonValue(obj, "id", id);
  jsonValue(obj, "code", code);

  if (type != "directive" || code.length() == 0) {
    Serial.print("CIA JSON (ignored) ");
    Serial.println(obj);
    return;
  }

  id.trim();
  Serial.print("DIRECTIVE id=");
  Serial.print(id);
  Serial.print(" code=");
  Serial.println(code);

  handleCommand(code, "CIA");
  replyDirective(id, true, "执行完成");
}

void ciaTask() {
  if (WiFi.status() != WL_CONNECTED) {
    if (cia.connected()) cia.stop();
    return;
  }
  if (cia.connected()) return;
  if (millis() - ciaT0 < RETRY_MS) return;
  ciaT0 = millis();

  Serial.print("CIA connecting ");
  Serial.print(CIA_HOST);
  Serial.print(':');
  Serial.print(CIA_PORT);

  if (!cia.connect(CIA_HOST, CIA_PORT, 1500)) {
    Serial.println(" -> FAILED, retry in 5s");
    return;
  }
  cia.print(CIA_UUID);            // 认证帧：只发设备密码
  Serial.println(" -> OK (device password sent)");
  ciaRx = "";
}

// ===== 9. 网络 =====
void wifiTask() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiUp) {
      wifiUp = true;
      Serial.print("WiFi OK, IP=");
      Serial.println(WiFi.localIP());
      server.begin();
      server.setNoDelay(true);
      Serial.print("TCP server on port ");
      Serial.println(TCP_PORT);
    }
    return;
  }

  if (wifiUp) {
    wifiUp = false;
    Serial.println("WiFi lost, retrying...");
  }
  if (millis() - wifiT0 < RETRY_MS) return;
  wifiT0 = millis();
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("WiFi connecting...");
}

// 只保留一个客户端，新连接顶掉旧的
void serverTask() {
  WiFiClient nc = server.available();
  if (nc && nc.connected()) {
    if (lanClient && lanClient.connected()) {
      lanClient.stop();
      Serial.println("LAN: replace old client");
    }
    lanClient = nc;
    Serial.print("LAN client: ");
    Serial.println(lanClient.remoteIP());
    lanClient.println("ESP32 ready. LED:1 RELAY:1 MOTOR:1 STATUS? ALL:0");
  }
  if (lanClient && !lanClient.connected()) {
    lanClient.stop();
    Serial.println("LAN client gone");
  }
}

// ===== 10. setup / loop =====
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);

  LEDC_SETUP();
  pwmApply(0);
  ledSet(false);
  relaySet(false);

  chSerial.lastByte = chLan.lastByte = millis();

  Serial.println();
  Serial.println("=== ESP32 CIA remote control (task 2) ===");
  Serial.println("LED:1 LED:0 | RELAY:1 RELAY:0 | MOTOR:1 MOTOR:0 MOTOR:6-100");
  Serial.println("STATUS? | ALL:0");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiT0 = millis();
  Serial.print("WiFi connecting to ");
  Serial.println(WIFI_SSID);
}

void loop() {
  wifiTask();
  ciaTask();
  serverTask();

  while (Serial.available()) feed(chSerial, (uint8_t)Serial.read(), "SER");

  if (lanClient && lanClient.connected()) {
    while (lanClient.available()) feed(chLan, (uint8_t)lanClient.read(), "LAN");
  }

  if (cia.connected()) {
    while (cia.available()) {
      uint8_t b = (uint8_t)cia.read();
#if CIA_SNIFF
      ciaSniff(b);
#endif
      ciaRx += (char)b;
      if (ciaRx.length() > 900) ciaRx.remove(0, ciaRx.length() - 600);

      for (;;) {
        int len = jsonObjectLength(ciaRx);
        if (len <= 0) break;
        int start = ciaRx.indexOf('{');
        String obj = ciaRx.substring(start, start + len);
        ciaRx.remove(0, start + len);
        handleDirective(obj);
      }
    }
  }

  flushIdle(chSerial, "SER");
  flushIdle(chLan, "LAN");

  updateMotor();
}

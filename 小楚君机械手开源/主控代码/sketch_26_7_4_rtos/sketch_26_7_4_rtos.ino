// ========================================================================
//  xiaochu 机械手固件 - ESP32-C3 + PCA9685
//  【FreeRTOS 多任务改造版】
// ------------------------------------------------------------------------
//  与原版 sketch_26_7_4.ino 的差别（协议、引脚、标定参数完全不变）：
//    1. 指令不再在 BLE 回调里直接操作舵机，而是塞进队列
//    2. 新增 motionTask：独占 I2C，按 10ms 固定节拍推进插值
//    3. loop() 只负责"断连后重开广播"这一件事
//    4. 断连时自动回到张开位（通过队列下发，不跨任务改状态）
//    5. 舵机没动就不写 I2C，减少总线占用
//    6. 新增带 XOR 校验的文本帧协议（$S1,120*7D），旧文本指令仍兼容
//
//  任务划分：
//    BT task（协议栈自带，19 左右）→ onWrite 只做解析 + 入队（微秒级）
//    motionTask（优先级 4）         → 出队 + 插值 + pwm.setPWM
//    loopTask（优先级 1）           → 广播看护
//
//  关键约束：pwm.setPWM() 内部走 Wire（I2C），全工程只允许 motionTask 调用。
// ========================================================================

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ---------- I2C 引脚 ----------
#define SDA_PIN 8
#define SCL_PIN 9

// ---------- PCA9685 参数 ----------
#define PCA9685_ADDR 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// ★★★ 在这里校准你的舵机脉宽范围（单位是 PCA9685 计数值，不是微秒）★★★
#define SERVO_MIN_PULSE 150   // 对应 0°（初始建议值，需要实测）
#define SERVO_MAX_PULSE 500   // 对应 180°（初始建议值，需要实测）

#define SERVO_FREQ 50         // 50Hz

// ---------- 延迟测量标记脚（D1）----------
// 接逻辑分析仪 / 示波器的 CH1：收到 BLE 指令时拉高，写完 PWM 后拉低，
// 高电平宽度就是「板内处理延迟」（不含无线传输和舵机机械响应）。
// 若该脚在你的板子上另有用途，可换成 3 / 5 / 6 / 7 / 10 中任一空闲脚。
#define MARK_PIN 4
#define MARK_HIGH() digitalWrite(MARK_PIN, HIGH)
#define MARK_LOW()  digitalWrite(MARK_PIN, LOW)

// ---------- BLE UUID（与上位机一致）----------
#define SERVICE_UUID           "89d60870-9908-4472-8f8c-e5b3e6573cd1"
#define WRITE_CHAR_UUID        "39dea685-a63e-44b2-8819-9a202581f8fe"

// ---------- 手指定义 ----------
#define FINGER_THUMB  0
#define FINGER_INDEX  1
#define FINGER_MIDDLE 2
#define FINGER_RING   3
#define FINGER_PINKY  4

#define THUMB_MAX_OPEN   30
#define THUMB_MAX_CLOSED 180
#define INDEX_MAX_OPEN   10
#define INDEX_MAX_CLOSED 180
#define MIDDLE_MAX_OPEN  180
#define MIDDLE_MAX_CLOSED 0
#define RING_MAX_OPEN    0
#define RING_MAX_CLOSED  180
#define PINKY_MAX_OPEN   180
#define PINKY_MAX_CLOSED 35

#define DEFAULT_STEP 2.0f     // 每个节拍走多少度

// ---------- 任务参数 ----------
#define MOTION_PERIOD_MS 10   // 运动任务节拍 = 原 loop() 的 delay(10)
#define CMD_QUEUE_LEN    8    // 指令队列深度
#define MOTION_STACK     4096 // 单位：字节（ESP-IDF 的 xTaskCreate 用字节）
#define MOTION_PRIORITY  4

// 打开后串口每 5 秒打印一次任务栈水位，调完栈大小记得关掉
#define RTOS_DEBUG 1

// ========================================================================
//  一、机械层（与原版一致，只把 run() 改成"动了才写 I2C"）
// ========================================================================
class Finger {
public:
  int channel;
  int maxOpen, maxClosed;
  int direction;
  float currentPosition, target, step;

  Finger(int ch, int open, int closed, int dir)
    : channel(ch), maxOpen(open), maxClosed(closed), direction(dir) {
    currentPosition = (direction == 1) ? maxOpen : maxClosed;
    target = currentPosition;
    step = DEFAULT_STEP;
  }

  void setTargetNormalized(int normalizedPos) {
    if (direction == 1) {
      target = map(normalizedPos, 0, 100, maxOpen, maxClosed);
    } else {
      target = map(normalizedPos, 0, 100, maxClosed, maxOpen);
    }
    target = constrain(target, 0, 180);
  }

  void setTargetAngle(int angle) {
    target = constrain(angle, 0, 180);
  }

  // 返回 true 表示这一拍位置真的变了（于是需要写一次 I2C）
  bool run() {
    float before = currentPosition;

    if (fabsf(currentPosition - target) <= step) {
      currentPosition = target;
    } else if (currentPosition < target) {
      currentPosition += step;
    } else {
      currentPosition -= step;
    }

    if (currentPosition == before) return false;   // 已经到位，不浪费总线

    int angle = (int)(currentPosition + 0.5f);     // 四舍五入
    int pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(channel, 0, pulse);
    return true;
  }

  bool isStill() { return fabsf(currentPosition - target) <= step; }
  void setStep(float s) { step = s; }
};

class Hand {
public:
  Finger* fingers[5];
  Hand() {
    fingers[0] = new Finger(FINGER_THUMB,  THUMB_MAX_OPEN,  THUMB_MAX_CLOSED,  1);
    fingers[1] = new Finger(FINGER_INDEX,  INDEX_MAX_OPEN,  INDEX_MAX_CLOSED,  1);
    fingers[2] = new Finger(FINGER_MIDDLE, MIDDLE_MAX_OPEN, MIDDLE_MAX_CLOSED, -1);
    fingers[3] = new Finger(FINGER_RING,   RING_MAX_OPEN,   RING_MAX_CLOSED,   1);
    fingers[4] = new Finger(FINGER_PINKY,  PINKY_MAX_OPEN,  PINKY_MAX_CLOSED, -1);
  }
  void moveToNormalized(int pos) { for (int i=0;i<5;i++) fingers[i]->setTargetNormalized(pos); }
  void moveFingerNormalized(int finger, int pos) { if (finger>=0&&finger<5) fingers[finger]->setTargetNormalized(pos); }
  void moveFingerAngle(int finger, int angle) { if (finger>=0&&finger<5) fingers[finger]->setTargetAngle(angle); }

  // 返回 true 表示这一拍有手指动过
  bool run() {
    bool moved = false;
    for (int i=0;i<5;i++) if (fingers[i]->run()) moved = true;
    return moved;
  }

  bool isStill() { for (int i=0;i<5;i++) if (!fingers[i]->isStill()) return false; return true; }
  void setStep(float s) { for (int i=0;i<5;i++) fingers[i]->setStep(s); }
};

// ========================================================================
//  二、手势层（与原版一致）
// ========================================================================
class HandMovementFactory {
public:
  static void applyGesture(Hand* hand, const String& name) {
    if (name == "fist") {
      hand->moveToNormalized(100);
    } else if (name == "five") {
      hand->moveToNormalized(0);
    } else if (name == "ok") {
      hand->fingers[0]->setTargetNormalized(0);
      hand->fingers[1]->setTargetNormalized(100);
      hand->fingers[2]->setTargetNormalized(100);
      hand->fingers[3]->setTargetNormalized(100);
      hand->fingers[4]->setTargetNormalized(100);
    } else if (name == "rock") {
      hand->fingers[0]->setTargetNormalized(100);
      hand->fingers[1]->setTargetNormalized(0);
      hand->fingers[2]->setTargetNormalized(100);
      hand->fingers[3]->setTargetNormalized(100);
      hand->fingers[4]->setTargetNormalized(0);
    } else if (name == "love") {
      hand->fingers[0]->setTargetNormalized(0);
      hand->fingers[1]->setTargetNormalized(0);
      hand->fingers[2]->setTargetNormalized(100);
      hand->fingers[3]->setTargetNormalized(100);
      hand->fingers[4]->setTargetNormalized(0);
    } else if (name == "one") {
      hand->fingers[0]->setTargetNormalized(100);
      hand->fingers[1]->setTargetNormalized(0);
      hand->fingers[2]->setTargetNormalized(100);
      hand->fingers[3]->setTargetNormalized(100);
      hand->fingers[4]->setTargetNormalized(100);
    } else if (name == "two") {
      hand->fingers[0]->setTargetNormalized(100);
      hand->fingers[1]->setTargetNormalized(0);
      hand->fingers[2]->setTargetNormalized(0);
      hand->fingers[3]->setTargetNormalized(100);
      hand->fingers[4]->setTargetNormalized(100);
    } else if (name == "three") {
      hand->fingers[0]->setTargetNormalized(100);
      hand->fingers[1]->setTargetNormalized(0);
      hand->fingers[2]->setTargetNormalized(0);
      hand->fingers[3]->setTargetNormalized(0);
      hand->fingers[4]->setTargetNormalized(100);
    } else if (name == "four") {
      hand->fingers[0]->setTargetNormalized(100);
      hand->fingers[1]->setTargetNormalized(0);
      hand->fingers[2]->setTargetNormalized(0);
      hand->fingers[3]->setTargetNormalized(0);
      hand->fingers[4]->setTargetNormalized(0);
    } else if (name == "scratch") {
      hand->fingers[0]->setTargetNormalized(0);
      hand->fingers[1]->setTargetNormalized(100);
      hand->fingers[2]->setTargetNormalized(0);
      hand->fingers[3]->setTargetNormalized(0);
      hand->fingers[4]->setTargetNormalized(0);
    } else if (name == "come") {
      hand->fingers[0]->setTargetNormalized(0);
      hand->fingers[1]->setTargetNormalized(0);
      hand->fingers[2]->setTargetNormalized(100);
      hand->fingers[3]->setTargetNormalized(0);
      hand->fingers[4]->setTargetNormalized(0);
    } else {
      Serial.printf("Unknown gesture: %s\n", name.c_str());
    }
  }
};

// ========================================================================
//  三、跨任务通信：一条指令就是一个结构体
// ------------------------------------------------------------------------
//  为什么用结构体而不是 String 跨任务传递？
//    String 会 new 内存，在 BLE 任务里 new、在运动任务里 delete，
//    长期运行会产生堆碎片。定长结构体直接按值拷贝，零分配。
// ========================================================================
enum CmdType : uint8_t {
  CMD_GESTURE = 0,   // 手势名
  CMD_ANGLE   = 1,   // 单指角度 servo_n=angle
  CMD_HOME    = 2    // 回张开位（断连时用）
};

struct HandCommand {
  CmdType type;
  uint8_t finger;
  uint8_t angle;
  char    gesture[16];
  uint32_t tRx;     // 收到指令的时刻（micros），跟着指令走，避免多条积压时串味
};

static QueueHandle_t cmdQueue = nullptr;
static Hand hand;                          // 只由 motionTask 修改，因此不需要互斥锁
static volatile bool bleConnected = false;
static volatile bool needRestartAdv = false;
static volatile bool markPending = false;  // 标记脚是否正等待本次指令执行完

// ---------- 队列与断连保护实验（D2）----------
// 三个实验全部由固件自己产生指令流，不需要上位机配合，也不需要任何仪器：
//   test=flood   瞬间灌 100 条，验证"丢旧保新"
//   test=stream  20Hz 连发 3 秒，测负载下的延迟与跟手性
//   test=estop   持续扫动，由人手断开蓝牙，测断连→静止的耗时
#define TEST_ENABLE 1
static QueueHandle_t testReqQ = nullptr;

static volatile uint32_t qDrop = 0;        // 因队列满被丢弃的指令数
static volatile uint32_t qDepthMax = 0;    // 队列历史最大占用

// 沉降检测：从某个时刻起，等到五根手指全部静止，打印耗时
enum SettleKind : uint8_t { SETTLE_NONE = 0, SETTLE_FLOOD, SETTLE_STREAM, SETTLE_DISC };
static volatile uint8_t  settleKind = SETTLE_NONE;
static volatile uint32_t settleRefUs = 0;
static bool settleSeenMoving = false;      // 先看到"在动"，再看到"静止"才算一次沉降

// ---------- 延迟统计（D1）----------
// 单次测量没有意义：指令落在 10ms 节拍窗口的位置是随机的，
// 真正的指标是分布范围（jitter）而不是平均值。
static uint32_t latN = 0, latMin = 0xFFFFFFFFu, latMax = 0, latSum = 0;
static void latRecord(uint32_t dtUs) {
  if (dtUs < latMin) latMin = dtUs;
  if (dtUs > latMax) latMax = dtUs;
  latSum += dtUs;
  latN++;
  if (latN % 20 == 0) {      // 每 20 条汇总一次
    Serial.printf("[LAT-STAT] n=%lu  min=%lu  max=%lu  avg=%lu us\n",
                  (unsigned long)latN,
                  (unsigned long)latMin,
                  (unsigned long)latMax,
                  (unsigned long)(latSum / latN));
  }
}

// 入队：队列满时丢最旧的一条，保证"最新指令一定生效"
static bool enqueueCommand(const HandCommand& cmd) {
  if (cmdQueue == nullptr) return false;

  UBaseType_t depth = uxQueueMessagesWaiting(cmdQueue);
  if (depth > qDepthMax) qDepthMax = depth;

  if (xQueueSend(cmdQueue, &cmd, 0) == pdTRUE) return true;

  // 队列已满：扔掉最旧的一条，再放新的进去
  HandCommand dropped;
  xQueueReceive(cmdQueue, &dropped, 0);
  qDrop++;
  return xQueueSend(cmdQueue, &cmd, 0) == pdTRUE;
}

// ========================================================================
//  三点五、协议层：文本帧 + XOR 校验（D3）
// ------------------------------------------------------------------------
//  帧格式：  $<body>*<HH>
//    body = S<finger>,<angle>    例：S1,120
//           G<gesture>           例：GFIST
//    HH   = body 全部字节按位异或，两位十六进制（大小写都收）
//
//  为什么要加校验：
//    原版是 startsWith("servo_") 直接解析文本。BLE 丢字节或截断时会产生
//    "servo_1=1" 这种残帧，它照样 toInt() 成一个合法角度并执行 —— 错误
//    指令被当成正常指令跑掉，日志里看不出任何异常。加校验后残帧一律拒收，
//    并且能报出拒收原因。
//
//  为什么还是文本而不是二进制：
//    上位机输入框、nRF Connect 都能直接手输，调试成本最低。二进制帧在
//    手机上没法手打，这个项目不值得为它牺牲可测性。
//
//  为什么不用 String：
//    这段代码跑在 BLE 回调里（协议栈任务上下文），堆分配会长期留下碎片。
//    全程 const char* + 逐字符解析，零分配。
// ========================================================================
enum ProtoErr : uint8_t {
  PROTO_OK = 0,
  PROTO_ERR_START,     // 不是以 '$' 开头
  PROTO_ERR_NO_STAR,   // 缺少 '*' 分隔符
  PROTO_ERR_CHK_LEN,   // 校验位不是 2 位十六进制
  PROTO_ERR_CHK,       // 校验值不匹配
  PROTO_ERR_CMD,       // 未知指令类型
  PROTO_ERR_RANGE,     // 参数越界
  PROTO_ERR_FORMAT     // 字段残缺
};
#define PROTO_ERR_COUNT 8

static const char* const PROTO_ERR_NAME[PROTO_ERR_COUNT] = {
  "OK", "no-start", "no-star", "chk-len", "chk-mismatch", "bad-cmd", "range", "format"
};

static volatile uint32_t protoOk = 0;                  // 通过校验并执行的帧数
static volatile uint32_t protoErr[PROTO_ERR_COUNT] = {0};  // 各类拒收计数

static uint8_t hexVal(char c) {
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
  return 0xFF;                                    // 非法字符
}

// 解析一帧。成功返回 PROTO_OK 并把结果写进 out（不改 out.tRx，调用者填）。
// calcOut 非空时回填实际算出的校验值，用于把"期望值"打印给手敲帧的人。
static ProtoErr parseFrame(const char* s, size_t len, HandCommand& out,
                           uint8_t* calcOut) {
  // 有些客户端会自动补 0x0A / 0x0D，先削掉尾部空白再算长度
  while (len > 0 && (s[len - 1] == '\n' || s[len - 1] == '\r' || s[len - 1] == ' ')) len--;

  if (len < 4 || s[0] != '$') return PROTO_ERR_START;

  size_t star = 0;
  for (size_t i = 1; i < len; i++) {
    if (s[i] == '*') { star = i; break; }
  }
  if (star == 0) return PROTO_ERR_NO_STAR;
  if (star + 3 > len) return PROTO_ERR_CHK_LEN;    // 校验位不够 2 位
  if (star + 3 < len) return PROTO_ERR_FORMAT;     // 校验位后面还有多余字节

  uint8_t hi = hexVal(s[star + 1]);
  uint8_t lo = hexVal(s[star + 2]);
  if (hi == 0xFF || lo == 0xFF) return PROTO_ERR_CHK_LEN;
  uint8_t expectChk = (uint8_t)((hi << 4) | lo);

  uint8_t calcChk = 0;
  for (size_t i = 1; i < star; i++) calcChk ^= (uint8_t)s[i];
  if (calcOut) *calcOut = calcChk;
  if (calcChk != expectChk) return PROTO_ERR_CHK;

  const char kind = s[1];

  if (kind == 'S' || kind == 's') {
    // S<finger>,<angle>
    const char* p   = s + 2;
    const char* end = s + star;
    if (p >= end || *p < '0' || *p > '9') return PROTO_ERR_FORMAT;

    long finger = 0;
    while (p < end && *p >= '0' && *p <= '9') { finger = finger * 10 + (*p - '0'); p++; }
    if (p >= end || *p != ',') return PROTO_ERR_FORMAT;
    p++;
    if (p >= end || *p < '0' || *p > '9') return PROTO_ERR_FORMAT;

    long angle = 0;
    while (p < end && *p >= '0' && *p <= '9') { angle = angle * 10 + (*p - '0'); p++; }
    if (p != end) return PROTO_ERR_FORMAT;         // 角度后面还有杂物

    if (finger < 0 || finger > 4) return PROTO_ERR_RANGE;
    if (angle  < 0 || angle  > 180) return PROTO_ERR_RANGE;

    out.type    = CMD_ANGLE;
    out.finger  = (uint8_t)finger;
    out.angle   = (uint8_t)angle;
    out.gesture[0] = '\0';
    return PROTO_OK;
  }

  if (kind == 'G' || kind == 'g') {
    size_t n = star - 2;                           // 手势名长度
    if (n == 0 || n >= sizeof(out.gesture)) return PROTO_ERR_FORMAT;
    for (size_t i = 0; i < n; i++) {
      char c = s[2 + i];
      if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');   // 统一小写
      out.gesture[i] = c;
    }
    out.gesture[n] = '\0';
    out.type   = CMD_GESTURE;
    out.finger = 0;
    out.angle  = 0;
    return PROTO_OK;
  }

  return PROTO_ERR_CMD;
}

// ========================================================================
//  四、BLE 层：回调里只解析 + 入队，绝不碰舵机
// ========================================================================
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    auto value = pCharacteristic->getValue();
    if (value.length() == 0) return;

    const char* raw = value.c_str();
    size_t rawLen   = value.length();

    // ---- 优先走帧路径：'$' 开头，全程不碰 String ----
    if (raw[0] == '$') {
      HandCommand c = {};
      uint8_t calc = 0;
      ProtoErr e = parseFrame(raw, rawLen, c, &calc);

      if (e != PROTO_OK) {
        protoErr[e]++;
        // 校验错时把正确值打出来：这个协议是手敲的，不给提示根本没法调
        if (e == PROTO_ERR_CHK) {
          Serial.printf("[PROTO] reject: %s | calc should be *%02X | raw=%s\n",
                        PROTO_ERR_NAME[e], calc, raw);
        } else {
          Serial.printf("[PROTO] reject: %s | raw=%s\n", PROTO_ERR_NAME[e], raw);
        }
        return;                       // 拒收：不入队、不动作
      }

      protoOk++;
      c.tRx = micros();
      if (!enqueueCommand(c)) {
        Serial.println("[QUEUE] enqueue failed");
        return;
      }
      markPending = true;
      MARK_HIGH();
#if RTOS_DEBUG
      Serial.printf("[PROTO] ok: type=%d finger=%u angle=%u chk=%02X\n",
                    c.type, c.finger, c.angle, calc);
#endif
      return;
    }

    String cmd = String(raw);
    cmd.trim();
    if (cmd.length() == 0) return;

#if TEST_ENABLE
    // 测试指令：不进队列，只唤醒测试任务
    if (cmd.startsWith("test=")) {
      String which = cmd.substring(5);
      which.trim();
      uint8_t id = 0;
      if      (which == "flood")  id = 1;
      else if (which == "stream") id = 2;
      else if (which == "estop")  id = 3;
      else if (which == "proto")  id = 4;
      else {
        Serial.println("[TEST] unknown. use test=flood | test=stream | test=estop | test=proto");
        return;
      }
      if (testReqQ != nullptr) xQueueSend(testReqQ, &id, 0);
      Serial.printf("[TEST] %s requested\n", which.c_str());
      return;
    }
#endif

    HandCommand c = {};
    c.type = CMD_GESTURE;
    c.finger = 0;
    c.angle = 0;

    if (cmd.startsWith("servo_")) {
      int eqPos = cmd.indexOf('=');
      if (eqPos < 0) return;
      int finger = cmd.substring(6, eqPos).toInt();
      if (finger < 0 || finger > 4) return;

      // 再转成角度：0~180 之外直接夹紧，不让非法值进队列
      int angle = constrain(cmd.substring(eqPos + 1).toInt(), 0, 180);
      c.type = CMD_ANGLE;
      c.finger = (uint8_t)finger;
      c.angle = (uint8_t)angle;
    } else {
      strncpy(c.gesture, cmd.c_str(), sizeof(c.gesture) - 1);
      c.gesture[sizeof(c.gesture) - 1] = '\0';
    }

    c.tRx = micros();          // 解析完成，此刻开始计时（必须写在入队之前）

    if (!enqueueCommand(c)) {
      Serial.println("[QUEUE] enqueue failed");
      return;
    }

    markPending = true;
    MARK_HIGH();               // 标记脚拉高：延迟测量的起点

#if RTOS_DEBUG
    Serial.printf("[BLE] queued type=%d, free heap=%u\n", c.type, (unsigned)ESP.getFreeHeap());
#endif
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    Serial.println("[BLE] 上位机已连接");
  }

  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    needRestartAdv = true;      // 广播由 loopTask 负责重开，不在回调里阻塞
    HandCommand home = {};
    home.type = CMD_HOME;
    home.tRx  = micros();       // 必须赋值：否则 tRx=0，延迟会算成"开机至今"的时长
    enqueueCommand(home);       // 断连回张开位，机械手不会僵在半紧状态

    // 断连保护计时起点：从这里到"五根手指全部静止"用了多久
    settleRefUs = micros();
    settleSeenMoving = true;   // 断连会下发 HOME，一定会有动作（除非本来就是张开位）
    settleKind = SETTLE_DISC;

    Serial.println("[BLE] 上位机已断开");
  }
};

// ========================================================================
//  五、运动任务：整个工程唯一调用 pwm.setPWM() 的地方
// ========================================================================
static void applyCommand(const HandCommand& c) {
  switch (c.type) {
    case CMD_GESTURE:
      if (c.gesture[0] != '\0') {
        Serial.printf("[MOTION] gesture=%s\n", c.gesture);
        HandMovementFactory::applyGesture(&hand, String(c.gesture));
      }
      break;
    case CMD_ANGLE:
      Serial.printf("[MOTION] finger %u -> %u°\n", c.finger, c.angle);
      hand.moveFingerAngle(c.finger, c.angle);
      break;
    case CMD_HOME:
      Serial.println("[MOTION] return home (open)");
      hand.moveToNormalized(0);
      break;
    default:
      break;
  }
}

static void motionTask(void* pvParameters) {
  (void)pvParameters;
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t tick = 0;

  for (;;) {
    // 1) 非阻塞取指令：这一拍没有新指令也要继续推进插值
    HandCommand cmd;
    uint32_t firstRx = 0;        // 本拍第一条指令的接收时刻（取最坏值）
    bool hadCmd = false;
    while (xQueueReceive(cmdQueue, &cmd, 0) == pdTRUE) {
      if (!hadCmd) firstRx = cmd.tRx;
      hadCmd = true;
      applyCommand(cmd);          // 一次清空队列，避免积压导致动作延迟
    }

    // 2) 推进 5 根手指的插值（内部决定要不要写 I2C）
    hand.run();

    // 3) 标记脚收尾 + 延迟统计
    //    统计不看 markPending：测试任务灌进来的指令也要计入延迟样本，
    //    否则 test=stream 的负载延迟就测不到。标记脚只是给示波器看的信号。
    if (hadCmd) {
      if (markPending) {
        MARK_LOW();
        markPending = false;
      }
#if RTOS_DEBUG
      // tRx=0 说明这条指令的来源没有打时间戳（例如未来新增的入口忘了赋值），
      // 直接跳过统计，否则 micros()-0 会得到"开机至今"，把 latMax/latSum 全带偏。
      if (firstRx != 0) {
        uint32_t dt = (uint32_t)(micros() - firstRx);
        latRecord(dt);
        Serial.printf("[LATENCY] board-side = %lu us\n", (unsigned long)dt);
      }
#endif
    }

    // 4) 沉降检测：先看到"在动"，再等到"全部静止"，两条都满足才打印
    //    只用 nowStill 判定会在测试刚开始、还没动起来时误触发。
    bool nowStill = hand.isStill();
    if (settleKind != SETTLE_NONE) {
      if (!nowStill) {
        settleSeenMoving = true;
      } else if (settleSeenMoving) {
        uint32_t ms = (uint32_t)((micros() - settleRefUs) / 1000);
        const char* tag = (settleKind == SETTLE_DISC) ? "ESTOP" : "SETTLE";
        Serial.printf("[%s] all fingers still after %lu ms\n", tag, (unsigned long)ms);
        settleKind = SETTLE_NONE;
        settleSeenMoving = false;
      }
    }

    // 5) 固定节拍：vTaskDelayUntil 比 vTaskDelay 更准，不会累积漂移
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTION_PERIOD_MS));

#if RTOS_DEBUG
    if (++tick % 500 == 0) {      // 每 5 秒一次
      Serial.printf("[RTOS] motion stack free = %u bytes, allStill=%d\n",
                    (unsigned)uxTaskGetStackHighWaterMark(NULL),
                    hand.isStill() ? 1 : 0);
    }
#else
    (void)tick;
#endif
  }
}

// ========================================================================
//  五点五、D2 实验：由固件自己产生指令流
// ------------------------------------------------------------------------
//  为什么不让上位机连着发？
//    上位机的发送节奏会被 BLE 连接间隔打散（见 docs/measure.md 的相位分析），
//    而这三个实验要看的是"队列 + 任务调度"的行为，指令源越干净越好。
//    注意这条路径绕过了 BLE 协议栈，所以延迟口径与 D1 一致（都从指令产生时刻算）。
// ========================================================================
static void latReset() {
  latN = 0; latMin = 0xFFFFFFFFu; latMax = 0; latSum = 0;
}

static void pushAngle(uint8_t finger, uint8_t angle) {
  HandCommand c = {};
  c.type   = CMD_ANGLE;
  c.finger = finger;
  c.angle  = angle;
  c.tRx    = micros();
  enqueueCommand(c);
}

static void pushGesture(const char* name) {
  HandCommand c = {};
  c.type = CMD_GESTURE;
  strncpy(c.gesture, name, sizeof(c.gesture) - 1);
  c.tRx = micros();
  enqueueCommand(c);
}

// ---------- D3 自测：16 条用例，覆盖正常帧和全部 7 种错误码 ----------
// 固件自带的单元测试：不依赖上位机、不依赖硬件，烧进去发 test=proto 就跑。
// 协议解析这种"纯逻辑"最适合这样测 —— 改一行校验逻辑，跑一次就知道有没有破。
struct ProtoCase {
  const char* frame;
  ProtoErr    expect;
  int8_t      finger;     // -1 = 不校验
  int16_t     angle;      // -1 = 不校验
  const char* gesture;    // nullptr = 不校验
};

static const ProtoCase PROTO_CASES[] = {
  // ---- 应当通过 ----
  { "$S1,120*7D",  PROTO_OK,          1, 120, nullptr },
  { "$S1,90*47",   PROTO_OK,          1,  90, nullptr },
  { "$s1,120*5D",  PROTO_OK,          1, 120, nullptr },   // 指令字小写
  { "$S1,120*7d",  PROTO_OK,          1, 120, nullptr },   // 校验位小写
  { "$GFIST*4F",   PROTO_OK,         -1,  -1, "fist"  },
  { "$GFIVE*5B",   PROTO_OK,         -1,  -1, "five"  },
  // ---- 应当拒收 ----
  { "S1,120*7D",   PROTO_ERR_START,  -1,  -1, nullptr },   // 缺 $
  { "$S1,120",     PROTO_ERR_NO_STAR,-1,  -1, nullptr },   // 缺 *
  { "$S1,120*7",   PROTO_ERR_CHK_LEN,-1,  -1, nullptr },   // 校验位只有 1 位
  { "$S1,120*ZZ",  PROTO_ERR_CHK_LEN,-1,  -1, nullptr },   // 校验位不是十六进制
  { "$S1,120*00",  PROTO_ERR_CHK,    -1,  -1, nullptr },   // 校验值错
  { "$S9,120*75",  PROTO_ERR_RANGE,  -1,  -1, nullptr },   // 手指号越界
  { "$S1,200*7C",  PROTO_ERR_RANGE,  -1,  -1, nullptr },   // 角度越界
  { "$S1*62",      PROTO_ERR_FORMAT, -1,  -1, nullptr },   // 缺逗号和角度
  { "$G*47",       PROTO_ERR_FORMAT, -1,  -1, nullptr },   // 手势名为空
  { "$X1,120*76",  PROTO_ERR_CMD,    -1,  -1, nullptr },   // 未知指令字
};

static void runProtoTest() {
  const int n = (int)(sizeof(PROTO_CASES) / sizeof(PROTO_CASES[0]));
  int pass = 0;
  Serial.printf("[PROTO-TEST] start, %d cases\n", n);

  for (int i = 0; i < n; i++) {
    const ProtoCase& tc = PROTO_CASES[i];
    HandCommand c = {};
    ProtoErr got = parseFrame(tc.frame, strlen(tc.frame), c, nullptr);

    bool ok = (got == tc.expect);
    // 只比对错误码还不够：解析成功但字段错了，同样算失败
    if (ok && got == PROTO_OK) {
      if (tc.finger  >= 0 && c.finger != (uint8_t)tc.finger)  ok = false;
      if (tc.angle   >= 0 && c.angle  != (uint8_t)tc.angle)   ok = false;
      if (tc.gesture && strcmp(c.gesture, tc.gesture) != 0)   ok = false;
    }

    char detail[24] = "";
    if (ok && got == PROTO_OK) {
      if (c.type == CMD_ANGLE) snprintf(detail, sizeof(detail), "f%u/a%u", c.finger, c.angle);
      else                     snprintf(detail, sizeof(detail), "g=%s", c.gesture);
    }

    Serial.printf("[PROTO-TEST] %2d %-14s expect=%-12s got=%-12s %s %s\n",
                  i + 1, tc.frame,
                  PROTO_ERR_NAME[tc.expect], PROTO_ERR_NAME[got],
                  ok ? "PASS" : "FAIL", detail);
    if (ok) pass++;
  }

  Serial.printf("[PROTO-TEST] %d/%d passed\n", pass, n);

  // 真实流量统计（自测只调 parseFrame，不经过 onWrite，所以这里报的是之前累计值）
  Serial.printf("[PROTO] live: ok=%lu", (unsigned long)protoOk);
  for (int i = 1; i < PROTO_ERR_COUNT; i++) {
    if (protoErr[i]) Serial.printf(", %s=%lu", PROTO_ERR_NAME[i], (unsigned long)protoErr[i]);
  }
  Serial.println();
}

static void testTask(void* pvParameters) {
  (void)pvParameters;
  uint8_t id = 0;

  for (;;) {
    if (xQueueReceive(testReqQ, &id, portMAX_DELAY) != pdTRUE) continue;

    // 每个实验开始前清空，避免上一次的残留影响结论
    xQueueReset(cmdQueue);
    latReset();
    qDrop = 0;
    qDepthMax = 0;

    if (id == 1) {
      // ---- flood：几百微秒内灌进 100 条，队列只有 8 深 ----
      Serial.println("[TEST] flood start: 100 cmds as fast as possible");
      settleRefUs = micros();
      settleSeenMoving = false;
      settleKind = SETTLE_FLOOD;

      uint32_t t0 = micros();
      for (int i = 0; i < 100; i++) {
        pushAngle(1, (uint8_t)(i * 180 / 99));
      }
      uint32_t dt = micros() - t0;
      Serial.printf("[TEST] flood: sent=100 in %lu us, dropped=%lu, max_depth=%lu\n",
                    (unsigned long)dt, (unsigned long)qDrop, (unsigned long)qDepthMax);
      Serial.println("[TEST] policy=drop-oldest, 最终目标应等于第 100 条 = 180 deg");
    } else if (id == 2) {
      // ---- stream：20Hz 连续给目标，模拟上位机拖动滑块 ----
      Serial.println("[TEST] stream start: 20Hz x 60 (3s), target ramps 30->148");
      settleRefUs = micros();
      settleSeenMoving = false;
      settleKind = SETTLE_STREAM;

      for (int i = 0; i < 60; i++) {
        if (!bleConnected) { Serial.println("[TEST] aborted: BLE lost"); break; }
        pushAngle(1, (uint8_t)(30 + i * 2));
        vTaskDelay(pdMS_TO_TICKS(50));
      }
      Serial.printf("[TEST] stream done: dropped=%lu, max_depth=%lu\n",
                    (unsigned long)qDrop, (unsigned long)qDepthMax);
    } else if (id == 3) {
      // ---- estop：持续扫动，等你手动断开蓝牙 ----
      Serial.println("[TEST] estop start: sweeping. 现在手动断开上位机");
      int k = 0;
      while (bleConnected) {
        pushGesture((k++ % 2) ? "fist" : "five");
        vTaskDelay(pdMS_TO_TICKS(200));
      }
      Serial.println("[TEST] estop: BLE lost, waiting for settle ...");
    } else if (id == 4) {
      // ---- proto：协议解析自测，不碰硬件 ----
      runProtoTest();
    }
  }
}

// ========================================================================
//  六、初始化
// ========================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== xiaochu FreeRTOS Version ===");

  // 标记脚：默认低电平，等待第一条 BLE 指令
  pinMode(MARK_PIN, OUTPUT);
  digitalWrite(MARK_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Wire.beginTransmission(PCA9685_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.printf("PCA9685 found at 0x%02X\n", PCA9685_ADDR);
  } else {
    Serial.println("ERROR: PCA9685 not found! 蓝牙仍会启动，请检查 SDA/SCL/共地/地址跳线");
  }

  // 初始归零（张开），此时任务还没起来，独占 I2C 是安全的
  hand.moveToNormalized(0);
  hand.run();

  // 队列必须在 BLE 启动之前建好，否则 onWrite 可能拿到空指针
  cmdQueue = xQueueCreate(CMD_QUEUE_LEN, sizeof(HandCommand));
  if (cmdQueue == nullptr) {
    Serial.println("FATAL: queue create failed");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  BaseType_t ok = xTaskCreate(
      motionTask,
      "motion",
      MOTION_STACK,
      nullptr,
      MOTION_PRIORITY,
      nullptr);
  if (ok != pdPASS) {
    Serial.println("FATAL: motion task create failed");
  } else {
    Serial.printf("motion task started, period=%dms, stack=%d bytes\n",
                  MOTION_PERIOD_MS, MOTION_STACK);
  }

#if TEST_ENABLE
  testReqQ = xQueueCreate(1, sizeof(uint8_t));
  if (testReqQ == nullptr ||
      xTaskCreate(testTask, "test", 4096, nullptr, 2, nullptr) != pdPASS) {
    Serial.println("WARN: test task create failed, D2 experiments disabled");
  } else {
    Serial.println("test task ready: send test=flood | test=stream | test=estop");
  }
#endif

  BLEDevice::init("xiaochu");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService* pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic* pWriteChar = pService->createCharacteristic(
      WRITE_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pWriteChar->setCallbacks(new MyCallbacks());
  pWriteChar->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.printf("BLE ready. pulse range: %d~%d, step=%.1f\n",
                SERVO_MIN_PULSE, SERVO_MAX_PULSE, DEFAULT_STEP);
}

// ========================================================================
//  七、loop() 只剩看护工作
// ========================================================================
void loop() {
  if (needRestartAdv) {
    vTaskDelay(pdMS_TO_TICKS(300));   // 等协议栈释放旧连接
    BLEDevice::getAdvertising()->start();
    needRestartAdv = false;
    Serial.println("[BLE] 已重新开始广播");
  }
  vTaskDelay(pdMS_TO_TICKS(100));
}

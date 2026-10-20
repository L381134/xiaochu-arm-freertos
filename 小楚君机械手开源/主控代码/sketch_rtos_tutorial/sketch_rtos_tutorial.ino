// ========================================================================
//  xiaochu 机械手固件 - ESP32-C3 + PCA9685
//  可以在下面调整 SERVO_MIN_PULSE 和 SERVO_MAX_PULSE 来适配你的舵机
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
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ★★★ 在这里校准你的舵机脉宽范围（建议先使用 100~500，然后微调） ★★★
#define SERVO_MIN_PULSE 150   // 对应 0°（初始建议值，需要实测）
#define SERVO_MAX_PULSE 500   // 对应 180°（初始建议值，需要实测）

#define SERVO_FREQ 50         // 50Hz

// ---------- BLE UUID（与上位机一致）----------
#define SERVICE_UUID           "89d60870-9908-4472-8f8c-e5b3e6573cd1"
#define WRITE_CHAR_UUID        "39dea685-a63e-44b2-8819-9a202581f8fe"

// ---------- 手指定义（同前）----------
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

//#define DEFAULT_STEP 1.0f
#define DEFAULT_STEP 2.0f   // 速度翻倍
// ---------- Finger 类（同前）----------
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

  void run() {
    if (abs(currentPosition - target) <= step) {
      currentPosition = target;
    } else if (currentPosition < target) {
      currentPosition += step;
    } else {
      currentPosition -= step;
    }
    int pulse = map(currentPosition, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(channel, 0, pulse);
  }

  bool isStill() { return abs(currentPosition - target) <= step; }
  void setStep(float s) { step = s; }
};

// ---------- Hand 类（同前）----------
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
  void run() { for (int i=0;i<5;i++) fingers[i]->run(); }
  bool isStill() { for (int i=0;i<5;i++) if (!fingers[i]->isStill()) return false; return true; }
  void setStep(float s) { for (int i=0;i<5;i++) fingers[i]->setStep(s); }
};

// ---------- 手势工厂（同前）----------
class HandMovementFactory {
public:
  static void applyGesture(Hand* hand, const String& name) {
    if (name == "fist") {
      hand->moveToNormalized(100);
    } else if (name == "five") {
      hand->fingers[0]->setTargetNormalized(0);
      hand->fingers[1]->setTargetNormalized(0);
      hand->fingers[2]->setTargetNormalized(0);
      hand->fingers[3]->setTargetNormalized(0);
      hand->fingers[4]->setTargetNormalized(0);
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

// ---------- 全局对象 ----------
Hand hand;
BLECharacteristic* pWriteChar = nullptr;

// ---------- 跨任务传递的指令包（定长，不用 String）----------
enum CmdType : uint8_t {
  CMD_GESTURE = 0,   // 手势名
  CMD_ANGLE   = 1,   // 单指角度 servo_n=angle
  CMD_HOME    = 2    // 回张开位（断连时用）
};

struct HandCommand {
  CmdType type;
  uint8_t finger;
  uint8_t angle;
  char    gesture[16];   // 手势名最长 7 个字符，16 足够
};

// 队列句柄，相当于"传菜口"的门牌号
QueueHandle_t cmdQueue = nullptr;

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    auto value = pCharacteristic->getValue();
    if (value.length() == 0) return;
    String cmd = String(value.c_str());
    cmd.trim();
    Serial.print("Received: ");
    Serial.println(cmd);

    if (cmd.startsWith("servo_")) {
      int eqPos = cmd.indexOf('=');
      if (eqPos != -1) {
        String idxStr = cmd.substring(6, eqPos);
        int finger = idxStr.toInt();
        if (finger >= 0 && finger < 5) {
          String angleStr = cmd.substring(eqPos + 1);
          int angle = angleStr.toInt();
          hand.moveFingerAngle(finger, angle);
          Serial.printf("Finger %d set to %d°\n", finger, angle);
        }
      }
      return;
    }

    HandMovementFactory::applyGesture(&hand, cmd);
  }
};

// ---------- 连接状态回调（断线后自动重开广播）----------
// 没有这个回调，断开一次后广播就永远停着，表现成"再也连不上，只能断电重启"
volatile bool needRestartAdv = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("[BLE] 上位机已连接");
  }

  void onDisconnect(BLEServer* pServer) override {
    Serial.println("[BLE] 上位机已断开");
    needRestartAdv = true;   // 不在回调里直接阻塞，交给 loop() 处理
  }
};

// ======================== 运动任务（第 2 关：先放空壳）========================
static void motionTask(void* pv) {
  (void)pv;                                   // 没用到的参数，显式忽略
  TickType_t lastWake = xTaskGetTickCount();  // 记录"下一次该醒的时刻"

  for (;;) {
    // 【第 3 关】这里会加上 hand.run()
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
  }
}

// ======================== setup ========================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== xiaochu Calibratable Version ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Wire.beginTransmission(0x40);
  if (Wire.endTransmission() == 0) {
    Serial.println("PCA9685 found at 0x40");
  } else {
    // 原来这里是 while(1) 死循环，会让蓝牙根本来不及启动，
    // 表现成"完全搜不到设备"，很难排查。改成只报警不卡死。
    Serial.println("ERROR: PCA9685 not found at 0x40! 蓝牙仍会启动，请检查 SDA/SCL/共地/地址跳线");
  }

  // 初始归零（打开）
  for (int i = 0; i < 5; i++) {
    hand.fingers[i]->setTargetNormalized(0);
  }
  for (int i = 0; i < 5; i++) {
    int pulse = map(0, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(i, 0, pulse);
  }
  Serial.println("All servos set to open position");

  // ---- FreeRTOS：顺序必须是 队列 → 任务 → BLE ----
  cmdQueue = xQueueCreate(8, sizeof(HandCommand));
  if (cmdQueue == nullptr) {
    Serial.println("FATAL: 队列创建失败（内存不够）");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  xTaskCreate(motionTask, "motion", 4096, nullptr, 4, nullptr);
  Serial.println("motion task started, period=10ms");

  BLEDevice::init("xiaochu");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());   // ★ 关键：注册连接/断开监听
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pWriteChar = pService->createCharacteristic(
    WRITE_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pWriteChar->setCallbacks(new MyCallbacks());
  pWriteChar->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);   // ★ 同时回扫描响应包，提高被发现概率
  pAdvertising->start();

  Serial.println("BLE ready, waiting for connection...");
  Serial.printf("Current pulse range: min=%d, max=%d\n", SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

// ======================== loop ========================
void loop() {
  hand.run();

  // ★ 断开后必须重新开广播，否则设备就"隐身"了，只能断电重启
  if (needRestartAdv) {
    delay(300);                             // 等协议栈把旧连接彻底释放，太短会启动失败
    BLEDevice::getAdvertising()->start();
    needRestartAdv = false;
    Serial.println("[BLE] 已重新开始广播，可以再次连接");
  }

  delay(10);
}
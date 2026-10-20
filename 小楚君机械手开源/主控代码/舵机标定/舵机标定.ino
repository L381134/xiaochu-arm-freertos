// ========================================================================
//  舵机脉宽标定工具  -  ESP32-C3 + PCA9685 + SG90
// ------------------------------------------------------------------------
//  用途：
//    你主程序里的 SERVO_MIN_PULSE=150 / SERVO_MAX_PULSE=500 只是"初始猜测值"。
//    这个工具帮你找出【你这几个舵机在你这套机构上】真实的脉宽范围，
//    找到之后填回主程序的这两个宏，再重烧主程序即可。
//
//  用法：
//    1. 先只接 1 个舵机（接到 PCA9685 的 通道0），不要把 5 个全接上。
//    2. 烧录本程序，打开串口监视器，波特率 115200，行尾选 "换行(NL)"。
//    3. 按下面提示敲命令，观察舵机转到哪。
//
//  命令表：
//    300      把当前通道脉宽设为 300
//    +20      在当前脉宽上加 20
//    -20      在当前脉宽上减 20
//    ch 2     切换当前通道到 2（0~4）
//    sweep    从 100 到 600 慢慢扫一遍，直接看全程行程
//    help     再打印一次帮助
// ========================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---------- I2C 引脚（与主程序一致）----------
#define SDA_PIN 8
#define SCL_PIN 9

// ---------- PCA9685 ----------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
// 如果串口报 "找不到 PCA9685"，把上面的 0x40 改成 0x70 再试
#define SERVO_FREQ 50

int currentChannel = 0;   // 当前在调哪个通道
int currentPulse   = 300; // 当前脉宽（PCA9685 计数单位）

void printHelp() {
  Serial.println();
  Serial.println("-------------------- 当前状态 --------------------");
  Serial.print("  当前通道 = "); Serial.print(currentChannel);
  Serial.print("   当前脉宽 = "); Serial.println(currentPulse);
  Serial.println("--------------------------------------------------");
  Serial.println("  300     把当前通道脉宽设为 300");
  Serial.println("  +20     脉宽 +20");
  Serial.println("  -20     脉宽 -20");
  Serial.println("  ch 2    切换到通道 2");
  Serial.println("  sweep   100->600 慢速扫描一遍");
  Serial.println("  help    显示本帮助");
  Serial.println();
  Serial.println("【怎么判断极限】脉宽一点点加大，舵机跟着转；");
  Serial.println("  直到加下去舵机不再动 / 发出顶死的嗡嗡声，");
  Serial.println("  那个值再往回退 20~30，就是安全端点。");
  Serial.println();
}

void setPulse(int ch, int pulse) {
  if (pulse < 0)   pulse = 0;
  if (pulse > 1000) pulse = 1000;
  currentPulse = pulse;
  pwm.setPWM(ch, 0, pulse);
  Serial.print("  -> ch"); Serial.print(ch);
  Serial.print("  pulse = "); Serial.println(pulse);
}

void sweep(int ch) {
  Serial.println("开始扫描 100 -> 600（每个值停 250ms）...");
  for (int p = 100; p <= 600; p += 10) {
    pwm.setPWM(ch, 0, p);
    Serial.print(p); Serial.print(" ");
    delay(250);
  }
  Serial.println("\n扫描结束。请把舵机停住 / 卡死的两个临界值记下来。");
  currentPulse = 300;
  pwm.setPWM(ch, 0, currentPulse);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== 舵机脉宽标定工具 ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // I2C 地址探测
  Wire.beginTransmission(0x40);
  if (Wire.endTransmission() == 0) {
    Serial.println("PCA9685 found at 0x40  (OK)");
  } else {
    Serial.println("ERROR: 找不到 PCA9685!");
    Serial.println("  检查: 1) SDA=GPIO8  SCL=GPIO9  有没有接反");
    Serial.println("        2) 驱动板 VCC 是否供电、GND 是否与 ESP32 共地");
    Serial.println("        3) 地址若不是 0x40，把代码里两处 0x40 改成 0x70");
    while (1) delay(1000);
  }

  // 上电先给一个中间值，避免舵机突然飞向一端
  pwm.setPWM(currentChannel, 0, currentPulse);
  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  Serial.print("命令: "); Serial.println(line);

  if (line == "help") {
    printHelp();
  } else if (line == "sweep") {
    sweep(currentChannel);
  } else if (line.startsWith("ch")) {
    int ch = line.substring(2).toInt();
    if (ch >= 0 && ch < 16) {
      currentChannel = ch;
      setPulse(currentChannel, currentPulse); // 新通道先给到当前脉宽
      Serial.print("  已切换到通道 "); Serial.println(currentChannel);
      Serial.println("  提示：请确认只剩这一个舵机在通电，其余建议先不接。");
    } else {
      Serial.println("  通道号无效（0~15）");
    }
  } else if (line.startsWith("+")) {
    setPulse(currentChannel, currentPulse + line.substring(1).toInt());
  } else if (line.startsWith("-")) {
    setPulse(currentChannel, currentPulse - line.substring(1).toInt());
  } else {
    int p = line.toInt();
    if (p > 0) {
      setPulse(currentChannel, p);
    } else {
      Serial.println("  无法识别，输入 help 看命令表");
    }
  }
}

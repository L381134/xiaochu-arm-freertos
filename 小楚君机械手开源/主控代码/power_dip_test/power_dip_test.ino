/*
 * 电源跌落测量 —— 用 ESP32 自己的 ADC 当"慢速示波器"
 *
 * 用途：
 *   五路舵机同时启动时，5V 轨会被瞬间拉低。没有示波器时，
 *   用分压电阻把 5V 轨引到 ADC 引脚，以几千 Hz 的速度连续采样，
 *   记录最低电压和低于阈值的持续时间。
 *
 * 接线（务必确认，接错会烧引脚）：
 *
 *   5V 轨 ──[ R_TOP ]──┬──[ R_BOTTOM ]── GND
 *                      │
 *                      └── GPIO3 (ADC_PIN)
 *
 *   1) 分压是必须的：ESP32-C3 的 ADC 输入不得超过 3.3V。
 *      两个 10kΩ 等值电阻分压后，5V 变 2.5V，安全。
 *   2) 5V 轨可以是舵机电源，也可以是开发板 5V 引脚，取决于你要查哪一段。
 *   3) **ESP32 的 GND 必须和舵机电源的 GND 接在一起**，否则读数无意义。
 *   4) 电阻不必是 10k，等值即可；比值改了请同步修改 R_TOP_K / R_BOTTOM_K。
 *
 * 输出示例：
 *   [BASE] 5.02 V
 *   [STRESS] min 3.41 V  max 5.11 V
 *   [DIP] 低于 4.51 V 的样本 1203 个 / 总 48213 个，约 2.5%
 *   [AFTER] 5.01 V
 *
 * 注意：ESP32 的 ADC 绝对精度约 ±30~50 mV，但本次要的是"掉了多少"这个相对量，
 *      精度足够。若读数整体偏移，先用万用表核对一次分压比。
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================== 需要你确认的配置 ====================
#define ADC_PIN        3        // 分压中点接到哪个引脚（GPIO3 = ADC1_CH3）
#define R_TOP_K        10.0     // 上臂电阻（接 5V 那一侧），单位 kΩ
#define R_BOTTOM_K     10.0     // 下臂电阻（接 GND 那一侧），单位 kΩ
// =========================================================

#define PCA9685_ADDR   0x40
#define SDA_PIN        8
#define SCL_PIN        9

#define N_CH           5
#define PULSE_MIN      150      // 0°  对应的计数值（与主固件一致，按实物标定）
#define PULSE_MAX      500      // 180°对应的计数值

#define SWEEP_REPS     5        // 来回扫动次数
#define STEP_DEG       6        // 每步转 6°，共 30 步走完 180°
#define SAMPLES_PER_STEP 40     // 每一步采样多少次
#define HEARTBEAT_MS   500

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

float   g_ratio = 1.0f;         // (Rtop + Rbot) / Rbot
uint32_t g_threshMv = 0;        // 低于这个值算一次跌落（基线的 90%）

uint32_t g_samples = 0;
uint32_t g_minMv = 0xFFFFFFFF;
uint32_t g_maxMv = 0;
uint32_t g_dipCount = 0;
uint32_t g_lastBeat = 0;

// ---------- 工具 ----------
uint32_t readRailMv() {
  uint32_t mvAtPin = analogReadMilliVolts(ADC_PIN);
  return (uint32_t)(mvAtPin * g_ratio + 0.5f);
}

int degToPulse(int deg) {
  return PULSE_MIN + (int)((PULSE_MAX - PULSE_MIN) * (long)deg / 180);
}

void setAllServos(int deg) {
  uint16_t off = (uint16_t)degToPulse(deg);
  for (int ch = 0; ch < N_CH; ch++) {
    pwm.setPWM(ch, 0, off);
  }
}

void sampleRail() {
  uint32_t mv = readRailMv();
  g_samples++;
  if (mv < g_minMv) g_minMv = mv;
  if (mv > g_maxMv) g_maxMv = mv;
  if (mv < g_threshMv) g_dipCount++;
}

void heartbeat() {
  if (millis() - g_lastBeat < HEARTBEAT_MS) return;
  g_lastBeat = millis();
  Serial.printf("[HB] n=%lu  now=%lu mV  min=%lu mV\n",
                (unsigned long)g_samples,
                (unsigned long)readRailMv(),
                (unsigned long)g_minMv);
}

// ---------- 主流程 ----------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Power Rail Monitor ===");

  g_ratio = (R_TOP_K + R_BOTTOM_K) / R_BOTTOM_K;
  Serial.printf("divider: %.0fk / %.0fk  ratio = %.3f\n", R_TOP_K, R_BOTTOM_K, g_ratio);

  analogSetAttenuation(ADC_11db);
  pinMode(ADC_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(100);

  // 基线：所有舵机静止
  setAllServos(90);
  delay(600);

  uint32_t sum = 0;
  const int N = 300;
  for (int i = 0; i < N; i++) {
    sum += readRailMv();
    delay(2);
  }
  uint32_t base = sum / N;
  g_threshMv = base * 90 / 100;      // 低于基线 10% 视为跌落
  Serial.printf("[BASE] %lu.%02lu V    threshold = %lu mV\n",
                (unsigned long)(base / 1000), (unsigned long)((base % 1000) / 10),
                (unsigned long)g_threshMv);

  Serial.printf("[RUN] %d reps x 2 sweeps, %d samples/step ...\n", SWEEP_REPS, SAMPLES_PER_STEP);
  uint32_t tStart = millis();

  for (int rep = 0; rep < SWEEP_REPS; rep++) {
    for (int deg = 0; deg <= 180; deg += STEP_DEG) {
      setAllServos(deg);
      for (int k = 0; k < SAMPLES_PER_STEP; k++) { sampleRail(); heartbeat(); }
    }
    for (int deg = 180; deg >= 0; deg -= STEP_DEG) {
      setAllServos(deg);
      for (int k = 0; k < SAMPLES_PER_STEP; k++) { sampleRail(); heartbeat(); }
    }
  }

  uint32_t elapsed = millis() - tStart;

  setAllServos(90);
  delay(500);

  uint32_t after = readRailMv();

  Serial.println("---- RESULT ----");
  Serial.printf("[STRESS] min %lu.%02lu V   max %lu.%02lu V\n",
                (unsigned long)(g_minMv / 1000), (unsigned long)((g_minMv % 1000) / 10),
                (unsigned long)(g_maxMv / 1000), (unsigned long)((g_maxMv % 1000) / 10));
  Serial.printf("[DIP] below %lu mV: %lu / %lu samples (%.1f%%), in %lu ms\n",
                (unsigned long)g_threshMv,
                (unsigned long)g_dipCount, (unsigned long)g_samples,
                100.0f * g_dipCount / (g_samples ? g_samples : 1),
                (unsigned long)elapsed);
  Serial.printf("[AFTER] %lu.%02lu V\n",
                (unsigned long)(after / 1000), (unsigned long)((after % 1000) / 10));

  uint32_t drop = base > g_minMv ? base - g_minMv : 0;
  Serial.printf("[DROP] %lu mV (%.1f%% of baseline)\n",
                (unsigned long)drop, 100.0f * drop / (base ? base : 1));

  if (g_minMv < 3000) {
    Serial.println("[WARN] below 3.0 V — ESP32 brownout risk, power supply is insufficient");
  } else if (drop > 500) {
    Serial.println("[WARN] drop > 0.5 V — wiring/reservoir cap needs improvement");
  } else {
    Serial.println("[OK] rail is stable");
  }
  Serial.println("=================");
}

void loop() {
  // 一次性测试，loop 留空
}

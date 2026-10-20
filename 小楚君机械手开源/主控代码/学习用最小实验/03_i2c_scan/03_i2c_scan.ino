// ========================================================================
//  实验 03：I2C 扫描器（20 秒定位 PCA9685 到底在不在总线上）
// ------------------------------------------------------------------------
//  什么时候用：
//    串口出现 "ERROR: PCA9685 not found at 0x40!"
//
//  它做什么：
//    往 I2C 总线上的每个地址问一句"有人吗"，谁应答就打印谁。
//    PCA9685 出厂默认地址是 0x40，扫到 0x40 才说明接线和供电都没问题。
//
//  怎么读结果：
//    扫到 0x40          → 线没问题，问题在别处（比如舵机电源/OE）
//    一个都没扫到        → SDA/SCL 接反、没共地、VCC 没供电、线断了
//    扫到 0x40 以外的地址 → 板上 A0~A5 跳线改了，把那个地址填回代码
//
//  注意：本项目一根舵机线都不用接，只需要 ESP32-C3 ↔ PCA9685 那 4 根：
//        3.3V → VCC    GND → GND    GPIO8 → SDA    GPIO9 → SCL
//        （OE 接地、V+ 接外部 5V 这两项不影响 I2C 扫描，可以先不管）
// ========================================================================

#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== I2C 扫描开始 ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);        // 先用 100kHz 慢速，线长/接触不良时更宽容

  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {          // 返回 0 = 对方应答了
      Serial.printf("发现设备: 0x%02X", addr);
      if (addr == 0x40) Serial.print("   <- 这就是 PCA9685，接线没问题");
      Serial.println();
      found++;
    }
  }

  Serial.println("=== 扫描结束 ===");
  if (found == 0) {
    Serial.println("一个设备都没有。按这个顺序查：");
    Serial.println("  1. 有没有共地：ESP32 GND / PCA9685 GND / 5V 电源负极 三处连一起");
    Serial.println("  2. SDA、SCL 是不是接反了（GPIO8=SDA，GPIO9=SCL）");
    Serial.println("  3. PCA9685 的 VCC 有没有接 3.3V（别接 5V，会烧主控 IO）");
    Serial.println("  4. 换两根杜邦线，面包板换一排孔试");
  } else {
    Serial.printf("共找到 %d 个设备\n", found);
  }
}

void loop() {
  delay(5000);
}

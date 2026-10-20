# 机械手固件 FreeRTOS 改造指南

> 目标文件：`小楚君机械手开源/主控代码/sketch_26_7_4_rtos/sketch_26_7_4_rtos.ino`（已写好，可直接编译）
> 原版保留在 `sketch_26_7_4/`，不要覆盖，方便对比和回滚。

---

## 一、先纠正一个认知：你其实早就在用 FreeRTOS

Arduino-ESP32 的 `setup()` / `loop()` 跑在一个叫 **loopTask** 的 FreeRTOS 任务里，`delay()` 内部就是 `vTaskDelay()`。所以：

- `delay(10)` **不会**占满 CPU，它只是让 loopTask 睡 10ms，其他任务照常跑；
- 你的 BLE 收包一直很及时，因为协议栈跑在**另一个任务**里，优先级比 loopTask 高。

**结论：这个项目现在并没有"性能瓶颈"，不改也能用。** 那为什么还要改？三个真实理由：

| 理由 | 具体说明 |
|---|---|
| **为扩展做准备** | 你下一步要加 MPU6050 读取、串口调试命令、状态上报。这些东西塞进同一个 `loop()`，谁耗时长就拖慢谁，最后变成一堆 `if (millis() - last > ...)` 的时间片轮询 |
| **节拍确定性** | `delay(10)` 的实际周期 = 10ms + 其他代码耗时，代码越多越慢。`vTaskDelayUntil` 保证固定 10ms，动作速度不会随代码量漂移 |
| **简历可讲** | "单 loop 结构 → 三任务 + 队列" 是嵌入式岗面试的常见加分项，比背概念强得多。但要**诚实**，别说"因为性能瓶颈引入 RTOS"，会被追问到底 |

---

## 二、改造后的结构

```
        ┌──────────────────────────────┐
        │  BT task（协议栈自带，优先级高）│
        │  onWrite: 只解析字符串 + 入队  │  ← 微秒级，不做任何 I2C
        └───────────────┬──────────────┘
                        │ HandCommand（定长结构体，按值拷贝）
                        ▼
                 ┌─────────────┐
                 │  cmdQueue   │  深度 8，满了丢最旧
                 └──────┬──────┘
                        │ 每 10ms 非阻塞取空
                        ▼
        ┌──────────────────────────────┐
        │  motionTask（优先级 4）        │
        │  出队 → 改目标值 → 插值 → 写PWM│  ← 全工程唯一调用 setPWM 的任务
        └───────────────┬──────────────┘
                        │ I2C（GPIO8/9）
                        ▼
                    PCA9685 → 5×SG90

        ┌──────────────────────────────┐
        │  loopTask（优先级 1）          │
        │  只负责断连后重开广播          │
        └──────────────────────────────┘
```

**一句话记住这个设计**：BLE 回调只"递纸条"，运动任务负责"干活"，两者不共享任何可变状态。

---

## 三、三个关键设计（面试就问这三点）

### 1. 用队列传指令，而不是用全局变量

原版是在 BLE 回调里直接 `hand.moveFingerAngle(...)` 改目标值，虽然能跑（因为只有 loop 里写 PWM），但一旦有第二个任务要读状态，就得加锁。

改后：`HandCommand` 结构体按值进队列，**只有 motionTask 碰 `hand` 对象**，所以一把锁都不需要。

```cpp
struct HandCommand {
  CmdType type;      // 手势 / 单指角度 / 回原点
  uint8_t finger;
  uint8_t angle;
  char    gesture[16];   // 定长，不用 String
};
```

**为什么不用 `String` 跨任务传？** `String` 会 `new`/`delete` 堆内存，在 BLE 任务里分配、在运动任务里释放，长时间跑会产生堆碎片。定长结构体走的是任务队列的拷贝，零分配。

### 2. 队列满了怎么办：丢最旧的

控制类指令的语义是"最新值有效"，不是"每条都要执行"。所以队列满时丢掉队首再塞新的：

```cpp
if (xQueueSend(cmdQueue, &cmd, 0) == pdTRUE) return true;
HandCommand dropped;
xQueueReceive(cmdQueue, &dropped, 0);   // 扔最旧
return xQueueSend(cmdQueue, &cmd, 0) == pdTRUE;
```

> 进阶做法：队列长度设 1，直接用 `xQueueOverwrite()` —— 天然只保留最后一条。但深度 8 + 丢最旧更适合"手势序列"场景，先用这个。

### 3. 固定节拍用 vTaskDelayUntil，不用 vTaskDelay

```cpp
TickType_t lastWake = xTaskGetTickCount();
for (;;) {
  ...
  vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));   // 按绝对时间唤醒
}
```

`vTaskDelay(10)` 是"从**现在**再睡 10ms"，每拍都会把执行耗时算进去，累积漂移；`vTaskDelayUntil` 是"到**第 N+10ms** 叫醒我"，节拍恒定。舵机速度完全取决于这个周期，所以这条很关键。

---

## 四、代码逐段对照

| 位置 | 原版 `sketch_26_7_4.ino` | 改造版 |
|---|---|---|
| 头部 | 无 | 加 `freertos/FreeRTOS.h`、`task.h`、`queue.h` |
| `Finger::run()` (L78) | `void`，每拍无条件 `setPWM` | 返回 `bool`，位置没变就不写 I2C |
| `Hand::run()` (L108) | `void` | 返回"这一拍有没有动过" |
| 全局对象 (L187) | `Hand hand; BLECharacteristic* pWriteChar` | 加 `QueueHandle_t cmdQueue`、`volatile bool bleConnected` |
| `MyCallbacks::onWrite` (L190) | 直接调 `hand.moveFingerAngle()` | 解析 → 填 `HandCommand` → `enqueueCommand()` |
| `onDisconnect` (L228) | 只置 `needRestartAdv` | 额外下发 `CMD_HOME`，断连自动回张开位 |
| `setup()` (L235) | 直接初始化 BLE | 先建队列、再建任务，**最后**启动 BLE（顺序不能反） |
| `loop()` (L268) | `hand.run(); delay(10);` | 只剩广播看护 + `vTaskDelay(100)` |
| 新增 | — | `motionTask()`、`applyCommand()`、`enqueueCommand()` |

**协议、引脚、标定参数一个都没动**，上位机（Kivy 桌面版 / 网页版）不需要任何修改。

---

## 五、烧录与验证（4 步，别跳）

1. **编译烧录**：Arduino IDE 打开 `sketch_26_7_4_rtos.ino`，板子选 `AirM2M_CORE_esp32c3`（和你原来一致），直接上传。
   串口 115200 应看到：
   ```
   === xiaochu FreeRTOS Version ===
   PCA9685 found at 0x40
   motion task started, period=10ms, stack=4096 bytes
   BLE ready. pulse range: 150~500, step=2.0
   ```
2. **功能回归**：连上位机，依次试 fist / five / ok、单指滑块、自定义 `servo_2=90`，动作应与原版一致。
3. **看节拍**：`RTOS_DEBUG 1` 打开时，每 5 秒打印一次 `motion stack free`。
   - 栈剩余 **> 1500 字节** 说明 4096 够用；低于 1000 就调到 6144。
   - 想量化节拍抖动：在 `motionTask` 里每 100 拍打印一次 `micros()` 差值，应稳定在 1,000,000µs 附近（±几 ms）；用 `delay(10)` 的版本会明显偏大。
4. **断连测试**：手机直接关蓝牙（不要点"断开"），观察两件事——机械手 ~1 秒内回到张开位；重新扫描能搜到 `xiaochu` 并连上。

---

## 六、五个必踩的坑

| 坑 | 说明 |
|---|---|
| **栈大小单位是字节** | ESP-IDF 的 `xTaskCreate` 用**字节**，不是字。网上大量教程写 1024 以为是 1KB，其实只给了 1KB，一 `printf` 就溢出重启。给 4096 起步 |
| **ESP32-C3 是单核** | 别抄 `xTaskCreatePinnedToCore(..., 0)`，C3 只有一个核，用 `xTaskCreate` 就行。双核的 ESP32/S3 才需要绑核 |
| **回调里不要做重活** | `onWrite` 跑在协议栈任务里，在里面 `delay`、读 I2C、`printf` 大量数据，会拖慢甚至拖垮 BLE 连接。只做"解析 + 入队" |
| **创建顺序** | 队列必须在 `BLEDevice::init()` 之前建好，否则连上瞬间来的 `onWrite` 会拿到空指针 |
| **初始化也别在任务外碰 I2C** | 任务起来后再从 `setup()` 调 `pwm.setPWM()` 就是两个写者了。归零动作放在任务启动前做完（现在的写法就是） |

---

## 七、写进简历 / 面试怎么说

简历 bullet（**测完再写数字**）：

> - 将固件从单 `loop()` 结构重构为三任务模型：BLE 回调仅做协议解析与入队，运动任务独占 I2C 并按 10ms 固定节拍推进插值，看护任务负责断连后广播重启。
> - 跨任务指令使用定长结构体经 FreeRTOS 队列按值传递，避免 `String` 堆分配造成内存碎片；队列满时淘汰最旧指令，保证控制指令的"最新值生效"语义。
> - 用 `vTaskDelayUntil` 替代 `delay()` 实现无累积漂移的节拍控制，并以栈水位（`uxTaskGetStackHighWaterMark`）验证 4KB 栈配置余量。

面试可能追问的三句：

1. **"为什么用队列不用互斥锁？"** → 因为我让数据单向流动：BLE 任务只写队列不碰对象，`hand` 只有一个写者，天然无竞争，锁是多余的。
2. **"任务优先级怎么定的？"** → 运动任务 4 高于 loopTask 1，因为它有 10ms 硬节拍；但它大部分时间阻塞在 `vTaskDelayUntil`，不抢 CPU。BLE 协议栈优先级由 IDF 固定，不归我管。
3. **"如果指令来得很猛怎么办？"** → 队列会满，所以做了淘汰策略；另外若指令是"连续拖动滑块"，更合适的是深度 1 的 `xQueueOverwrite`，只保留最新值。

---

## 八、下一步可选升级（做完再写进简历）

| 升级 | 收益 | 难度 |
|---|---|---|
| 加 `telemetryTask`：每 200ms 通过 Notify 上报 5 个手指当前角度 | 上位机能显示真实状态，形成闭环 | ★★ |
| 用事件组（EventGroup）管理连接/运动状态，替代 `volatile bool` | 状态多了以后更清晰 | ★★ |
| 换 `NimBLE-Arduino` 替代自带 BLE 库 | 省 30~50KB RAM，给任务栈腾空间 | ★★★（API 要改一批） |
| 加 `ESP32` 的 `esp_timer` 做微秒级时间戳 | 能测真实指令延迟 | ★★★ |

---

## 九、最后一句实话

这次改造**不会让机械手转得更快或更准**，它的价值在"结构"和"可扩展性"。所以别在简历里把它写成性能优化，写成**架构重构**——面试官对"知道什么时候该用 RTOS、也知道现在其实不必用"的候选人，评价会高于"什么都往上堆"的人。

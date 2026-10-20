# 零基础 FreeRTOS 改造 · 六关教程

> 对象：没学过 FreeRTOS，但能看懂 `setup()`/`loop()` 的 C 语言水平。
> 练手文件：`主控代码/sketch_rtos_tutorial/sketch_rtos_tutorial.ino`（原版逐字复制，改坏了不影响项目）
> 参考答案：`主控代码/sketch_26_7_4_rtos/sketch_26_7_4_rtos.ino`（我写好的完整版，卡住了就对照它）
>
> **铁律：每关都要烧录验证通过，再进下一关。** 一口气改完，出错你不知道是哪一步坏的。

---

## 第 0 关：先记住四个词（不写代码）

FreeRTOS 能用的东西有几十个 API，你这个项目只需要四个。先把这张表看懂：

| 词 | 大白话 | 在你的项目里 | 对应函数 |
|---|---|---|---|
| **任务** Task | 一个独立干活的工人，有自己的小仓库（栈）和死循环 | 收蓝牙的、转舵机的、看广播的，各是一个工人 | `xTaskCreate` |
| **阻塞** Block | 工人主动说"我等着，先别给我派活" | `vTaskDelay(10)` 就是等 10ms | `vTaskDelay` / `vTaskDelayUntil` |
| **优先级** | 谁先上工，数字大的优先（0~24） | 转舵机的 4，loop 的 1 | `xTaskCreate` 第 5 个参数 |
| **队列** Queue | 传菜口，窗口固定几个，谁都能往里放，谁都能拿 | 蓝牙把指令放进去，转舵机的来取 | `xQueueCreate/Send/Receive` |

**必须纠正的一个认知**：你的原版代码**没有卡死 CPU**。`delay()` 内部就是 `vTaskDelay()`，它会让出 CPU，所以蓝牙一直很灵敏。原版真正的问题只有两个：

1. 所有事情挤在 `loop()` 里，以后加传感器会互相拖慢；
2. `delay(10)` 的周期 = 10ms + 前面代码的耗时，代码越写越多，机械手会越转越慢。

所以改造不是"修 bug"，是"换结构"。这一点想清楚了，后面每一步才有意义。

---

## 第 1 关：先跑两个玩具实验（30 分钟）

在碰项目之前，先在两个空 sketch 上把概念跑通。**不要跳过**，这两个实验就是你后面要抄的模板。

### 实验 01：两个任务同时跑

打开 `学习用最小实验/01_two_tasks/01_two_tasks.ino`，直接烧录，串口 115200。

你会看到 `[A]`、`[B]`、`[loop]` 交错打印。**为什么看起来像同时？** 因为每个任务在 `vTaskDelay` 时主动让出 CPU，调度器立刻换下一个。C3 只有一个核，同一时刻只有一个任务在跑——这点面试会问。

重点看这一行，六个参数你会用一辈子：

```cpp
xTaskCreate(taskA, "taskA", 4096, nullptr, 2, &handleA);
//         ①函数   ②名字   ③栈字节 ④传参  ⑤优先级 ⑥句柄
```

⚠️ 第 3 个参数单位是**字节**，不是字。写 1024 就真的只有 1KB，一个 `printf` 就爆栈重启。

### 实验 02：队列传值

打开 `学习用最小实验/02_queue_demo/02_queue_demo.ino`，烧录。

现象：生产者 300ms 产一个，消费者 1000ms 吃一个，队列（长度 3）很快满了，开始打印"丢掉最旧的"。

**这个实验演示的正是你要用的策略**：控制指令的正确语义是"最新值有效"，不是"每条都必须执行"。用户拖滑块拖出 20 个中间值，机械手只需要最后那个。

两个实验跑通了再往下。

---

## 第 2 关：只加骨架，不碰逻辑

**目标：加进任务的"空壳"，机械手行为和原来一模一样。**

这是重构的第一原则：**先加新东西，不动老东西，确认没坏再拆老的。**

打开 `sketch_rtos_tutorial.ino`，做 4 处修改。

### 2-A 加三行头文件

找到这一行（原文件第 11 行）：

```cpp
#include <Adafruit_PWMServoDriver.h>
```

在它**下面**插入：

```cpp
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
```

> 这三个头文件是 ESP32 核心自带的，不用装任何库。

### 2-B 定义"指令"长什么样

找到这里（原文件第 185~187 行）：

```cpp
// ---------- 全局对象 ----------
Hand hand;
BLECharacteristic* pWriteChar = nullptr;
```

在 `pWriteChar` 那行**下面**插入：

```cpp
// ------ 跨任务传递的指令包（定长，不用 String）------
enum CmdType : uint8_t {
  CMD_GESTURE = 0,   // 手势名
  CMD_ANGLE   = 1,   // 单指角度 servo_n=angle
  CMD_HOME    = 2    // 回张开位
};

struct HandCommand {
  CmdType type;
  uint8_t finger;
  uint8_t angle;
  char    gesture[16];   // 手势名最长 7 个字符，16 足够
};

QueueHandle_t cmdQueue = nullptr;   // 队列句柄，相当于"传菜口"的门牌号
```

**为什么不用 `String`？** `String` 会在堆上 `new`/`delete`，在一个任务里创建、在另一个任务里销毁，跑久了会产生内存碎片。定长结构体是按值拷贝，零分配。

### 2-C 加一个空的任务函数

找到这一行（原文件第 232 行）：

```cpp
// ======================== setup() ========================
```

在它**上面**插入：

```cpp
// ======================== 运动任务（先放空壳）========================
static void motionTask(void* pv) {
  (void)pv;
  // 记录"下一次该醒的时刻"，后面用它做固定节拍
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    // 【第 3 关】这里会加上 hand.run()
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
  }
}
```

`vTaskDelayUntil(&lastWake, 10ms)` 的意思是"到 lastWake+10ms 这个**绝对时刻**再叫醒我"，而 `vTaskDelay(10)` 是"从**现在**再睡 10ms"。前者不会累积误差，后者会。舵机速度完全由这个周期决定，所以必须用前者。

### 2-D 在 setup 里把队列和任务创建出来

找到 `setup()` 里这一行（原文件第 263 行）：

```cpp
  BLEDevice::init("xiaochu");
```

在它**上面**插入：

```cpp
  // ---- FreeRTOS：顺序必须是 队列 → 任务 → BLE ----
  cmdQueue = xQueueCreate(8, sizeof(HandCommand));
  if (cmdQueue == nullptr) {
    Serial.println("FATAL: 队列创建失败（内存不够）");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  xTaskCreate(motionTask, "motion", 4096, nullptr, 4, nullptr);
  Serial.println("motion task started, period=10ms");
```

**顺序为什么不能反？** 队列必须在 BLE 启动前建好，否则设备连上的瞬间来了 `onWrite`，那时 `cmdQueue` 还是空指针，直接崩溃。

### ✅ 第 2 关验证

烧录，串口应该出现：

```
motion task started, period=10ms
BLE ready, waiting for connection...
```

然后连上位机，试手势和单指角度——**动作必须和改造前一模一样**。因为此时 `motionTask` 是空壳，真正干活的还是 `loop()`。

> 如果机械手不动了：说明你把 `loop` 里的 `hand.run()` 删早了，加回去，第 3 关才删。

---

## 第 3 关：把插值搬进任务（第一次"搬家"）

**目标：让 `motionTask` 真正干活，`loop()` 只留广播看护。**

### 3-A 删掉 loop 里的活

找到文件末尾的 `loop()`（原文件第 284~297 行），它现在长这样：

```cpp
void loop() {
  hand.run();
  if (needRestartAdv) {
    delay(300);
    BLEDevice::getAdvertising()->start();
    needRestartAdv = false;
    Serial.println("[BLE] 已重新开始广播，可以再次连接");
  }
  delay(10);
}
```

改成：

```cpp
void loop() {
  if (needRestartAdv) {
    vTaskDelay(pdMS_TO_TICKS(300));   // delay() 也能用，但这里统一成 RTOS 写法
    BLEDevice::getAdvertising()->start();
    needRestartAdv = false;
    Serial.println("[BLE] 已重新开始广播，可以再次连接");
  }
  vTaskDelay(pdMS_TO_TICKS(100));     // 看护任务不需要跑得快，100ms 够了
}
```

### 3-B 把 hand.run() 放进任务

回到 `motionTask`，把 `// 【第 3 关】这里会加上 hand.run()` 这行注释换成：

```cpp
    hand.run();
```

### ✅ 第 3 关验证

烧录，动作必须**还是不変**。

如果动作明显变快或变慢：检查 `pdMS_TO_TICKS(10)` 是不是写成了 1 或 100。`pdMS_TO_TICKS` 是按当前 tick 频率换算，别自己手算。

**这一关结束后，隐患出现了**：`onWrite` 还在 BLE 任务里直接改 `hand` 的目标值，而 `motionTask` 正在读同一个对象。两个任务同时碰一块内存，这叫**数据竞争**。现在还没炸，是因为 `float` 读写在 C3 上是原子的，纯属运气。第 4 关把它彻底解决。

---

## 第 4 关：让蓝牙只"递纸条"（核心一关）

**目标：`onWrite` 不再碰机械手，只把指令塞进队列。**

### 4-A 加入队函数

找到 2-B 里加的 `QueueHandle_t cmdQueue = nullptr;`，在它**下面**插入：

```cpp
// 入队：队列满时丢最旧的一条，保证最新指令一定生效
static bool enqueueCommand(const HandCommand& cmd) {
  if (cmdQueue == nullptr) return false;
  if (xQueueSend(cmdQueue, &cmd, 0) == pdTRUE) return true;   // 第三个参数 0 = 不等待

  HandCommand dropped;
  xQueueReceive(cmdQueue, &dropped, 0);     // 扔掉队头那条最旧的
  return xQueueSend(cmdQueue, &cmd, 0) == pdTRUE;
}
```

> 为什么第三个参数是 0（不等待）？因为这是从蓝牙回调里调的，绝不能在这里卡住等队列腾位置，会拖垮整个协议栈。

### 4-B 整个替换 onWrite

找到 `MyCallbacks` 这个类（原文件第 189~215 行），**整段**替换成：

```cpp
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    auto value = pCharacteristic->getValue();
    if (value.length() == 0) return;

    String cmd = String(value.c_str());
    cmd.trim();
    if (cmd.length() == 0) return;

    Serial.print("Received: ");
    Serial.println(cmd);

    // ---- 从这里开始和老代码不一样：只填结构体，不动 hand ----
    HandCommand c = {};
    c.type = CMD_GESTURE;

    if (cmd.startsWith("servo_")) {
      int eqPos = cmd.indexOf('=');
      if (eqPos < 0) return;

      int finger = cmd.substring(6, eqPos).toInt();
      if (finger < 0 || finger > 4) return;

      int angle = constrain(cmd.substring(eqPos + 1).toInt(), 0, 180);
      c.type   = CMD_ANGLE;
      c.finger = (uint8_t)finger;
      c.angle  = (uint8_t)angle;
    } else {
      strncpy(c.gesture, cmd.c_str(), sizeof(c.gesture) - 1);
      c.gesture[sizeof(c.gesture) - 1] = '\0';
    }

    enqueueCommand(c);      // 递纸条，完事
  }
};
```

注意老代码里的 `hand.moveFingerAngle(...)` 和 `HandMovementFactory::applyGesture(&hand, cmd)` **全部删掉**了——它们搬去了下一段。

### 4-C 加"执行指令"的函数

回到 `motionTask` 上方，在 `// ======================== 运动任务` 那行**上面**插入：

```cpp
// 把一条指令真正作用到机械手上（只有 motionTask 会调用它）
static void applyCommand(const HandCommand& c) {
  switch (c.type) {
    case CMD_GESTURE:
      if (c.gesture[0] != '\0') {
        Serial.printf("[MOTION] gesture=%s\n", c.gesture);
        HandMovementFactory::applyGesture(&hand, String(c.gesture));
      }
      break;

    case CMD_ANGLE:
      Serial.printf("[MOTION] finger %u -> %u\n", c.finger, c.angle);
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
```

### 4-D 让任务每拍先清空队列

把 `motionTask` 改成这样：

```cpp
static void motionTask(void* pv) {
  (void)pv;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    // 1) 把队列里积压的指令全部处理掉（非阻塞，没指令就立刻返回）
    HandCommand cmd;
    while (xQueueReceive(cmdQueue, &cmd, 0) == pdTRUE) {
      applyCommand(cmd);
    }

    // 2) 推进 5 根手指的插值
    hand.run();

    // 3) 固定 10ms 节拍
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
  }
}
```

### ✅ 第 4 关验证

烧录，串口应该看到成对的日志：

```
Received: fist
[MOTION] gesture=fist
```

试这些，全部要正常：`fist` / `five` / `ok` / `rock`，单指滑块 `servo_2=90`，自定义输入框。

**验证解耦成功了没**：上位机快速连点 10 个不同手势，机械手应该**只做最后一个**（中间的被队列淘汰或在一拍内连续覆盖），而不是排队慢慢做完 10 个。这就是控制指令该有的行为。

---

## 第 5 关：两个小优化（可选，但面试能讲）

### 5-A 断连自动回张开位

找到 `MyServerCallbacks::onDisconnect`（原文件第 226 行），改成：

```cpp
  void onDisconnect(BLEServer* pServer) override {
    Serial.println("[BLE] 上位机已断开");
    needRestartAdv = true;          // 不在回调里直接阻塞，交给 loop() 处理

    HandCommand home = {};
    home.type = CMD_HOME;
    enqueueCommand(home);           // 断连回张开位，机械手不会僵在半紧状态
  }
```

好处：避免 Finger 停在半握状态憋着舵机（长期堵转会发热、耗电、伤齿轮）。

### 5-B 没动就不写 I2C

改两个函数的返回值。`Finger::run()`（原文件第 78~88 行）改成：

```cpp
  bool run() {                        // void 改成 bool
    float before = currentPosition;

    if (fabsf(currentPosition - target) <= step) {
      currentPosition = target;
    } else if (currentPosition < target) {
      currentPosition += step;
    } else {
      currentPosition -= step;
    }

    if (currentPosition == before) return false;   // 没动，不写总线

    int angle = (int)(currentPosition + 0.5f);
    int pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(channel, 0, pulse);
    return true;
  }
```

`Hand::run()`（第 108 行）改成：

```cpp
  bool run() {
    bool moved = false;
    for (int i = 0; i < 5; i++) if (fingers[i]->run()) moved = true;
    return moved;
  }
```

⚠️ 注意 `abs()` 换成了 `fabsf()`：Arduino 里的 `abs` 是个宏，会把参数求值两次，传表达式（比如 `currentPosition - target`）时容易出意外。对 float 明确写 `fabsf()` 更稳。

---

## 第 6 关：观测与调参

### 6-A 打开栈水位打印

在文件顶部宏定义区加：

```cpp
#define RTOS_DEBUG 1
```

在 `motionTask` 里加：

```cpp
    static uint32_t tick = 0;
    if (++tick % 500 == 0) {      // 每 5 秒一次
      Serial.printf("[RTOS] stack free = %u bytes\n",
                    (unsigned)uxTaskGetStackHighWaterMark(NULL));
    }
```

**怎么读**：这个数字是"历史最低剩余量"。

| 剩余 | 结论 |
|---|---|
| > 1500 字节 | 4096 够用，安全 |
| 500~1500 | 有点紧，建议调到 6144 |
| < 500 或频繁重启 | 立刻加栈，否则某个 `printf` 就爆 |

### 6-B 测节拍是否真的稳了

在 `motionTask` 里加：

```cpp
    static uint32_t t0 = micros();
    static uint32_t n = 0;
    if (++n % 100 == 0) {                        // 每 100 拍 = 理论 1 秒
      uint32_t now = micros();
      Serial.printf("[TICK] 100 拍耗时 %u us\n", now - t0);
      t0 = now;
    }
```

**应该接近 1000000 µs（±几 ms）**。如果你把 `vTaskDelayUntil` 换成 `vTaskDelay` 再测，会看到明显偏大——这个对比就是你能写进简历的实验数据。

---

## 报错急救表

| 现象 | 原因 | 修法 |
|---|---|---|
| `'cmdQueue' was not declared in this scope` | 队列句柄定义在 `MyCallbacks` 后面，回调看不到 | 把 2-B 那段挪到 `MyCallbacks` 类**之前** |
| `'motionTask' was not declared` | 任务函数定义在 `setup()` 之后 | 只能放前面，或者在 `setup()` 上方加一句 `static void motionTask(void* pv);` 声明 |
| 编译过，但机械手完全不动 | 忘了在 `motionTask` 里写 `hand.run()`；或 `loop()` 里的 `hand.run()` 删了但任务没加 | 检查 3-B |
| 一上电就反复重启 | 栈溢出 | 4096 → 8192 |
| 能连上但指令没反应 | `onWrite` 里还残留老代码，或 `applyCommand` 的 switch 漏了分支 | 串口看有没有 `[MOTION]` 日志 |
| 手势做了但只做了一半 | `hand.run()` 写在出队循环外面是对的；如果写成 `if` 只取一条，多条指令会延迟 | 用 `while` 清空队列 |
| 断开后再也连不上 | 第 3 关改 `loop()` 时把 `needRestartAdv` 那段删了 | 对照 3-A 恢复 |

---

## 全部做完后，你会得到什么

| 能力 | 说明 |
|---|---|
| 会建任务、会用队列、懂阻塞和优先级 | 这是 FreeRTOS 面试的 80% |
| 能说清"什么该放任务、什么该放回调" | 上面试一问就知道你动过手 |
| 有一份"改造前后对比"的实测数据 | 节拍漂移、栈水位，比空谈可靠 |

**别做的事**：不要为了显得高级，把 5 个舵机拆成 5 个任务。5 个任务抢同一根 I2C 总线，还得加互斥锁，只会更慢更乱。**"一个写者"比"多个任务"值钱。**

---

## 附录 A：四个基础概念速查（A / B / C / D）

> 改造过程中卡住的每一个"看不懂"，基本都落在这四块里。这一节是回头查用的速查表，不按顺序读也没问题。

### A 类型：这个变量占几个格子

| 类型 | 字节 | 备注 |
|---|---|---|
| `char` | 1 | 一个字符 |
| `bool` | 1 | 真 / 假 |
| `uint8_t` | 1 | 0~255，省内存时用 |
| `int` | 4 | ESP32 上是 4；Arduino UNO 上是 **2** |
| `float` | 4 | 小数 |
| `void*` | 4 | 一个地址（32 位芯片，地址就是 4 字节） |

- `sizeof` 是**运算符**，不是函数，编译期就把数字填好，运行时零开销。
- `_t` 结尾的都是类型名（`size_t`、`TickType_t`、`QueueHandle_t`），只能用来声明变量。
- **`QueueHandle_t` = 取餐号码牌**：内部结构长什么样你不需要知道，只要拿牌子去窗口操作。

### B `&` 与队列的复制语义

| 符号 | 方向 | 例子 |
|---|---|---|
| `&` 取地址 | 变量 → 地址 | `&cmd` |
| `*` 解引用 | 地址 → 变量 | `*(&cmd)` 等于 `cmd` |

- `xQueueSend` 收的是 `const void*`（万能地址），它不知道类型，**只能靠 `xQueueCreate` 时给的 `sizeof` 决定复制多少字节**。
- **队列存的是副本，不是地址。** 所以用局部变量 `HandCommand c = {};` 填完直接入队是安全的，函数返回后 `c` 销毁也不影响队列。
- 三处必须配套：`xQueueCreate(8, sizeof(HandCommand))` → `xQueueSend(q, &cmd, 0)` → 内部 `memcpy` 24 字节。`sizeof` 写小了是**静默错误**，编译不报。
- **数组名本身就是首地址**，`char buf[16]` 传 `buf` 即可，不要写 `&buf`。

### C 参数顺序：不需要背，记两条模板

| 模板 | 顺序 | 用在哪 |
|---|---|---|
| 对象 → 数据 → 策略 | `xQueueSend(哪个队列, 传什么, 不行等多久)` | RTOS 通讯类函数 |
| 值 → 输入域 → 输出域 | `map(值, 输入下限, 输入上限, 输出下限, 输出上限)` | `map`、`constrain` |

- `xQueueSend` 与 `xQueueReceive` 的签名**刻意对称**，位置一一对应，记一次就够。
- **最危险的情况**：类型相同、顺序写错，编译器完全不报警。例如 `constrain(0, 180, target)` 能编译，但结果永远是 180（手指只能卡在全合位置）。
- 项目里的证据：第 71 / 73 行两个 `map` 只对调了最后两个参数，就实现了手指方向反转——**位置本身带语义**。

### D 任务 / 优先级 / 栈

- **任务 = 永不返回的函数 + 私有栈 + 优先级**。普通函数 `return` 就结束，任务只在 `for(;;)` 里循环。
- **优先级数字越大越优先**（0~24）。高优先级任务一就绪就**抢占**，低优先级任务的现场原样保留，之后接着跑。
- 项目排布：蓝牙任务（二十几）> `motionTask`（4）> `loopTask`（1）。4 这个数是取舍：高于 `loop` 保证节拍，低于蓝牙保证收包。
- **栈单位是字节**（不是字）。`4096` = 4KB。溢出表现为随机重启 / `Guru Meditation Error`，用 `uxTaskGetStackHighWaterMark()` 看水位，**大于 1500 字节**才安全。
- **四个状态**：Running（在跑）、Ready（想跑但轮不到）、Blocked（等时间或等数据）、Suspended（被人为挂起）。**只有 Running 占 CPU。**
- **优先级 ≠ 算力。** 调高优先级只改变"同时就绪时谁先跑"，不会增加分配到的 CPU 时间。

### 串起来：一条蓝牙指令的完整旅程

```
手机发 "fist"
  ↓  BLE 协议栈任务（优先级二十几）收包
  ↓  onWrite 回调：解析字符串 → 填 HandCommand（A：24 字节）
  ↓  xQueueSend(cmdQueue, &cmd, 0)（B：复制副本；C：对象→数据→策略；0 = 满了不等）
  ↓  cmdQueue 里躺着一条 24 字节的指令
  ↓  motionTask 每 10ms 醒来（D：优先级 4、栈 4096、平时处于 Blocked）
  ↓  while 出队 → 改 target → 插值（每拍 2°）→ map（C：值→输入域→输出域）→ setPWM
  ↓  PCA9685 输出 50Hz PWM
舵机转到目标角度
```

**这条链路记住，面试时按这个顺序讲，对方立刻知道你动过手。**

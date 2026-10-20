// ========================================================================
//  实验 02：用队列把数据从"生产者"传给"消费者"
// ------------------------------------------------------------------------
//  这是改造机械手最关键的第二个零件。场景：
//     生产者（BLE 收指令）= 很快，随时可能来
//     消费者（运动任务）    = 慢，固定 10ms 才处理一次
//  如果生产者直接去改消费者的数据，两边会打架；用队列就不会。
//
//  队列的三个性质，记住就行：
//     1. 长度固定，创建时定死，之后不会扩容
//     2. 存的是【值的拷贝】，不是指针（所以跨任务很安全）
//     3. 满了再写会失败（返回 pdFALSE），空了再读也会失败
//
//  本实验：生产者每 300ms 产一个数字，消费者每 1000ms 吃一个。
//  明显消费不过来 → 队列会满 → 演示两种处理策略。
// ========================================================================

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define QUEUE_LEN 1

QueueHandle_t demoQueue = nullptr;
int produced = 0;
int dropped  = 0;

// ------------------------------------------------------------------------
//  生产者：每 300ms 往队列塞一个数
// ------------------------------------------------------------------------
void producerTask(void* pv) {
  (void)pv;
  for (;;) {
    int value = ++produced;

    // 先试试能不能塞进去（第三个参数 0 = 不等，塞不进立刻返回）
    if (xQueueSend(demoQueue, &value, 0) == pdTRUE) {
      Serial.printf("[产] %d 入队，队列里现在 %d 个\n",
                    value, (int)uxQueueMessagesWaiting(demoQueue));
    } else {
      // ---- 策略 A：丢掉刚生产的这个（新手最容易写成的样子）----
      dropped++;
      Serial.printf("[产] 队列满了，丢弃 %d（累计丢 %d）\n", value, dropped);
    }



    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

// ------------------------------------------------------------------------
//  消费者：每 1000ms 才吃一个，故意比生产者慢
// ------------------------------------------------------------------------
void consumerTask(void* pv) {
  (void)pv;
  for (;;) {
    int got = -1;
    // 第二个参数 0 = 不等；如果写 pdMS_TO_TICKS(100) 就是最多等 100ms
    if (xQueueReceive(demoQueue, &got, 0) == pdTRUE) {
      Serial.printf("        [消] 拿到 %d，还剩 %d 个\n",
                    got, (int)uxQueueMessagesWaiting(demoQueue));
    } else {
      Serial.println("        [消] 队列空了，没得吃");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== 实验 02 开始 ===");

  // 创建一个能装 3 个 int 的队列
  //   长度 3，每个元素 sizeof(int) 字节
  demoQueue = xQueueCreate(QUEUE_LEN, sizeof(int));
  if (demoQueue == nullptr) {
    Serial.println("队列创建失败（内存不够）");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  xTaskCreate(producerTask, "producer", 4096, nullptr, 3, nullptr);
  xTaskCreate(consumerTask, "consumer", 4096, nullptr, 2, nullptr);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(5000));
}

// ========================================================================
//  烧录后你会看到的关键现象
// ------------------------------------------------------------------------
//  1. 生产者跑得比消费者快，队列很快满了，开始出现"丢掉最旧的 N"
//  2. 消费者每次拿到的永远是队列里【最新的那个】，中间的值被丢掉了
//     → 这就是控制指令的正确语义：用户拖滑块，只要最后一个位置生效
//  3. 打开上面的"策略 A"（注释掉 B，取消注释 dropped++）再跑一遍：
//     你会发现消费者拿到的是一串很久以前的旧值 —— 这就是错误策略的样子
//
//  动手改一改：
//    a) 把 QUEUE_LEN 改成 1，再跑，看行为变化
//    b) 把消费者的 xQueueReceive 第三个参数改成 pdMS_TO_TICKS(2000)
//       这叫"带超时阻塞"：队列空时它会让出 CPU 等着，比死循环轮询省电
//    c) 队列长度设 1 时，其实更优雅的写法是 xQueueOverwrite(demoQueue, &value)
//       它永远只保留最新值，且不会返回失败
// ========================================================================

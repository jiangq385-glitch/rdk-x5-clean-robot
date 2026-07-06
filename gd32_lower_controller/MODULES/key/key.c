#include "key.h"



extern TaskHandle_t ControlTask_Handler;
extern TaskHandle_t LEDTask_Handler;

// 按键消息队列
QueueHandle_t xKeyQueue = NULL;
volatile uint8_t g_emergencyStop = 0;

#define KEY_SCAN_PERIOD_MS     10u     // 扫描周期 10ms
#define KEY_DEBOUNCE_COUNT     3u      // 连续稳定 3 次才认为状态改变（约 30ms 防抖）

static uint8_t key_is_pressed(uint32_t port, uint32_t pin)
{
    // 上拉输入：按下为 0
    return (gpio_input_bit_get(port, pin) == RESET) ? 1u : 0u;
}

// 按键状态结构体数组
static KeyDebounce_t s_keys[] =
{
    { KEY0_PORT, KEY0_PIN, KEY0_PRESSED, 0u, 0u, 0u },
    { KEY1_PORT, KEY1_PIN, KEY1_PRESSED, 0u, 0u, 0u },
    { KEY2_PORT, KEY2_PIN, KEY2_PRESSED, 0u, 0u, 0u },
};

//按键初始化
void KEY_Init(void)
{
	/* 使能 GPIO 时钟 */
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOE);

	/* 上拉输入：按下为 0 */
	gpio_mode_set(KEY1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, KEY1_PIN);
	gpio_mode_set(KEY0_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, KEY0_PIN | KEY1_PIN);

    // 队列长度建议大一点，避免连按时丢事件
   
} 

//按键扫描任务
void KEY_ScanTask(void *pvParameters)
{
    TickType_t lastWake;
    uint32_t i;

    (void)pvParameters;
    lastWake = xTaskGetTickCount();

    for (;;)
    {
        for (i = 0; i < (sizeof(s_keys) / sizeof(s_keys[0])); i++)
        {
            uint8_t raw = key_is_pressed(s_keys[i].port, s_keys[i].pin);

            if (raw == s_keys[i].lastRaw)
            {
                if (s_keys[i].stableCount < KEY_DEBOUNCE_COUNT)
                {
                    s_keys[i].stableCount++;
                }
            }
            else
            {
                s_keys[i].lastRaw = raw;
                s_keys[i].stableCount = 0;
            }

            if (s_keys[i].stableCount == KEY_DEBOUNCE_COUNT)
            {
                // 状态确认发生改变：更新防抖后的稳定态
                if (s_keys[i].debounced != raw)
                {
                    s_keys[i].debounced = raw;

                    // 只在“按下”瞬间发一次事件（松开不发）
                    if (raw == 1u)
                    {
                        int8_t evt = s_keys[i].eventPressed;
                        if (xKeyQueue != NULL)
                        {
                            // 任务里发队列：用 xQueueSend（不用 FromISR）
                            (void)xQueueSend(xKeyQueue, &evt, 0);
                        }
                    }
                }
            }
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(KEY_SCAN_PERIOD_MS));
    }
}

void key_control(void)
{
     int8_t evt;

    for (;;)
    {
        if (xQueueReceive(xKeyQueue, &evt, portMAX_DELAY) == pdPASS)
        {
            if (evt == KEY0_PRESSED)
            {
                g_emergencyStop = (uint8_t)!g_emergencyStop;

                if (g_emergencyStop)
                {
                   // 立刻停 + 失能（按需改 addr）
                 Emm_V5_Stop_Now(1, false);
                 Emm_V5_En_Control(1, false, false);
                 Emm_V5_Stop_Now(2, false);
                 Emm_V5_En_Control(2, false, false);
                //  vTaskSuspend(ControlTask_Handler); 
			  vTaskSuspend(LEDTask_Handler);
                }
                else
                {
                     // 恢复使能（按需改 addr）
                 Emm_V5_En_Control(1, true, false);
                 Emm_V5_En_Control(2, true, false);
                // vTaskResume(ControlTask_Handler);
                vTaskResume(LEDTask_Handler);
                }
            }
        }
    }
}
















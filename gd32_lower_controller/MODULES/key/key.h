#ifndef __KEY_H
#define __KEY_H	 
#include "main.h"
#include "task.h"
#include "queue.h"

//按引脚定义

#define KEY0_PIN         GPIO_PIN_9
#define KEY0_PORT        GPIOC

#define KEY1_PIN         GPIO_PIN_8
#define KEY1_PORT        GPIOC

#define KEY2_PIN         GPIO_PIN_12
#define KEY2_PORT        GPIOE


// 按键事件定义
#define KEY0_PRESSED     1
#define KEY1_PRESSED     2
#define KEY2_PRESSED     3
#define KEY3_PRESSED     4


// 按键状态结构体
typedef struct
{
    uint32_t port;
    uint32_t pin;
    int8_t eventPressed;      // 按下事件值：KEY0_PRESSED...
    uint8_t debounced;        // 0=松开, 1=按下（防抖后的稳定态）
    uint8_t lastRaw;          // 上一次原始采样
    uint8_t stableCount;      // 原始采样保持不变的次数
} KeyDebounce_t;


// 按键事件队列句柄（在 key.c 中定义）
extern QueueHandle_t xKeyQueue;
extern volatile uint8_t g_emergencyStop;


void KEY_Init(void);	//IO初始
void KEY_ScanTask(void *pvParameters);//按键扫描任务
void key_control(void); //按键事件处理函数
#endif

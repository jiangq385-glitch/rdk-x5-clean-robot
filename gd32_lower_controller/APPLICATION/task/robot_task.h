
#include "main.h"
#include "encoder.h"
#include "exchange.h"
#include "key.h"


#define START_TASK_PRIO     1       //起始任务优先级
#define LED_TASK_PRIO      2       //LED任务优先级  
#define CONTROL_TASK_PRIO      3       //控制任务优先级
#define KEY_TASK_PRIO     4       //按键任务优先级

//任务堆栈大小定义
#define START_STK_SIZE      256     //起始任务堆栈
#define LED_STK_SIZE       100      //LED任务堆栈
#define CONTROL_STK_SIZE    256     //电机任务堆栈  
#define IMU_STK_SIZE       100      //IMU任务堆栈  
#define KEY_STK_SIZE       100      //按键任务堆栈  


//任务句柄声明
TaskHandle_t StartTask_Handler;
TaskHandle_t LEDTask_Handler; 
TaskHandle_t ControlTask_Handler;
TaskHandle_t KEYTask_Handler;

//函数声明
void start_task(void *pvParameters);
void led_task(void *pvParameters);
void control_task(void *pvParameters);
void key_task(void *pvParameters);

//任务创建初始化
int Task_Init(void)
{ 
    //创建起始任务
    xTaskCreate(
        (TaskFunction_t)start_task,       //任务函数指针
        "start_task",                      //任务名称(字符串)
        START_STK_SIZE,                    //任务堆栈大小
        NULL,                              //任务参数
        START_TASK_PRIO,                   //任务优先级
        &StartTask_Handler                 //任务句柄指针
    );
    
    vTaskStartScheduler();  //启动FreeRTOS调度器

    return 0; //正常情况下不会执行到这里
}

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();  //进入临界区
    
    //创建LED任务(500ms周期闪烁)
    xTaskCreate(
        led_task,
        "led_task",
        LED_STK_SIZE,
        NULL,
        LED_TASK_PRIO,
        &LEDTask_Handler
    );
    
   // 创建控制任务
   xTaskCreate(
       control_task,
       "control_task",
       CONTROL_STK_SIZE,
       NULL,
       CONTROL_TASK_PRIO,
       &ControlTask_Handler
   );
    
   // 创建按键任务
   xTaskCreate(
       key_task,
       "key_task", 
       KEY_STK_SIZE,
       NULL,
       KEY_TASK_PRIO,
       &KEYTask_Handler
   );
    
    
    vTaskDelete(StartTask_Handler);  //删除自身任务
    taskEXIT_CRITICAL();             //退出临界区
}

//LED任务函数
void led_task(void *pvParameters)
{
    
   
    while(1)
    {
        
        
        led0_on();  //LED0亮
        vTaskDelay(500);       //延时500ms
		
        led0_off(); //LED0灭
        vTaskDelay(500);       //延时500ms
    }
}
extern float Yaw_Angle;
//机器人任务函数  
int a;


float deg;
float s;
extern AS5048 AS5048s[AS5048_NUMBER];
extern ENCl ENCLs[Encoder_NUMBER];
extern MPU_DATA mpu_data;
void control_task(void *pvParameters)
{

    Emm_V5_En_Control(1, true, false);
    Emm_V5_En_Control(2, true, false);

    TickType_t last_imu_tick = xTaskGetTickCount();
    TickType_t last_odo_tick = xTaskGetTickCount();

    while(1)
    {
        TickType_t now = xTaskGetTickCount();

        if (g_emergencyStop)
        {
            Emm_V5_Stop_Now(1, false);
            Emm_V5_Stop_Now(2, false);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // 每 20ms 读取 IMU
        if ((now - last_imu_tick) >= pdMS_TO_TICKS(20)) {
            Read_imu_data();
            last_imu_tick = now;
        }

        // 每 20ms 计算里程
        if ((now - last_odo_tick) >= pdMS_TO_TICKS(20)) {
            // 发送 ENCL 读取请求
            odom_read_task();
            last_odo_tick = now;
        }
        exchange_task();
        chasis_task();
       // arm_bus_task();
        // 电机控制（可放在这里或单独的循环里）
//         Emm_V5_Vel_Control(1, 1, 5, 5, 0);
//         Emm_V5_Vel_Control(2, 1, 5, 5, 0);

        vTaskDelay(20);

            
    }
}


//按键任务函数
void key_task(void *pvParameters)
{ 
    if (xKeyQueue == NULL)
    {
            xKeyQueue = xQueueCreate(8, sizeof(int8_t));
    }

    xTaskCreate(KEY_ScanTask, "keyscan", 256, NULL, tskIDLE_PRIORITY+3, NULL);
        key_control();
       
    
}

//#endif

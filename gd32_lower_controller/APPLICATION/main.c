/***************************************************************************//**
  文件: main.c
  作者: Zhengyu https://gzwelink.taobao.com
  版本: V1.0.0
  时间: 20220401
	平台:MINI-F407VET6

*******************************************************************************/

#include "main.h"
#include "robot_init.h"
#include "robot_task.h"

int main(void)
{
     Bsp_init();
    //等待按键KEY0按下启动系统
    delay_xms(1000);
	//任务初始化
    Task_Init();
}

#ifndef __LED_H
#define __LED_H
#include "sys.h"


//LED端口定义
#define LED0 PDout(3)	// DS0
#define LED1 PDout(4)	// DS1	 

void LED_Init(void);//初始化	
void led0_on(void);  //LED0亮
void led0_off(void); //LED0灭  
void led1_on(void);  //LED1亮
void led1_off(void); //LED1灭
void led0_turn(void); //LED0翻转
void led1_turn(void); //LED1翻转
void led2_on(void);
void led2_off(void);

#endif

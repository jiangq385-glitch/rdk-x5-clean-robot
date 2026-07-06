//产生8路同频的PWM信号
#include "pwm.h"
			  					  		
/**************************************************************************
函数功能：TIM1产生4路PWM（PE9、PE11、PE13、PE14）
入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void TIM1_PWM_Init(u16 arr,u16 psc)
{		
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOE,&GPIO_InitStructure);
  //PE11:左前,PE13:左后,PE9:右前,PE14:右后
	
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);	
//	TimerPeriod =  (SystemCoreClock / 20000 ) ; 				//168M/20000 
//  ccr1 = 0;//TimerPeriod /2;  //占空比1/2 = 50%
//  ccr2 = 0;//TimerPeriod / 3;  //占空比1/3 = 33%
//  ccr3 = TimerPeriod / 1.1;  //占空比1/4 = 25%
//  ccr4 = 0;//TimerPeriod / 1;  //占空比1/5 = 20%
    
	//时基初始化
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //死区控制用。
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  //计数器方向
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;   //Timer clock = sysclock /(TIM_Prescaler+1) = 168M
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInitStructure.TIM_Period = arr;    //20K
  TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
 
//  TIM_OCInitStructure.TIM_Pulse = ccr1;
  TIM_OC1Init(TIM1,&TIM_OCInitStructure);
//  TIM_OCInitStructure.TIM_Pulse = ccr2;
  TIM_OC2Init(TIM1,&TIM_OCInitStructure);  
////  TIM_OCInitStructure.TIM_Pulse = ccr3;
//  TIM_OC3Init(TIM1,&TIM_OCInitStructure); 
////  TIM_OCInitStructure.TIM_Pulse = ccr4;
//  TIM_OC4Init(TIM1,&TIM_OCInitStructure);
  
  TIM_Cmd(TIM1,ENABLE);
  TIM_CtrlPWMOutputs(TIM1,ENABLE);
}  
void TIM9_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM9); //GPIOB9复用为定时器4
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure); 
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	
	TIM_OC2Init(TIM9,&TIM_OCInitStructure);
	
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM9,ENABLE);  //ARPE使能 
	
	TIM_Cmd(TIM9,ENABLE);
}






#ifndef __PWM_H
#define __PWM_H
#include "sys.h"


//void TIM4_PWM_Init(u32 arr,u32 psc);
void TIM1_PWM_Init(u16 arr,u16 psc);
void TIM9_PWM_Init(u16 arr,u16 psc);
//static void TIM8_PWM_Init(u16 arr,u16 psc);
//void PWM_Init(u16 arr,u16 psc);


#endif

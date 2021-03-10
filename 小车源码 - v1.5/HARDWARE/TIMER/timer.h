#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

extern int temp;
extern int t2;
void TIM3_Int_Init(u16 arr,u16 psc);
	
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM6_Int_Init(u16 arr,u16 psc);
void TIM7_Int_Init(u16 arr,u16 psc);
#endif

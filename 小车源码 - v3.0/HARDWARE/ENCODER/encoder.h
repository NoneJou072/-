#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h" 
#include "sys.h" 

#define ENCODER_TIM_PERIOD (u16)(65535)   //不可大于65535 因为F103的定时器是16位的。

extern int cntCount, cntCount1, cntCount2, cntCount3, cntCount4, cntCountNew;
void TIM2_Encoder_Config(void);
void TIM4_Encoder_Config(void);
void TIM5_Encoder_Config(void);
void TIM8_Encoder_Config(void);

s16 getTIMx_DetaCnt(TIM_TypeDef * TIMx);
void Get_Motor_Speed(int *leftSpeed,int *leftSpeed2,int *rightSpeed,int *rightSpeed2);

#endif

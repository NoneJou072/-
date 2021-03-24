#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h" 

extern int leftSpeedSet, rightSpeedSet, leftSpeedSet2, rightSpeedSet2;//mm/s//速度设定值

void Set_Pwmm(int left,int left2,int right,int right2);//左后，左前
int myabs(int a);
void Timer3_PWM_SetDutyCycle(u8 ch, u16 Timer3_PWM_DutyCycle);

void Motor_Straight(void);
void Motor_Stop(void);
void Motor_TLEFT(void);
void Motor_TRIGHT(void);
void Motor_Back(void);
void Motor_TurnR(void);
void Motor_TurnL(void);
void Motor_XIE(int dir,int ang);

#endif

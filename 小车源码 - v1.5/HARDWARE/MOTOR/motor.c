#include "motor.h"
#include "led.h"
#include "delay.h"
#include "timer.h"
#include "sys.h"



//电机方向

void Motor_Straight(){//直行

	leftSpeedSet = 2000;
	rightSpeedSet = 2000;
	leftSpeedSet2 = 2000;
	rightSpeedSet2 = 2000;
	Set_Pwmm(600,600,600,600);
	LAIN1 = 0;//左前
	LAIN2 = 1;
	LAIN3 = 0;//左后
	LAIN4 = 1;
	RAIN1 = 1;//右前
	RAIN2 = 0;
	RAIN3 = 0;//右后
	RAIN4 = 1;
}

void Motor_Stop(){//停止
	LAIN1 = !LAIN1;//左前
	LAIN2 = !LAIN2;
	LAIN3 = !LAIN3;//左后
	LAIN4 = !LAIN4;
	RAIN1 = !RAIN1;//右后
	RAIN2 = !RAIN2;
	RAIN3 = !RAIN3;//右前
	RAIN4 = !RAIN4;
	Set_Pwmm(300,300,300,300);
	delay_ms(50);
	Set_Pwmm(0,0,0,0);
	LAIN1 = 0;//左前
	LAIN2 = 0;
	LAIN3 = 0;//左后
	LAIN4 = 0;
	RAIN1 = 0;//右后
	RAIN2 = 0;
	RAIN3 = 0;//右前
	RAIN4 = 0;
	leftSpeedSet   = 0;//mm/s//速度设定值
  rightSpeedSet = 0;//mm/s
  leftSpeedSet2   = 0;//mm/s
  rightSpeedSet2 = 0;//mm/s
	
}
void Motor_TLEFT(){//向左直行
  
	leftSpeedSet = 2000;
	rightSpeedSet = 2000;
	leftSpeedSet2 = 2000;
	rightSpeedSet2 = 2000;
	LAIN1 = 0;//左前
	LAIN2 = 1;
	LAIN3 = 1;//左后
	LAIN4 = 0;
	RAIN1 = 0;//右前
	RAIN2 = 1;
	RAIN3 = 0;//右后
	RAIN4 = 1;
}
void Motor_TRIGHT(){//向右直行
	Set_Pwmm(800,820,800,800);
  leftSpeedSet = 2000;
	rightSpeedSet = 2000;
	leftSpeedSet2 = 2000;
	rightSpeedSet2 = 2000;
	//PWM_TRIGHT();
	LAIN1 = 1;//左前
	LAIN2 = 0;
	LAIN3 = 0;//左后
	LAIN4 = 1;
	RAIN1 = 1;//右前
	RAIN2 = 0;
	RAIN3 = 1;//右后
	RAIN4 = 0;
}

void Motor_Back(){//后退
	leftSpeedSet = 2000;
	rightSpeedSet = 2000;
	leftSpeedSet2 = 2000;
	rightSpeedSet2 = 2000;
	Set_Pwmm(600,600,600,600);
	LAIN1 = 1;//左前
	LAIN2 = 0;
	LAIN3 = 1;//左后
	LAIN4 = 0;
	RAIN1 = 0;//右前
	RAIN2 = 1;
	RAIN3 = 1;//右后
	RAIN4 = 0;
}

void Motor_TurnR(){//右转

	Set_Pwmm(200,200,200,200);
	LAIN1 = 1;//左前
	LAIN2 = 0;
	LAIN3 = 1;//左后
	LAIN4 = 0;
	RAIN1 = 1;//右前
	RAIN2 = 0;
	RAIN3 = 0;//右后
	RAIN4 = 1;
}

void Motor_TurnL(){//左转
	Set_Pwmm(200,200,200,200);
	LAIN1 = 0;//左前
	LAIN2 = 1;
	LAIN3 = 0;//左后
	LAIN4 = 1;
	RAIN1 = 0;//右前
	RAIN2 = 1;
	RAIN3 = 1;//右后
	RAIN4 = 0;
}



void Motor_XIE(int dir,int ang){//斜着走↖↙↗↘
	switch (dir){
		case 1:{//↖
			LAIN1 = 1;//左前
			LAIN2 = 0;
			LAIN3 = 1;//左后
			LAIN4 = 0;
			RAIN1 = 0;//右前
			RAIN2 = 1;
			RAIN3 = 1;//右后
			RAIN4 = 0;
			leftSpeedSet = 2000;
			rightSpeedSet = 2000;
			Timer3_PWM_SetDutyCycle(1,800);//左后
			Timer3_PWM_SetDutyCycle(2,0 + ang);//左前
			Timer3_PWM_SetDutyCycle(3,800);//右前
			Timer3_PWM_SetDutyCycle(4,0 + ang);//右后
			break;
			
		}
		case 2:{//↙
			LAIN1 = 1;//左前
			LAIN2 = 0;
			LAIN3 = 1;//左后
			LAIN4 = 0;
			RAIN1 = 0;//右前
			RAIN2 = 1;
			RAIN3 = 1;//右后
			RAIN4 = 0;
			
			Timer3_PWM_SetDutyCycle(1,0 + ang);//左后
			Timer3_PWM_SetDutyCycle(2,800);//左前
			Timer3_PWM_SetDutyCycle(3,0 + ang);//右前
			Timer3_PWM_SetDutyCycle(4,800);//右后
			break;
		}
		case 3:{//↗
			LAIN1 = 0;//左前
			LAIN2 = 1;
			LAIN3 = 0;//左后
			LAIN4 = 1;
			RAIN1 = 1;//右前
			RAIN2 = 0;
			RAIN3 = 0;//右后
			RAIN4 = 1;
			leftSpeedSet2 = 2000;
			rightSpeedSet2 = 2000;
			Timer3_PWM_SetDutyCycle(1,0 + ang);//左后
			Timer3_PWM_SetDutyCycle(2,800);//左前
			Timer3_PWM_SetDutyCycle(3,0 + ang);//右前
			Timer3_PWM_SetDutyCycle(4,800);//右后
			break;
		}
		case 4:{//↘
			LAIN1 = 0;//左前
			LAIN2 = 1;
			LAIN3 = 0;//左后
			LAIN4 = 1;
			RAIN1 = 1;//右前
			RAIN2 = 0;
			RAIN3 = 0;//右后
			RAIN4 = 1;
			
			Timer3_PWM_SetDutyCycle(1,800);//左后
			Timer3_PWM_SetDutyCycle(2,0 + ang);//左前
			Timer3_PWM_SetDutyCycle(3,800);//右前
			Timer3_PWM_SetDutyCycle(4,0 + ang);//右后
			break;
		}
	}
}

//PWM

void Timer3_PWM_SetDutyCycle(u8 ch, u16 Timer3_PWM_DutyCycle){

    switch (ch)
    {
    case 2:
        TIM_SetCompare1(TIM3, Timer3_PWM_DutyCycle );//通道1占空比//左后-左前b
        break;
    case 1:
        TIM_SetCompare2(TIM3, Timer3_PWM_DutyCycle );//通道2占空比//左前d
        break;
    case 3:
        TIM_SetCompare3(TIM3, Timer3_PWM_DutyCycle );//通道3占空比//右前a
        break;
    case 4:
        TIM_SetCompare4(TIM3, Timer3_PWM_DutyCycle );//通道4占空比//右后c
        break;	                                                                                                                                                                                                                                                                                                                                                              
    }
}

void Set_Pwmm(int left,int left2,int right,int right2){//B,D,A,C

		Timer3_PWM_SetDutyCycle(1,myabs(left));//左后
		Timer3_PWM_SetDutyCycle(2,myabs(left2));//左前
	  Timer3_PWM_SetDutyCycle(3,myabs(right));//右前
		Timer3_PWM_SetDutyCycle(4,myabs(right2));//右后
		
}
int myabs(int a){//绝对值函数 		   
	int temp;
	if(a<0)  
	  temp=-a;  
	else 
	  temp=a;
	return temp;
}

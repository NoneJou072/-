#include "timer.h"
#include "led.h"
#include "usart.h"

int temp=0;
int t2=0;
void Pwmin_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;//左后左前
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;//右前右后
    GPIO_Init(GPIOB, &GPIO_InitStructure);
 
}

//定时器6初始化配置
void TIM6_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef tim;//结构体
	NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);//开启时钟
     
	tim.TIM_ClockDivision=TIM_CKD_DIV1;//采样分频
  tim.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
  tim.TIM_Period=arr;//自动重装载寄存器的值
  tim.TIM_Prescaler=psc;//时钟预分频
  TIM_TimeBaseInit(TIM6,&tim);//初始化结构体
     
  TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
  
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM6,ENABLE);//开启时钟
     
}
//定时器7初始化配置
void TIM7_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef tim;//结构体
	NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);//开启时钟
     
	tim.TIM_ClockDivision=TIM_CKD_DIV1;//采样分频
  tim.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
  tim.TIM_Period=arr;//自动重装载寄存器的值
  tim.TIM_Prescaler=psc;//时钟预分频
  TIM_TimeBaseInit(TIM7,&tim);//初始化结构体
     
  TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
  
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM7,ENABLE);//开启时钟
     
}
void TIM3_PWM_Init(u16 arr,u16 psc)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	TIM_OCInitTypeDef TIM_OCInitTypeStrue;
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);
	//定时器配置
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;//0//设置时钟分割
	TIM_TimeBaseInitStrue.TIM_Period= arr;//1000//设置计数溢出大小，每计1000个数就产生一个更新事件
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//设置计数器模式为向上计数模式
	TIM_TimeBaseInitStrue.TIM_Prescaler= psc;//72//设置72分频
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStrue); //将配置应用到TIM3中
	
	//PWM配置CH1
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM2;//配置为PWM模式2
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCPolarity_Low;//有效电平为低电平
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable;//比较输出使能
	TIM_OCInitTypeStrue.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OC1Init(TIM3, &TIM_OCInitTypeStrue);//使能通道1
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
		//PWM配置CH2
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable;//比较输出使能
	TIM_OCInitTypeStrue.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OC2Init(TIM3, &TIM_OCInitTypeStrue);//使能通道2
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
		//PWM配置CH3
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable;//比较输出使能
	TIM_OCInitTypeStrue.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OC3Init(TIM3, &TIM_OCInitTypeStrue);//使能通道3
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
		//PWM配置CH4
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable;//比较输出使能
	TIM_OCInitTypeStrue.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OC4Init(TIM3, &TIM_OCInitTypeStrue);//使能通道4
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM3, ENABLE);//使能TIM3重载寄存器ARR
	TIM_Cmd(TIM3,ENABLE);
}



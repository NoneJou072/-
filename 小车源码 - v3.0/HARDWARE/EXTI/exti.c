#include "exti.h"
#include "led.h"
#include "delay.h"
#include "usart.h"

void EXTIX_Init(void)
{
		
   	EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟

   //GPIOB.3	  中断线以及中断初始化配置 下降沿触发
//  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource2);
//  	EXTI_InitStructure.EXTI_Line=EXTI_Line2;
//  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
//  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//		EXTI_InitStructure.EXTI_LineCmd = ENABLE;	//使能中断线
//  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource5);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line5;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;	//使能中断线
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource3);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line3;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;	//使能中断线
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
		
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource0);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;	//使能中断线
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
		
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource1);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;	//使能中断线
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

//  	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//抢占优先级2 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					//子优先级1 
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
//  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;					//子优先级1 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;					//子优先级1 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;					//子优先级1 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;					//子优先级1 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

}

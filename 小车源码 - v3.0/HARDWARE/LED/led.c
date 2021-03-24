#include "led.h"

void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);	 //使能PA.B.C端口时钟
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;	//PB12.13.14.15   
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz	
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出	
 GPIO_Init(GPIOB, &GPIO_InitStructure);	  				 
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8;	//PA4.5.8
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz	
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出	
 GPIO_Init(GPIOA, &GPIO_InitStructure);	  		
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	//PC4	
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz	
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出	
 GPIO_Init(GPIOC, &GPIO_InitStructure);	  	
}
 
void hongwai(void)//初始化红外传感器io口
{
	 GPIO_InitTypeDef  GPIO_InitStructure;
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能PC端口时钟
	 GPIO_PinRemapConfig(GPIO_Remap_PD01,ENABLE);/*映射PD01的使能*/ 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 |  GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_1 | GPIO_Pin_3;	    //
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;  //浮空输入
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
}


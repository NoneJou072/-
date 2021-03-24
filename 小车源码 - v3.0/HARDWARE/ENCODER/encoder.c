#include "encoder.h"
#include "led.h"
#include "usart.h"
#include "motor.h"

/**************************************************************************
函数功能：把TIM8初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void TIM8_Encoder_Config(void)//PC6.7 motorC
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;      

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; 
	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);                           


    TIM_DeInit(TIM8);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; 
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);              

    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10; 
    TIM_ICInit(TIM8, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM8, TIM_FLAG_Update);
    TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
		//Reset counter
		TIM_SetCounter(TIM8,0);
    TIM8->CNT = 0x7fff;
    TIM_Cmd(TIM8, ENABLE);
}
/**************************************************************************
函数功能：把TIM2初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void TIM2_Encoder_Config(void)//PB3.PA15 motorB
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    //使能定时器2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);   //使能PA端口时钟
	//重映射必须要开AFIO时钟x
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//禁用JTAG 启用 SWD
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//根据设定参数初始化GPIOA
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//根据设定参数初始化GPIOB

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;              //预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);                   //清除TIM的更新标志位
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM2,0);
	//===============================================
	TIM2->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM2, ENABLE); 
}
/**************************************************************************
函数功能：把TIM4初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/

void TIM4_Encoder_Config(void)//PB6.7 motorD
{

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    //使能定时器4的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //使能PB端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//根据设定参数初始化GPIOB

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;              // 预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);                   //清除TIM的更新标志位
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM4,0);
	//===============================================
	TIM4->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM4, ENABLE); 
}
/**************************************************************************
函数功能：把TIM5初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/

void TIM5_Encoder_Config(void)//PA01  motorA,右前
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;      

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                           


    TIM_DeInit(TIM5);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; 
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);              

    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10; 
    TIM_ICInit(TIM5, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM5, TIM_FLAG_Update);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
		
		//Reset counter
		TIM_SetCounter(TIM5,0);
    TIM5->CNT = 0x7fff;
    TIM_Cmd(TIM5, ENABLE);
}


/**************************************************************************
函数功能：读取编码器脉冲差值
入口参数：TIM_TypeDef * TIMx
返回  值：无
**************************************************************************/

s16 getTIMx_DetaCnt(TIM_TypeDef * TIMx)
{
	s16 cnt;
	cnt = TIMx->CNT-0x7fff;
	TIMx->CNT = 0x7fff;
	if(TIMx == TIM2){
		cntCount1 += cnt;
	}else if(TIMx == TIM4){
		cntCount2 += cnt;
	}else if(TIMx == TIM5){
		cntCount3 += cnt;
	}else if(TIMx == TIM8){
		cntCount4 += cnt;
	}
	cntCountNew = (myabs(cntCount1) + myabs(cntCount2) + myabs(cntCount3) + myabs(cntCount4)) / 40;
	return cnt;
}

/**************************************************************************
函数功能：计算左右轮速
入口参数：int *leftSpeed,int *rightSpeed
返回  值：无
		//计算左右车轮线速度，正向速度为正值 ，反向速度为负值，速度为乘以1000之后的速度 mm/s
		//一定时间内的编码器变化值*转化率（转化为直线上的距离m）*200s（5ms计算一次） 得到 m/s *1000转化为int数据

		一圈的脉冲数：
			左：1560
			右：1040
		轮子半径：0.03m
		轮子周长：2*pi*r
		一个脉冲的距离：
			左：0.000120830m
			右：0.000181245m
		速度分辨率：
			左：0.0120m/s 12.0mm/s
			右：0.0181m/s 18.1mm/s
**************************************************************************/



void Get_Motor_Speed(int *leftSpeed,int *leftSpeed2,int *rightSpeed,int *rightSpeed2)
{
	//5ms测速 5ms即这里说的单位时间  	
	*leftSpeed   = getTIMx_DetaCnt(TIM4) * 200;  //D
	*rightSpeed  = getTIMx_DetaCnt(TIM5) * 200;//A
	*leftSpeed2 = getTIMx_DetaCnt(TIM2) * 200;//B
	*rightSpeed2 = getTIMx_DetaCnt(TIM8) * 200;//C
	
}

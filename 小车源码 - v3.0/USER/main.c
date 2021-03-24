#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "exti.h"
#include "encoder.h"
#include "motor.h"
#include "string.h"
#include "usart5.h"
#include "pid.h"
#include "JY61.h"
#include "DIO.h"
#include "UART2.h"
#include <stdio.h>

struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;

void TIM2_Int_Init(u16 arr,u16 psc);
short MPU6050_DATE(void);
void Pwmin_init(void);
void codePrint(int pcode1,int pcode2);
int colPick(int runs);

extern u16 Resnum[100];
int roadtime;//路径长度
u8 mpucon,codescan,resi,aftercode;
float p_average;
char res2[2]="",codeget[10]="";
uint8_t OpenMV_Rx_BUF[8]={0},openmv_blobcol[8]={0},openmv_cir[8]={0},cirsend[3]={0};//存储二维码，颜色顺序，色环,
int devide_code[3]={0};//转义二维码
int openmvi,openmvj;//openmv执行步数,openmv数组位序
int depcode1,depcode2;
int arduino_code[3]={-1,-1,-1};
u8 rx4,rx4_2,cirstep,cirget;
int leftSpeedNow, leftSpeedNow2, rightSpeedNow, rightSpeedNow2;//当前轮速
int leftSpeedSet, rightSpeedSet, leftSpeedSet2, rightSpeedSet2;//mm/s//速度设定值
int motorLeft, motorLeft2, motorRight,motorRight2;//PWM
int cirCarMovingStep;
int ardi,jxbreceive,cirnum;
int cntCount, cntCount1, cntCount2, cntCount3, cntCount4, cntCountNew, cntSet = 280;//脉冲记数
char *ard_char_cir;
int a,mpu_begin;
float mpu_u = 100;
int SpeedSet = 2600;

void sendcmd(char cmd[]){//用串口2给JY模块发送指令
	char i;
	for(i=0;i<3;i++)
		UART2_Put_Char(cmd[i]);
}


void CopeSerial2Data(unsigned char ucData){//CopeSerialData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数

	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;

		}
		ucRxCnt=0;//清空缓存区
	}
}

void CopeSerial1Data(unsigned char ucData){	
	UART2_Put_Char(ucData);//转发串口1收到的数据给串口2（JY模块）
}

char *reverse(char *s){//反转字符串
    char temp;
    char *p = s;    //p指向s的头部
    char *q = s;    //q指向s的尾部
    while(*q)
        ++q;
    q--;

    //交换移动指针，直到p和q交叉
    while(q > p)
    {
        temp = *p;
        *p++ = *q;
        *q-- = temp;
    }
    return s;
}
char *my_itoa(int n){//功能：整数转换为字符串
    int i = 0,isNegative = 0;
    static char s[100];      //必须为static变量，或者是全局变量
    if((isNegative = n) < 0) //如果是负数，先转为正数
    {
        n = -n;
    }
    do      //从各位开始变为字符，直到最高位，最后应该反转
    {
        s[i++] = n%10 + '0';
        n = n/10;
    }while(n > 0);

    if(isNegative < 0)   //如果是负数，补上负号
    {
        s[i++] = '-';
    }
    s[i] = '\0';    //最后加上字符串结束符
    return reverse(s);
}
void Xunji_Choice_SandB(){//前后
		EXTI->IMR &= ~(EXTI_Line0); //屏蔽外部中断0
		EXTI->IMR &= ~(EXTI_Line1); //屏蔽外部中断1
		EXTI->IMR |= EXTI_Line5; //使能外部中断5
		EXTI->IMR |= EXTI_Line3; //使能外部中断10
}

void Xunji_Choice_LandR(){//左右
		EXTI->IMR &= ~(EXTI_Line5); //屏蔽外部中断5
		EXTI->IMR &= ~(EXTI_Line3); //屏蔽外部中断3
		EXTI->IMR |= EXTI_Line0; //使能外部中断0
		EXTI->IMR |= EXTI_Line1; //使能外部中断1
}

void IQR_STOP(){//屏蔽全部
	EXTI->IMR &= ~(EXTI_Line1); //屏蔽外部中断5
	EXTI->IMR &= ~(EXTI_Line0); //屏蔽外部中断0
	EXTI->IMR &= ~(EXTI_Line5); //屏蔽外部中断5
	EXTI->IMR &= ~(EXTI_Line3); //屏蔽外部中断10
}
//pd闭环控制
void TIM7_IRQHandler(void){//电机测速
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{	
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);//清除更新中断标志位
		
		Get_Motor_Speed(&leftSpeedNow,&leftSpeedNow2,&rightSpeedNow,&rightSpeedNow2);//dcab->bdac
	}
}
void pidcon(){
		//给速度设定值，想修改速度，就更该leftSpeeSet、rightSpeedSet变量的值
		pid_Task_Letf.speedSet  = myabs(leftSpeedSet);
		pid_Task_Right.speedSet = myabs(rightSpeedSet);
		pid_Task_Letf2.speedSet  = myabs(leftSpeedSet2);
		pid_Task_Right2.speedSet = myabs(rightSpeedSet2);
		//给定速度实时值
		pid_Task_Letf.speedNow  = myabs(leftSpeedNow);
		pid_Task_Right.speedNow = myabs(rightSpeedNow);
		pid_Task_Letf2.speedNow  = myabs(leftSpeedNow2);
		pid_Task_Right2.speedNow = myabs(rightSpeedNow2);
		//执行PID控制函数
		Pid_Ctrl(&motorLeft,&motorRight,&motorLeft2,&motorRight2);
		delay_ms(2);
} 
void pid_others(){
		if(leftSpeedSet || leftSpeedSet2 || rightSpeedSet || rightSpeedSet2){
			pidcon();
			if(!leftSpeedSet)Timer3_PWM_SetDutyCycle(1,0),motorLeft = 0;
			if(!leftSpeedSet2)Timer3_PWM_SetDutyCycle(2,0),motorLeft2 = 0;
			if(!rightSpeedSet)Timer3_PWM_SetDutyCycle(3,0),motorRight = 0;
			if(!rightSpeedSet2)Timer3_PWM_SetDutyCycle(4,0),motorRight2 = 0;
		}else {
			if(p_average>0.8){
				if(roadtime==1){
					if(mpucon==1)Motor_TurnL();
					else Motor_TurnR();
				}
				
			}else{
				Set_Pwmm(0,0,0,0);
				motorLeft = 0;
				motorLeft2 = 0;
				motorRight = 0;
				motorRight2 = 0;
				cntCount1 = cntCount2 = cntCount3 = cntCount4 = 0;
			}
		 }
		if((cntCountNew>(cntSet/1.5))){
			if(leftSpeedSet>500)leftSpeedSet-=20;
			if(leftSpeedSet2>500)leftSpeedSet2-=20;
			if(rightSpeedSet>500)rightSpeedSet-=20;
			if(rightSpeedSet2>500)rightSpeedSet2-=20;
		}
}
//gpu串口屏
void GpuSend(char * buf1){  //串口屏数据发送函数	
	u8 i=0;
 while (1){  
	 if (buf1[i]!=0){  
		 USART_SendData(USART1, buf1[i]);  //发送一个byte到串口
		//delay_ms(20);
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}; //等待发送结束
       i++;		
	}else return;
}
}

void num(void){ //串口屏内容
	codePrint(OpenMV_Rx_BUF[2],OpenMV_Rx_BUF[3]);
	printf("DS48(0,60,'col:%d:%d:%d',16,0);\r\n",openmv_blobcol[2],openmv_blobcol[3],openmv_blobcol[4]);
	printf("DS48(10,10,'speed:%d,%d,%d,%d',16,0);\r\n",leftSpeedNow,leftSpeedNow2,rightSpeedNow,rightSpeedNow2);
	printf("DS32(0,100,'%d+%d',16,0);\r\n",depcode1,depcode2);
	printf("DS48(10,40,'arduino:%d,%d,%d',16,0);\r\n",arduino_code[0],arduino_code[1],arduino_code[2]);
	//printf("DS48(10,25,'pwm:%d:%d:%d:%d',16,0);\r\n",motorLeft,motorLeft2,motorRight,motorRight2);
	printf("DS48(10,25,'pwm:%d:%d:%d:%d',16,0);\r\n",leftSpeedSet,leftSpeedSet2,rightSpeedSet,rightSpeedSet2);
	//printf("DS48(0,25,'cl:%d:%d:%d',16,0);\r\n",openmv_cir[2],openmv_cir[3],openmv_cir[4]);
	//printf("DS48(0,40,'charcir:%c',16,0);\r\n",*ard_char_cir);
	//printf("DS48(0,25,'cn:%d:%d:%d',16,0);\r\n",cirsend[0],cirsend[1],cirsend[2]);
	printf("DS48(0,80,'row:%.2f',16,0);\r\n",p_average);
	
}
//openmv摄像头
void cir(){//色环接收后动作
	/*如：任务码213，色环132，则返回312作为动作顺序*/
	int j=0,z=0;
	for(j=0;j<3;j++)
		for(z=0;z<3;z++){
			if(openmv_cir[j+2]==devide_code[z])
				cirsend[j]=z+1;
			}
	ard_char_cir = my_itoa(cirsend[2]);
	//如果颜色无差，执行接下来的动作
	//if(cirsend[0] && cirsend[1] && cirsend[2] && cirsend[0]!=cirsend[1]!=cirsend[2]){
		UART4SendByte(*my_itoa(cirsend[0])); //第一次发送
		delay_ms(20);
		UART4SendByte(*my_itoa(cirsend[1])); //第二次发送
		delay_ms(20);
		UART4SendByte(*my_itoa(cirsend[2])); //开始放下物料
		//}
}
void codePrint(int pcode1,int pcode2){//二维码解密
	if(pcode1==0x01)depcode1=123;
	else if (pcode1==0x02)depcode1=132;
	else if (pcode1==0x03)depcode1=213;
	else if (pcode1==0x04)depcode1=231;
	else if (pcode1==0x05)depcode1=312;
	else if (pcode1==0x06)depcode1=321;
	
	if(pcode2==0x01)depcode2=123;
	else if (pcode2==0x02)depcode2=132;
	else if (pcode2==0x03)depcode2=213;
	else if (pcode2==0x04)depcode2=231;
	else if (pcode2==0x05)depcode2=312;
	else if (pcode2==0x06)depcode2=321;
	
}
void openmv_code(){//扫码后动作
		if(OpenMV_Rx_BUF[2] && OpenMV_Rx_BUF[3])
			codescan=1;
		if(codescan && !aftercode && depcode1 && depcode2){
			Motor_Straight();
			cntCountNew = cntCount1 = cntCount2 = cntCount3 = cntCount4 = 0;
			codescan=2;
			aftercode=1;
		 }
}

void USART2_IRQHandler(void){//openmv串口中断
	
	static uint8_t rebuf[8]={0},i=0;
	
	if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET)
	{
		USART_ClearFlag(USART2,USART_FLAG_RXNE);
		rebuf[i++]=USART_ReceiveData(USART2);	
		if(rebuf[0]!=0xb3)//帧头
			i=0;
	  if((i==2)&&(rebuf[1]!=0xb3))//判断帧头
			i=0;
		if(i>=7)//代表一帧数据完毕
		{
			if(openmvi==0){//二维码
				memcpy(OpenMV_Rx_BUF,rebuf,i);
			}else if(openmvi==1){//物料
				memcpy(openmv_blobcol,rebuf,i);
				//colPick(0);
				while(colPick(0));
				openmvi=-1;//只接受一次
				UART1SendByte2(0x35);
				UART4SendByte(0x33);//机械臂复位
				delay_s(1);
				Motor_TRIGHT();//识别完成后向右走
				mpu_begin = 0;
				cntSet = 240;
			}else if(openmvi==2){//色环
				memcpy(openmv_cir,rebuf,i);
				cirget = 1;
				cir();
				openmvi=-1;//只接受一次
			}
			i = 0;
		}
		
	}	
}
//arduino机械臂
int colPick(int runs){//机械臂抓取顺序计算
	int j=0,z=0;
	int col_code[3]={0};
	for(j=0;j<3;j++){
		col_code[j]=openmv_blobcol[j+2];
	}
	switch(runs){//二维码解析
		case 0:{
			devide_code[0]=depcode1/100;
			devide_code[1]=depcode1/10%10;
			devide_code[2]=depcode1%10;
		break;
		}
		case 1:{
			devide_code[0]=depcode2/100;
			devide_code[1]=depcode2/10%10;
			devide_code[2]=depcode2%10;
		break;
		}
	}
	for(j=0;j<3;j++){//按序排列
		for(z=0;z<3;z++){
			if(devide_code[j]==col_code[z])
				arduino_code[j]=z+1;
		}
	}
	if(arduino_code[0] && arduino_code[1] && arduino_code[2]){//判断数据是否正常
	return 0;
	}else return 1;
}

void UART4_IRQHandler(void){ //arduino串口中断
    u8 res4;
    if(USART_GetITStatus(UART4,USART_IT_RXNE)!=RESET)
    {
        USART_ClearITPendingBit(UART4,USART_IT_RXNE);
        res4=USART_ReceiveData(UART4);
        //if(res4 == 0x39) rx4 = 0;
				if(res4 == 0x38){ 
					if(rx4==0){//抓取上层物料，放入车内
						delay_s(1);
						for(ardi=0;ardi<3;ardi++){
							char *ard_char = my_itoa(arduino_code[ardi]);
							UART4SendByte(*ard_char); 
							delay_ms(50);
							jxbreceive++;
						}
					}else if(rx4==3){//物块拾取完毕，小车启动
						Motor_Straight();
						while(!hongwai5);
						while(hongwai5);
						Xunji_Choice_LandR();
						Motor_TLEFT();
						cntCountNew = cntCount1 = cntCount2 = cntCount3 = cntCount4 = 0;
					}else if(rx4==6){//物料拾取完毕，小车启动
						Motor_XIE(1,0);
					}else if(rx4==9){
						Motor_TRIGHT();
						while(!hongwai3 && !hongwai4);//陀螺仪校准
						Motor_Stop();
						if(hongwai3){
							Motor_TurnR();
						}else if(hongwai4){
							Motor_TurnL();
						}
						while(!hongwai3 || !hongwai4);
						Motor_Stop();
						if(mpucon==1)mpu_u = p_average;
						else mpu_u = p_average * (-1);
						Motor_Back();
					}	
					rx4++;	
				}
    }
}
//路径及动作

void EXTI9_5_IRQHandler(void){//
	while(hongwai2 && !hongwai1){
		if(roadtime==1 || roadtime==5)Set_Pwmm(300,200,800,300);;	//正走寻迹
		if(roadtime==10)Set_Pwmm(200,800,200,800);	//倒退寻迹
	}
	EXTI_ClearITPendingBit(EXTI_Line5);  //清除LINE5上的中断标志位 
}
void EXTI3_IRQHandler(void){//
	while(hongwai1 && !hongwai2){
		if(roadtime==1 || roadtime==5)Set_Pwmm(300,800,200,300);//正走寻迹,STR=1,BACK=0
		if(roadtime==10)Set_Pwmm(800,200,800,200);//倒退寻迹
	}
	EXTI_ClearITPendingBit(EXTI_Line3);  //清除LINE10上的中断标志位 
}


void EXTI1_IRQHandler(void){//侧走往左修
	while(hongwai5 && !hongwai4){
		if(roadtime==3 || roadtime==10)Set_Pwmm(600,600,200,700);//左走寻迹
		if(roadtime==4)Set_Pwmm(200,700,600,600);//右走寻迹
	}
	EXTI_ClearITPendingBit(EXTI_Line1);  //清除LINE0上的中断标志位
}


void EXTI0_IRQHandler(void){//侧走往右修
	while(hongwai4 && !hongwai5){
		if(roadtime==3 || roadtime==10)Set_Pwmm(600,600,700,200);//左走寻迹
		if(roadtime==4)Set_Pwmm(700,200,600,600);//右走寻迹
	}
	EXTI_ClearITPendingBit(EXTI_Line0);  //清除LINE0上的中断标志位 
}
void goon(){
		if(cntCountNew >= cntSet){
			IQR_STOP();
			cntCountNew = cntCount1 = cntCount2 = cntCount3 = cntCount4 = 0;
			if(roadtime==0){ //前->扫码
				Motor_Straight();
				UART1SendByte2(0x32);//向串口2发送2，准备扫码
				while(cntCountNew<40);
				Motor_Stop();
				cntSet = 620;
				delay_s(1);
			}else if(roadtime==1){//扫码->识别
				Motor_Stop();
				Motor_TLEFT();
				while(cntCountNew<140);
				while(!hongwai1);
				Motor_Stop();
				openmvi=1;
				delay_s(1);
				UART1SendByte2('3');//开始识别颜色顺序
				
			}else if(roadtime==2){//识别->取物
				mpu_begin = 1;
				while(!hongwai2);
				LAIN1 = !LAIN1;//左前
				LAIN2 = !LAIN2;
				LAIN3 = !LAIN3;//左后
				LAIN4 = !LAIN4;
				RAIN1 = !RAIN1;//右后
				RAIN2 = !RAIN2;
				RAIN3 = !RAIN3;//右前
				RAIN4 = !RAIN4;
				delay_ms(50);
				Motor_Stop();
				//开始抓取物料
				cntSet = 520;
			}else if(roadtime==3){//左->放物取物
				while(!hongwai1);
				LAIN1 = !LAIN1;//左前
				LAIN2 = !LAIN2;
				LAIN3 = !LAIN3;//左后
				LAIN4 = !LAIN4;
				RAIN1 = !RAIN1;//右后
				RAIN2 = !RAIN2;
				RAIN3 = !RAIN3;//右前
				RAIN4 = !RAIN4;
				delay_ms(50);
				Motor_Stop();
				delay_ms(200);
				cntCountNew = cntCount1 = cntCount2 = cntCount3 = cntCount4 = 0;
				Motor_Straight();
				while(cntCountNew<75);
				mpu_begin = 1;
				UART4SendByte('5');//执行识别色环顺序的机械臂动作
				Motor_Stop();
				openmvi=2;
				delay_s(1);
				UART1SendByte2('4');//向openmv发送4,开始识别色环
				cntSet = 525;
			}else if(roadtime==4){//↖->放物	
				mpu_begin = 0;
				Motor_Stop();
				UART4SendByte('7');//成品区放置物料
				cntSet = 555;
			}else if(roadtime==5){//回家
				Motor_Stop();		
				Motor_TLEFT();
				while(cntCountNew<190);
				Motor_Stop();
			}
			cntCount1 = cntCount2 = cntCount3 = cntCount4 = 0;
			roadtime++;
		}
}
//陀螺仪
void TRACING_CALIBRATION_ROADING(int direction){//占空比调整，UP=1,BACK=2,LEFT=3,RIGHT=4
	switch (direction)
	{
		case 1:{
			if(mpucon==1)
				Set_Pwmm(motorLeft + p_average*70,motorLeft2 - p_average*70,motorRight,motorRight2);//BDAC左前左后右前右后
			else if(mpucon==2)
				Set_Pwmm(motorLeft - p_average*70,motorLeft2 + p_average*70,motorRight,motorRight2);//BDAC左前左后右前右后
			break;
		}
		case 2:{
			if(mpucon==1)
				Set_Pwmm(motorLeft,motorLeft2,motorRight - p_average*70,motorRight2 + p_average*70);
			else if(mpucon==2)
				Set_Pwmm(motorLeft,motorLeft2,motorRight + p_average*70,motorRight2 - p_average*70);
			break;
		}
		case 3:{
			if(mpucon==1)
				Set_Pwmm(motorLeft + p_average*100,motorLeft2 - p_average*100,motorRight + p_average*100,motorRight2 - p_average*100);//BDAC左前左后右前右后
		  else if(mpucon==2)
				Set_Pwmm(motorLeft - p_average*100,motorLeft2 + p_average*100,motorRight - p_average*100,motorRight2 + p_average*100);//BDAC左前左后右前右后
			break;
		}
		case 4:{
			if(mpucon==1){
				if(leftSpeedSet + p_average * mpu_u < 3000)leftSpeedSet = SpeedSet + p_average * mpu_u;
				if(leftSpeedSet2 - p_average * mpu_u > 100)leftSpeedSet2 = SpeedSet - p_average * mpu_u;
				if(rightSpeedSet - p_average * mpu_u > 100)rightSpeedSet = SpeedSet - p_average * mpu_u;
				if(rightSpeedSet2 + p_average * mpu_u < 3000)rightSpeedSet2 = SpeedSet + p_average * mpu_u;
				Set_Pwmm(motorLeft,motorLeft2,motorRight,motorRight2);//BDAC左前左后右前右后
			}
			else if(mpucon==2){
				if(leftSpeedSet - p_average * mpu_u > 100)leftSpeedSet = SpeedSet - p_average * mpu_u;
				if(leftSpeedSet2 + p_average * mpu_u < 3000)leftSpeedSet2 = SpeedSet + p_average * mpu_u;
				if(rightSpeedSet + p_average * mpu_u < 3000)rightSpeedSet = SpeedSet + p_average * mpu_u;
				if(rightSpeedSet2 - p_average * mpu_u > 100)rightSpeedSet2 = SpeedSet - p_average * mpu_u;
				Set_Pwmm(motorLeft,motorLeft2,motorRight,motorRight2);//BDAC左前左后右前右后
			}
			break;
		}
		case 5:
		{
			if(mpucon==1)
				Set_Pwmm(motorLeft - p_average*50,motorLeft2,motorRight + p_average*50,motorRight2);//BDAC左前左后右前右后
			else if(mpucon==2)
				Set_Pwmm(motorLeft + p_average*50,motorLeft2,motorRight - p_average*50,motorRight2);//BDAC左前左后右前右后
			break;
		}
		case 6:
		{
			if(mpucon==1)
			{
			Timer3_PWM_SetDutyCycle(2,800-p_average*30);//左前
			Timer3_PWM_SetDutyCycle(4,800+p_average*30);//右后
			}
			else if(mpucon==2)
			{
			Timer3_PWM_SetDutyCycle(2,800+p_average*30);//左前
			Timer3_PWM_SetDutyCycle(4,800-p_average*30);//右后
			}
			break;
		}
		case 7:
		{
			if(mpucon==1)
				Set_Pwmm(motorLeft,motorLeft2 - p_average*50,motorRight,motorRight2 + p_average*50);//BDAC左前左后右前右后
			else if(mpucon==2)
				Set_Pwmm(motorLeft,motorLeft2 + p_average*50,motorRight,motorRight2 - p_average*50);//BDAC左前左后右前右后
			break;
		}
		case 8:
		{
			if(mpucon==1)
			{
			Timer3_PWM_SetDutyCycle(1,800+p_average*30);//左后
			Timer3_PWM_SetDutyCycle(3,800-p_average*30);//右前
			}
			else if(mpucon==2)
			{
			Timer3_PWM_SetDutyCycle(1,800-p_average*30);//左后
			Timer3_PWM_SetDutyCycle(3,800+p_average*30);//右前
			}
			break;
		}
	}
}

void TRACING_CALIBRATION_ROADINGON(){//线路调整选择
	if(roadtime==1 || roadtime==5)TRACING_CALIBRATION_ROADING(1);//前进
	else if(roadtime==10)TRACING_CALIBRATION_ROADING(2);//后退
	else if(roadtime==3)TRACING_CALIBRATION_ROADING(3);//左走
	else if(roadtime==2)TRACING_CALIBRATION_ROADING(4);//右走
	else if(roadtime==4)TRACING_CALIBRATION_ROADING(5);//↖
	else if(roadtime==10)TRACING_CALIBRATION_ROADING(6);//↙
	else if(roadtime==0)TRACING_CALIBRATION_ROADING(7);//↗
	else if(roadtime==10)TRACING_CALIBRATION_ROADING(8);//↘
}

//main
int main(void){	
		u8 i=10;
	  ard_char_cir = "0";
		delay_init();//延时初始化 
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置NVIC中断分组2:2位抢占优先级，2位响应优先级 
		Pwmin_init();	
		hongwai();
		delay_ms(10);
		uart_init(115200);
		uart_init2(115200);
		uart_init4(115200);
		delay_ms(20);
		while(i)printf("CLS(0)\r\n"),i--,delay_ms(20);
		TIM3_PWM_Init(1000,72);//899.0
		EXTIX_Init();
		LED_Init(); 
		PID_Init();						//=====PID参数初始化		
		TIM2_Encoder_Config();            //=====初始化编码器1接口
		TIM4_Encoder_Config();            //=====初始化编码器2接口
		TIM5_Encoder_Config();            //=====初始化编码器3接口
		TIM8_Encoder_Config();            //=====初始化编码器4接口
		TIM7_Int_Init(49,7199);//50ms 
		Initial_UART2(115200);//接JY61模块的串口	
		delay_ms(2000);//等待JY61初始化完成
		memset(res2,0x00,sizeof(char)*10);//清空串口缓存区
		IQR_STOP();
		//while(1);
		delay_s(4);
		UART4SendByte('2');
		Motor_XIE(3,0);
	 while(1)
	 {	
		 
		 
		 a++;
		if(a>50){
			printf("CLS(0);\r\n");
			a=0;
			//功能现象，10秒钟左右会进行一次加速度校准,Z轴角度归零，XYZ角度会缓慢回到0度状态
		//printf("正在进行加速度校准\r\n");
		sendcmd(ACCCMD);//等待模块内部自动校准好，模块内部会自动计算需要一定的时间
		//printf("加速度校准完成\r\n");
		delay_ms(100);
		//printf("进行Z轴角度清零\r\n");
		sendcmd(YAWCMD);
		//printf("Z轴角度清零完成\r\n");
		}
		
		p_average = (float)stcAngle.Angle[2]/32768*180;//取整数
		if(p_average<0){
			p_average = -p_average;
			mpucon = 1;
		}else mpucon = 2;
		
		 goon();
		 pid_others();
		 num();
		 openmv_code();
		TRACING_CALIBRATION_ROADINGON();
	 }
}

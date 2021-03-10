#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "exti.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "string.h"
#include "usart5.h"
#include "pid.h"
#include "stdlib.h"
#include "stdio.h"

void TIM2_Int_Init(u16 arr,u16 psc);
void MPU6050_DATE(void);
void IQR_STOP(void);
void Pwmin_init(void);
void Xunji_Choice_SandB(void);
void Xunji_Choice_LandR(void);
void codePrint(int pcode1,int pcode2);
int colPick(int runs);
char *reverse(char *s);
char *my_itoa(int n);
void extiToLoad(void);

extern u16 Resnum[100];
int roadtime;//路径长度
u8 mpucon,cls,codescan,resi,aftercode;
u16 p_num;
u16 mpuangle[10]={0,0,0,0,0};
short temp0,p,d;
float p_time,p_average;
char res2[2]="";
char codeget[10]="";
uint8_t OpenMV_Rx_BUF[8]={0},openmv_blobcol[8]={0},openmv_cir[8]={0},cirsend[3]={0};//存储二维码，颜色顺序，色环,
int devide_code[3]={0};//转义二维码
int openmvi,openmvj;//openmv执行步数,openmv数组位序
int depcode1,depcode2;
int arduino_code[3]={-1,-1,-1};
u8 rx4,rx4_2,cirstep,cirget;
int leftSpeedNow, leftSpeedNow2, rightSpeedNow, rightSpeedNow2;//当前轮速
int leftSpeedSet, rightSpeedSet, leftSpeedSet2, rightSpeedSet2;//mm/s//速度设定值
int motorLeft, motorLeft2, motorRight,motorRight2;//PWM
int mpu_u;
int line0,line3,line5;//辅助数线
int cirCarMovingStep;
int ardi,jxbreceive,cirnum, cirperform;
u8 res4judge;
int cntCount, cntCount1, cntCount2, cntCount3, cntCount4, cntCountNew, cntSet = 278;//脉冲记数278
char *ard_char_cir;

//闭环速度控制
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
//串口屏
void GpuSend(char * buf1){  //串口屏数据发送函数	
	u8 i=0;
 while (1)
{  if (buf1[i]!=0)
	{  USART_SendData(USART1, buf1[i]);  //发送一个byte到串口
		//delay_ms(20);
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}; //等待发送结束
       i++;
				
	}
	else return;
}
}

void num(void){ //串口屏内容
	codePrint(OpenMV_Rx_BUF[2],OpenMV_Rx_BUF[3]);
	printf("DS48(0,60,'col:%d:%d:%d',16,0);\r\n",openmv_blobcol[2],openmv_blobcol[3],openmv_blobcol[4]);
	//printf("DS48(10,10,'speed:%d:%d:%d:%d cnt:%d',16,0);\r\n",leftSpeedNow,leftSpeedNow2,rightSpeedNow,rightSpeedNow2,cntCountNew);
	printf("DS32(0,100,'%d+%d',16,0);\r\n",depcode1,depcode2);
	printf("DS48(10,40,'arduino:%d,%d,%d',16,0);\r\n",arduino_code[0],arduino_code[1],arduino_code[2]);
	//printf("DS48(10,25,'pwm:%d:%d:%d:%d',16,0);\r\n",motorLeft,motorLeft2,motorRight,motorRight2);
	//printf("DS48(0,40,'cl:%d:%d:%d',16,0);\r\n",openmv_cir[2],openmv_cir[3],openmv_cir[4]);
	//printf("DS48(0,40,'charcir:%c',16,0);\r\n",*ard_char_cir);
	printf("DS48(0,25,'cn:%d:%d:%d',16,0);\r\n",cirsend[0],cirsend[1],cirsend[2]);
}
//openmv
void cir(){//色环接收后动作
	/*执行动作：
	1.主控接收到从左至右的颜色顺序
	2.计算动作的先后执行顺序，传至arduino
	3.任务码的顺序是小车从车上拿起物料的顺序
	4.色环顺序要对应任务码的顺序
	如：任务码213，色环132，则返回312作为动作顺序*/
	int j=0,z=0;
	if(openmvi==2 && cirget && !cirperform){
		for(j=0;j<3;j++)
			for(z=0;z<3;z++){
				if(openmv_cir[j+2]==devide_code[z])
					cirsend[j]=z+1;
			}
		ard_char_cir = my_itoa(cirsend[2]);
		//如果颜色无差，执行接下来的动作，否则重新识别
		//if((openmv_cir[2]==openmv_cir[5]) && (openmv_cir[3]==openmv_cir[6]) && (openmv_cir[4]==openmv_cir[7])){
			if(cirsend[0] && cirsend[1] && cirsend[2] && cirsend[0]!=cirsend[1]!=cirsend[2]){
			cirperform = !cirperform;
			UART4SendByte(*my_itoa(cirsend[0])); //第一次发送
			delay_ms(20);
			UART4SendByte(*my_itoa(cirsend[1])); //第二次发送
			delay_ms(20);
			UART4SendByte(*my_itoa(cirsend[2])); //开始放下物料
		}
		cirget = 0;
	}
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
			UART4SendByte('3');  //程序复位机械臂
			Set_Pwmm(800,800,800,800);
			Xunji_Choice_SandB();//开启外部中断01，关闭5.10
			Motor_Straight();
			
			 codescan=2;
			 aftercode=1;
		 }
}

void USART2_IRQHandler(void){//openmv串口中断
	
	static uint8_t rebuf[8]={0},i=0;
	
	if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET)
	{
		rebuf[i++]=USART_ReceiveData(USART2);	
		if(rebuf[0]!=0xb3)//帧头
			i=0;
	  if((i==2)&&(rebuf[1]!=0xb3))//判断帧头
			i=0;
		if(i>=7)//代表一帧数据完毕
		{
			if(openmvi==0){//二维码
				memcpy(OpenMV_Rx_BUF,rebuf,i);
				//openmvi=-1;
			}else if(openmvi==1){//物料
				memcpy(openmv_blobcol,rebuf,i);
				colPick(0);
				while(colPick(0));
				openmvi=-1;//只接受一次
			}else if(openmvi==2){//色环
				memcpy(openmv_cir,rebuf,i);
				cirget = 1;
				cir();
				//openmvi=-1;//只接受一次
			}
			i = 0;
		}
		USART_ClearFlag(USART2,USART_FLAG_RXNE);
	}	
}


//arduino
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
        if(res4 == 0x39) rx4 = 0;
				else if(res4 == 0x38){ 
					res4judge++;
					if(rx4==0){//抓取上层物料，放入车内
						for(ardi=0;ardi<3;ardi++){
							char *ard_char = my_itoa(arduino_code[ardi]);
							UART4SendByte(*ard_char); 
							delay_ms(50);
							jxbreceive++;
						}
					}
					else if(rx4==3){//物块拾取完毕，小车启动
						cntCountNew = cntCount1 = cntCount2 = cntCount3 = cntCount4 = 0;
						Motor_Straight();
						while(!hongwai5);
						while(hongwai5);
						Motor_Stop();
						delay_ms(200);
						Motor_TLEFT();
						Xunji_Choice_LandR();
					}
					
					else if(rx4==6){//物料拾取完毕，小车启动
						Motor_Back();
						while(!hongwai4);
						while(hongwai4);
						extiToLoad();//调整车身方位
						Motor_XIE(1,0);
					}	
					rx4++;	
				}
    }
 
}
//寻迹
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
		if(roadtime==2 || roadtime==10)Set_Pwmm(600,600,200,700);//左走寻迹
		if(roadtime==4)Set_Pwmm(200,700,600,600);//右走寻迹
	}
	EXTI_ClearITPendingBit(EXTI_Line1);  //清除LINE0上的中断标志位
}


void EXTI0_IRQHandler(void){//侧走往右修
	while(hongwai4 && !hongwai5){
		if(roadtime==2 || roadtime==10)Set_Pwmm(600,600,700,200);//左走寻迹
		if(roadtime==4)Set_Pwmm(700,200,600,600);//右走寻迹
	}
	EXTI_ClearITPendingBit(EXTI_Line0);  //清除LINE0上的中断标志位 
}
//动作
void extiToLoad(){//停止+方向调平
	  Motor_Stop();
		delay_s(1);
				
		MPU6050_DATE();
		while(temp0/10>0){
			MPU6050_DATE();
			if(mpucon==1)Motor_TurnL();
			else if(mpucon==2)Motor_TurnR();
		}	
		Motor_Stop();
		delay_ms(500);
		while(temp0/10>0){
			MPU6050_DATE();
			if(mpucon==1)Motor_TurnL();
			else if(mpucon==2)Motor_TurnR();
		}	
		Motor_Stop();
	  delay_s(1);
		cntCountNew = cntCount1 = cntCount2 = cntCount3 = cntCount4 = 0;
}

void goon(){
		if(cntCountNew >= cntSet){
		  IQR_STOP();
			if(roadtime==0){ //前->扫码
				Motor_Straight();
				while(cntCountNew<318);
				extiToLoad();//调整车身方位
				memset(res2,0x00,sizeof(char)*10);//清空串口缓存区
				codescan=0;
				openmvi=0;
				UART1SendByte2(0x32);//向串口2发送2，准备扫码
				cntSet = 655;
			}else if(roadtime==1){//前->取物
				extiToLoad();//调整车身方位
				Motor_TRIGHT();
				while(cntCountNew<178);
				UART4SendByte('4'); //向arduino发送0x34，执行机械臂识别颜色动作
				extiToLoad();
				openmvi=1;
				UART1SendByte2('3');//向串口2发送3,开始识别颜色顺序
				cntSet = 520;
			}else if(roadtime==2){//左->放物取物
				while(!hongwai1);
				extiToLoad();//调整车身方位
				Motor_Straight();
				while(cntCountNew<110);
				UART4SendByte('5');//执行识别色环顺序的机械臂动作
				extiToLoad();//调整车身方位
				openmvi=2;			
				UART1SendByte2('4');//向openmv发送4,开始识别色环
				cntSet = 400;
			}else if(roadtime==3){//↖->放物
				while(!hongwai4);
				while(hongwai4);
				extiToLoad();//调整车身方位
				Motor_TLEFT();
				Set_Pwmm(800,800,800,800);
				while(!hongwai2);
				while(hongwai2);
				extiToLoad();//调整车身方位
				
				UART4SendByte('7');//成品区放置物料
				while(1);
//				Motor_TRIGHT();
//				delay_ms(500);
//				Xunji_Choice_LandR();
			}else if(roadtime==4){//右->前
				while(!hongwai1);
				while(hongwai1);
				extiToLoad();//调整车身方位
				Xunji_Choice_SandB();
				Motor_Straight();
			}
			else if(roadtime==5){//前->取物
				delay_ms(100);
				while(!hongwai5);
				while(hongwai5);
				extiToLoad();//调整车身方位
			}	
			cntCount1 = cntCount2 = cntCount3 = cntCount4 = 0;
			roadtime++;
		}
}
//陀螺仪
void MPU6050_DATE(void){	//取得角度和方向 
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据	
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			temp0=MPU_Get_Temperature();	//得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据 
			temp0=yaw*10 + mpu_u;		
			if(temp0<0){
				temp0=-temp0;		//转为正数
				mpucon=1;
			}else mpucon=2;
		}
}
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
				Set_Pwmm(motorLeft,motorLeft2,motorRight + p_average*100,motorRight2 - p_average*100);//BDAC左前左后右前右后
		  else if(mpucon==2)
				Set_Pwmm(motorLeft,motorLeft2,motorRight - p_average*100,motorRight2 + p_average*100);//BDAC左前左后右前右后
			break;
		}
		case 4:{
			if(mpucon==1)
				Set_Pwmm(motorLeft - p_average*100,motorLeft2 + p_average*100,motorRight,motorRight2);//BDAC左前左后右前右后
			else if(mpucon==2)
				Set_Pwmm(motorLeft + p_average*100,motorLeft2 - p_average*100,motorRight,motorRight2);//BDAC左前左后右前右后
			break;
		}
		case 5:
		{
			if(mpucon==1)
				Set_Pwmm(motorLeft - p_average*50,0,motorRight + p_average*50,0);//BDAC左前左后右前右后
			else if(mpucon==2)
				Set_Pwmm(motorLeft + p_average*50,0,motorRight - p_average*50,0);//BDAC左前左后右前右后
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
				Set_Pwmm(0,motorLeft2 - p_average*50,0,motorRight2 + p_average*50);//BDAC左前左后右前右后
			else if(mpucon==2)
				Set_Pwmm(0,motorLeft2 + p_average*50,0,motorRight2 - p_average*50);//BDAC左前左后右前右后
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
	if(roadtime==10)TRACING_CALIBRATION_ROADING(2);//后退
	if(roadtime==2)TRACING_CALIBRATION_ROADING(3);//左走
	if(roadtime==4)TRACING_CALIBRATION_ROADING(4);//右走
	if(roadtime==3)TRACING_CALIBRATION_ROADING(5);//↖
	if(roadtime==10)TRACING_CALIBRATION_ROADING(6);//↙
	if(roadtime==0)TRACING_CALIBRATION_ROADING(7);//↗
	if(roadtime==10)TRACING_CALIBRATION_ROADING(8);//↘
}


void TIM6_IRQHandler(void){//定时器中断服务函数，角度计算
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)!=RESET){
		printf("CLS(0)\r\n");
		printf("DS48(42,80,'row:%d,rx4:%d',16,0);\r\n",temp0,rx4);
		MPU6050_DATE();
		mpuangle[p_num]=temp0/10;//取整数
		p_num++;
		p_average=(mpuangle[0]+mpuangle[1]+mpuangle[2])*1.0/3.0;//取均值
		if(p_num>=3)p_num=0;
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
	}
}
//其他
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
int main(void){	
		u8 i=5;
	  ard_char_cir = "0";
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//禁用JTAG 启用 SWD
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置NVIC中断分组2:2位抢占优先级，2位响应优先级 
		Pwmin_init();
		delay_init();//延时初始化 
		hongwai();
		delay_ms(10);
		uart_init(115200);
		delay_ms(10);
		uart_init2(115200);
		uart_init4(115200);
		delay_ms(20);
		while(i)printf("CLS(0)\r\n"),i--,delay_ms(20);
		delay_ms(20);
		TIM3_PWM_Init(1000,72);//899.0
		EXTIX_Init();
		LED_Init(); 
		delay_ms(100);
		MPU_Init();					//初始化MPU6050
		while(mpu_dmp_init())delay_ms(200);
		delay_ms(100);
		TIM2_Encoder_Config();            //=====初始化编码器1接口
		TIM4_Encoder_Config();            //=====初始化编码器2接口
		TIM5_Encoder_Config();            //=====初始化编码器3接口
		TIM8_Encoder_Config();            //=====初始化编码器4接口
		TIM6_Int_Init(1999,7199);//设置定时器为0.2s
		TIM7_Int_Init(49,7199);//50ms 
		PID_Init();						//=====PID参数初始化
		memset(res2,0x00,sizeof(char)*10);//清空串口缓存区
		IQR_STOP();
		//while(1);
		
		Motor_XIE(3,0);
		UART4SendByte('2');
	 
	 while(1)
	 {	
		 goon();
		 if(leftSpeedSet || leftSpeedSet2 || rightSpeedSet || rightSpeedSet2)pidcon();
		 else {
			 Set_Pwmm(0,0,0,0);
			 motorLeft = 0;
			 motorLeft2 = 0;
			 motorRight = 0;
			 motorRight2 = 0;
		 }
		 if(!leftSpeedSet)Timer3_PWM_SetDutyCycle(1,0),motorLeft = 0;
		 if(!leftSpeedSet2)Timer3_PWM_SetDutyCycle(2,0),motorLeft2 = 0;
		 if(!rightSpeedSet)Timer3_PWM_SetDutyCycle(3,0),motorRight = 0;
		 if(!rightSpeedSet2)Timer3_PWM_SetDutyCycle(4,0),motorRight2 = 0;

		 num();
		 TRACING_CALIBRATION_ROADINGON();
		 openmv_code();
	 }
 }

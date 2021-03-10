#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

#define LAIN1 PAout(4)// 左前
#define LAIN2 PAout(5)// 
#define LAIN3 PAout(8)//左后
#define LAIN4 PCout(4)//
#define RAIN1 PBout(12)//右前
#define RAIN2 PBout(14)//
#define RAIN3 PBout(13)//右后
#define RAIN4 PBout(15)//
#define hongwai1  PCin(3)//正面
#define hongwai2  PCin(5)//正面
#define hongwai3  PCin(2)//数线
#define hongwai4  PCin(0)//侧面左
#define hongwai5  PCin(1)//侧面右

void LED_Init(void);//初始化
void hongwai(void);
		 				    
#endif

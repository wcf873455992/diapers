#ifndef __AHT10_H__
#define __AHT10_H__
#include <stdint.h>
#include <reg24le1.h>
#include "config.h"
#include <string.h>

#if (BOARD == IKMSIK)
	#define  SCL    P14
	#define  SDA    P15
#else
	#define  SCL    P02
	#define  SDA    P06
#endif

#define		HIGH		1
#define		LOW			0

typedef struct AHT10_VALUE {
    uint8_t  tempH;  //温度高4位十位，第四位个位
    uint8_t  tempL;  //温度小数位
    uint8_t  humyH;   //湿度高4位十位，第四位个位
    uint8_t  humyL;  //湿度小数位

} AHT10VALUE;

extern   AHT10VALUE AHT10Value;

uint8_t AHT10_Init(void);
uint8_t JH_Read_Cal_Enable(void);
void Read_AHT10();

void  SDA_Pin_Output_Low(void);
void  SDA_Pin_Output_Low(void);
void  SCL_Pin_Output_HIGH(void);
void  SCL_Pin_Output_HIGH(void);
void Init_I2C_Sensor_Port(void);
#endif
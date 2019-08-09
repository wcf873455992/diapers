#ifndef __AHT10_H__
#define __AHT10_H__
#include <stdint.h>
#include <reg24le1.h>

#define  SCL    P14
#define  SDA    P15
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

#endif
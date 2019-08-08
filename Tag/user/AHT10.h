#ifndef __AHT10_H__
#define __AHT10_H__
#include <stdint.h>
#include <reg24le1.h>

#define  SCL    P14
#define  SDA    P15
#define		HIGH		1
#define		LOW			0

typedef struct AHT10_VALUE
{
    uint8_t  tempH;  //温度高字节（十位）
    uint8_t  tempL;  //温度低位节（个位）
    uint8_t	 tempD;//温度小数位
    //uint8_t	 tempD2;//温度小数位
    uint8_t  humyH;   //湿度高字节（十位）
    uint8_t  humyL;  //湿度高字节（个位）
    uint8_t	 humyD;		//湿度小数位
    //uint8_t	 humyD2;		//湿度小数位
    int	temp;
    int humy;

} AHT10VALUE;

extern   AHT10VALUE AHT10Value;

uint8_t AHT10_Init(void);
uint8_t JH_Read_Cal_Enable(void);
void Read_AHT10();

#endif
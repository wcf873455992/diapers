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
    uint8_t  tempH;  //�¶ȸ��ֽڣ�ʮλ��
    uint8_t  tempL;  //�¶ȵ�λ�ڣ���λ��
    uint8_t	 tempD;//�¶�С��λ
    //uint8_t	 tempD2;//�¶�С��λ
    uint8_t  humyH;   //ʪ�ȸ��ֽڣ�ʮλ��
    uint8_t  humyL;  //ʪ�ȸ��ֽڣ���λ��
    uint8_t	 humyD;		//ʪ��С��λ
    //uint8_t	 humyD2;		//ʪ��С��λ
    int	temp;
    int humy;

} AHT10VALUE;

extern   AHT10VALUE AHT10Value;

uint8_t AHT10_Init(void);
uint8_t JH_Read_Cal_Enable(void);
void Read_AHT10();

#endif
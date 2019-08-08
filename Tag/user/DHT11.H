#ifndef __DHT11_H__
#define __DHT11_H__
#include <stdint.h>

#define PIN_DATA_OUT   (P0DIR &= ~0x04);   //P0.2����Ϊ����
#define PIN_DATA_IN    (P0DIR |= 0x04);    //P0.2����Ϊ���


typedef struct DHT11_VALUE 
{
    uint16_t tempH;  //�¶ȸ��ֽڣ�ʮλ��
    uint8_t  tempL;  //�¶ȵ�λ�ڣ���λ��
		uint8_t  tempD;  //�¶ȵ�λ�ڣ���λ��
    uint8_t  humyH;   //ʪ�ȸ��ֽڣ�ʮλ��
    uint8_t  humyL;  //ʪ�ȸ��ֽڣ���λ�
		uint8_t  humyD;  //ʪ�ȸ��ֽڣ���λ���

}DHT11VALUE;

extern   DHT11VALUE DHT11Value;




extern void COM(void);          // ��ʪд��
extern void DHT11(void);        //��ʪ�ȴ���������

extern uint8_t temp[2]; 
extern uint8_t temp1[5];
extern uint8_t humidity[2];
extern uint8_t humidity1[9];

#endif 
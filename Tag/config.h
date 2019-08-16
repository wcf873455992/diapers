#ifndef CONFIG_H__
#define CONFIG_H__

#define IKMSIK 1
#define DIAPER 2

#define BOARD DIAPER//DIAPER//IKMSIK
#define	TRUE	1
#define FALSE	0
/******************************************************************************/
/* ������Ϣ*/

#define  DEBUG_LED
//#define  DEBUG_UART

/******************************************************************************/
/* Tag ����  */
#define  TAG_ID          1000	    // ID
#define  TAG_CH          50	      // �ŵ�

#if 1
	#define  TAG_TIME        2*32768U	  // ����ʱ�䣺2*1000ms
#else
	#define  TAG_TIME        2.5*3276U	  // ����ʱ�䣺10ms ������
#endif

#define  MAX_TX_PAYLOAD  10//6        // ���߷������ݳ���
#define  TAG_ID_LEN      2        // ID�ֽ���
#define  ADC_TIME        7200U      // ADC���ʱ����

#define MINUTE		0//30=1����
#define HOUR		60
/******************************************************************************/
/* Watchdog*/
/*
** ���Ź���ʱʱ�����
**  ��ʱʱ�� = (WDSV * 256)/32768
**  ���ԣ���С���Ź���ʱ���� = 7.8125ms
**        ����Ź���ʱ���� = 512s
*/
//#define USE_WDT   1//
#define WDT_TIME  256  //С������2Sʱ������


/******************************************************************************/
/* ADC */
/*
** Descriptions: ÿ����һ�ΰ���S2��nRF24LE1��VDD����һ�β�������������ֵͨ���������
** ADC���ã�
**   ��׼��ѹ���ڲ�1.2V
**   ͨ�������1/3 VDD��ѹ
**   �ֱ��ʣ�12λ
**   ����ģʽ�����β���
**   �����ٶȣ�2ksps
**   ���ݶ��뷽ʽ���Ҷ���
** ���ڲ����ʣ�57600
** ��ѹ���㹫ʽ��V =��1.2*3��*����ֵ/4096
*/

typedef struct Tag_Information {
    union {
        uint16_t id16;
        uint8_t id8[2];
    } id;

    uint8_t   CellVoltageH;	  // ��ص�ѹ��λ//���ֽ�
    uint8_t   CellVoltageL;		// ��4λ��ص�ѹС������1λ
    // ��4λ��ص�ѹС������2λ
    uint8_t		tempH;	//��4λ�¶�ʮλ����4λ�¶ȸ�λ
    uint8_t		tempL;	//�¶�С��λ
    uint8_t		humyH;	//��4λʪ��ʮλ����4λʪ�ȸ�λ
    uint8_t		humyL;	//ʪ��С��λ

} TagInformation;

extern  xdata TagInformation TagInfo;


#endif

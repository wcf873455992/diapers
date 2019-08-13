/****************************************Copyright (c)****************************************************
**
**                                 �Ϸʰ���ķ���ӿƼ����޹�˾
**                                  ��̳��http://930ebbs.com
**--------------File Info---------------------------------------------------------------------------------
** File name:			main.c
** Last modified Date: 2017-3-1
** Last Version:		   1.3
** Descriptions:
**
**--------------------------------------------------------------------------------------------------------
** Created by:			FiYu
** Created date:		2014-11-12
** Version:			    1.0
** Descriptions:		��ԴRFIDʵ�����-TAG
**						      �������ݸ�ʽ
**                  ���ȣ�6���ֽ�
**                  ID���ȣ�2�ֽ�
�ֽ�:    1     2   3    4  5   6	7		8		9		10
����:   ����  IDH IDL  VH  VL  �¶�		ʪ��		У��

**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Descriptions:
**
** Rechecked by:
**********************************************************************************************************/
#include <reg24le1.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "hal_nrf.h"
#include "hal_nrf_hw.h"
#include "hal_clk.h"
#include "hal_rtc.h"
#include "hal_delay.h"
#include "hal_adc.h"
#include "hal_wdog.h"
#include "hal_uart.h"

#include <stdio.h>
#include <string.h>

#include "user/AHT10.h"
#include "user/DHT11.h"
#include "user/led.h"
#include "user/key.h"

/*-------------------�ܽŶ���--------------------------------------------------*/
#define  D1    P00  // �������ϵ�ָʾ��D1
#define  D2    P01  // �������ϵ�ָʾ��D2
#define  S2    P12  // �������ϵİ���S2
#define  S3    P13  // �������ϵİ���S3
#define  ADC   P06



/*******************************************************************************************************
 * ��������
 *******************************************************************************************************/
xdata bool  radio_busy;
xdata uint8_t  TxPayload[32];
xdata uint8_t  RxPayload[32];
uint16_t PipeAndLen;
uint8_t  TX_ADDRESS[5]  = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // TX address

xdata TagInformation TagInfo;


/*******************************************************************************************************
 * ��  �� : ��ʼ��IO
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void IoInit(void) {
    P0DIR = 0x00;
    P1DIR = 0x00;
}
#ifdef DEBUG_UART
/*******************************************************************************************************
 * ��  �� : ��������ַ�
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void uart_sendchar(uint8_t dat) {
    S0BUF = dat;
    while(!TI0);
    TI0 = 0;
}
#endif
/*******************************************************************************************************
 * ��  �� : ��������ַ���
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void PutString(char *s) {
#ifdef DEBUG_UART
    while(*s != 0)
        uart_sendchar(*s++);
    //delay_ms(1);
#endif
}
/*********************************************************************************************************
** ��  ��:  adc��ʼ��
** ��  ��:  NONE
** ����ֵ:  NONE
*********************************************************************************************************/
void adc_init(void) {
    hal_adc_set_input_channel(HAL_INP_VDD1_3);          //����ͨ�� ���1/3 VDD��ѹ
    hal_adc_set_reference(HAL_ADC_REF_INT);             //���òο���ѹ �ڲ�1.22V
    hal_adc_set_input_mode(HAL_ADC_SINGLE);             //��������
    hal_adc_set_conversion_mode(HAL_ADC_SINGLE_STEP);   //���β���ģʽ
    hal_adc_set_sampling_rate(HAL_ADC_2KSPS);           //��������Ϊ	2ksps
    hal_adc_set_resolution(HAL_ADC_RES_12BIT);          //12λADC
    hal_adc_set_data_just(HAL_ADC_JUST_RIGHT);          //�����Ҷ���
}

/*******************************************************************************************************
 * ��  �� : ��������ʱ��
 * ��  �� : period:����ʱ��
 * ����ֵ : ��
 *******************************************************************************************************/
void set_timer_period(uint16_t period) {
    if((period < 10) && (period > 65536))period = 32768;

    hal_rtc_start(false);
    hal_rtc_start(true);
    hal_rtc_set_compare_value(period - 1);
}

/*******************************************************************************************************
 * ��  �� : �������߲���
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void RfCofig(void) {
    RFCKEN = 1;	     //ʹ��RFʱ��

    hal_nrf_close_pipe(HAL_NRF_ALL);           //�ȹر����е�ͨ��.
    hal_nrf_open_pipe(HAL_NRF_PIPE0, false);	  //�ٴ�ͨ��0.

    hal_nrf_set_operation_mode(HAL_NRF_PTX);    // ģʽ������
    hal_nrf_set_rf_channel(TAG_CH);		          // RF�ŵ���50�����պͷ��ͱ��봦��ͬһ�ŵ�
    hal_nrf_set_datarate(HAL_NRF_250KBPS);	    // RF���ʣ�250KBPS
    hal_nrf_set_output_power(HAL_NRF_0DBM);	    // ���ʣ�0DBM
    hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);    //����CRCУ�飺16λCRC������ͽ����豸һ�¡�
    hal_nrf_set_address(HAL_NRF_TX, TX_ADDRESS); //���÷������ַ
    hal_nrf_set_auto_retr(0, 1500);			           //�Զ��ط�:0

    hal_nrf_set_power_mode(HAL_NRF_PWR_UP);	        //������ϵ�
    RF = 1;       //ʹ�������ж�
    EA = 1;	     // ʹ��ȫ���ж�
}

/*******************************************************************************************************
 * ��  �� : ʱ�Ӻ�RTC��������
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void mcu_init(void) {
    hal_rtc_start(false);
    hal_clklf_set_source(HAL_CLKLF_RCOSC32K);           // Use 32.768KHz��ʱ��ԴΪ�ڲ�RC

    hal_rtc_set_compare_mode(HAL_RTC_COMPARE_MODE_0);   // Use 32 KHz timer mode 0
    set_timer_period(TAG_TIME);	                        // Set the RTC2 time��card sleep time
    hal_clk_set_16m_source(HAL_CLK_XOSC16M);            // Always run on 16MHz crystal oscillator
    hal_clk_regret_xosc16m_on(0);                       // Keep XOSC16M off in register retention

    hal_rtc_start(true);

    while((CLKLFCTRL & 0x80) == 0x80);	                // Wait for the 32kHz to startup (change phase)
    while((CLKLFCTRL & 0x80) != 0x80);
}

/*******************************************************************************************************
 * ��  �� : ��װ����
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void Assemble_Data(void) {
    xdata uint8_t fcs = 0, i;


    TxPayload[0] = 0x1E;
    TxPayload[1] = TagInfo.id.id8[0];  //IDL
    TxPayload[2] = TagInfo.id.id8[1];  //IDH
    TxPayload[3] = TagInfo.CellVoltageH;
    TxPayload[4] = TagInfo.CellVoltageL;
    TxPayload[5] = TagInfo.tempH;
    TxPayload[6] = TagInfo.tempL;
    TxPayload[7] = TagInfo.humyH;
    TxPayload[8] = TagInfo.humyL;

    for(i = 0; i < (MAX_TX_PAYLOAD - 1); i++)fcs += TxPayload[i];

    TxPayload[MAX_TX_PAYLOAD - 1] = (256 - fcs) % 256;
}
void CellVoltage() {
    uint32_t	ad_value = 0, temp = 0;
    hal_adc_start();           //����ADC
    while( hal_adc_busy())     //�ȴ�ADCת������
        ;
    TagInfo.CellVoltageH = hal_adc_read_MSB(); //��ȡADC����ֵ
    TagInfo.CellVoltageL = hal_adc_read_LSB();
    ad_value = (ad_value | hal_adc_read_MSB()) << 8;
    ad_value = ad_value | hal_adc_read_LSB();
    temp = ad_value * 1.2 * 3 * 100 / 4096; //��ѹ����100��
    TagInfo.CellVoltageH = temp / 100; //
    TagInfo.CellVoltageL = ((temp % 100 / 10) << 4) | (temp % 100 % 10);
}

void AHT10(void) {
    uint8_t temp[5];
    uint8_t humidity[5];

    memset(temp, 0, 5);
    memset(humidity, 0, 5);
    temp[2] = '.';
    humidity[2] = '.';

    //while(1)
    {

        //DisableIrq(); //������ģ��I2C,Ϊ��ȡ���ݸ�׼ȷ ����ȡ֮ǰ��ֹ�ж�
        Read_AHT10();  //��ȡ�¶Ⱥ�ʪ�� �� �ɼ��1.5S��һ��
        //EnableIrq(); //�ָ��ж�

        TagInfo.tempH = AHT10Value.tempH ;
        TagInfo.tempL = AHT10Value.tempL ;
        TagInfo.humyH = AHT10Value.humyH ;
        TagInfo.humyL = AHT10Value.humyL ;

        temp[0] = (AHT10Value.tempH >> 4) + 0x30;
        temp[1] = (AHT10Value.tempH & 0x0f) + 0x30;
        temp[3] = (AHT10Value.tempL >> 4) + 0x30;
        humidity[0] = (TagInfo.humyH >> 4) + 0x30;
        humidity[1] = (TagInfo.humyH & 0x0f) + 0x30;
        humidity[3] = (TagInfo.humyL >> 4) + 0x30;

        PutString("AHT10-�¶�:");
        PutString(temp);
        PutString("    ");
        PutString("ʪ��:");
        PutString(humidity);
        PutString("\r\n");
        /**/
        //delay_ms(1500); //��ʱ1.5S
        //Ϊ��ȡ�����ݸ��ȶ���������ʹ��ƽ��ֵ�˲����ߴ����˲�������ǰ���ȡ��ֵ������ֵ����̫��
    }
}
void	Send_data() {
    Assemble_Data();  // ���ݴ��
    hal_nrf_write_tx_payload(TxPayload, MAX_TX_PAYLOAD);

    CE_PULSE();	            //���߷�������
    radio_busy = true;
    while(radio_busy)		    //�ȴ��������
        ;
    led_flash(led1);
}
/*******************************************************************************************************
 * ��  �� : ��ȡ��һ�ε͹���ģʽ����ͨ�����ڴ�ӡ
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void GetPrintLastPWM(void) {
    uint8_t PowrMode;

    PowrMode = PWRDWN & 0x07;

    switch(PowrMode) {
    case 0x00:
        PutString("Last mode:Power Off!");
        break;
    case 0x01:
        PutString("Last mode:Deep Sleep!");
        break;
    case 0x02:
        PutString("Last mode:Memory Retention, Timer Off!");
        break;
    case 0x03:
        PutString("Last mode:Memory Retention, Timer On!");
        break;
    case 0x04:
        PutString("Last mode:Register Retention!");
        break;
    default  :
        PutString("Error Or Reserved!");
        break;
    }
}
/*******************************************************************************************************
 * ��  �� : ���û���PIN
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void SetWakeUpPin(void) {
    OPMCON = 0x04;             /* �������͵�ƽ����          */
    WUOPC0 = 0x00;             /* P0���ѵ��������ã���      */
    WUOPC1 = 0x04;             /* P1���ѵ��������ã�P12     */

    P1DIR |= 0x04;             /* P12��ʼ��Ϊ����I/O��      */
    P12   = 1;                 /* P12��ʼ��Ϊ�ߵ�ƽ         */
}
/*******************************************************************************************************
 * ��  �� : ������
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/


#define  DEEPSLEEP        1  // ���˯��
#define  MEMRET_TIMEOFF   2	 // �洢��ά�֣���ʱ���ر�
#define  MEMRET_TIMEON    3	 // �洢��ά�֣���ʱ������
#define  REGRET           4	 // �Ĵ���ά��
/*******************************************************************************************************
 * ��  �� : ����nRF24LE1�͹���ģʽ
 * ��  �� : mode:�͹���ģʽ
 * ����ֵ : ��
 *******************************************************************************************************/
void SetPowrDownMode(uint8_t mode) {
    uint8_t PowrMode;

    switch(mode) {
    case DEEPSLEEP  :     // ���˯�ߣ����Ѻ�λ
        PowrMode = 0x01;
        break;
    case MEMRET_TIMEOFF : // �洢��ά�֣���ʱ���رգ����Ѻ�λ
        PowrMode = 0x02;
        break;
    case MEMRET_TIMEON:	  // �洢��ά�֣���ʱ�����������Ѻ�λ
        PowrMode = 0x03;
        break;
    case REGRET :         // �Ĵ���ά��
        PowrMode = 0x04;
        break;
    default         :	   //����
        PowrMode = 0x00;
        break;
    }

    if(PowrMode == 0x01)hal_clk_set_16m_source(HAL_CLK_RCOSC16M); // ���뵽DEEPSLEEPǰһ��Ҫ����RCʱ��Դ
    if((PowrMode == 0x01) || (PowrMode == 0x02) || (PowrMode == 0x03))OPMCON |= 0x02;	 // ��3�ֵ͹���ģʽ���Ѻ�nRF24LE1�Ḵλ������Ҫ����IO

    PWRDWN = PowrMode;
    PWRDWN = 0x00;  // Clear power down
}

void main(void) {
    uint8_t RfReceLen, second = 0;
    uint32_t	minute = 0;

    TagInfo.id.id16 = TAG_ID;
    IoInit();      //����IO
    SetWakeUpPin(); //���û��ѹܽ�
    led_init();
    key_init();
    mcu_init();
    adc_init();
    AHT10_Init();
    RfCofig();
#ifdef DEBUG_UART
    hal_uart_init(UART_BAUD_57K6); // ��ʼ��UART0
    while(hal_clk_get_16m_source() != HAL_CLK_XOSC16M) //�ȴ�ʱ���ȶ�
        ;
#endif
#if USE_WDT
    hal_wdog_init(WDT_TIME);//���ÿ��Ź���ʱʱ��2s��ʹ�ܿ��Ź�
#endif
    PutString("Program starting...\r\n");
    GetPrintLastPWM();
    CellVoltage();
    AHT10();
    Send_data();
    while(1) {

#if USE_WDT
        hal_wdog_restart(); //ι��
#endif
        if(KEY1 == 0) {
            delay_ms(100);
            if(KEY1 == 0) {
                second = 0;
                minute = 0;
                CellVoltage();
                AHT10();
                Send_data();
            }
        }
        if(second <= MINUTE) { //һ����
            second++;
            SetPowrDownMode(REGRET);
            PutString("into REGRET\n");
        } else {
            second = 0;
            minute++;
            if(minute == 24 * HOUR) {
                PutString("into DEEPSLEEP\n");
                //SetPowrDownMode(DEEPSLEEP);							
								CellVoltage();
								minute = 0;
            }
            AHT10();
            Send_data();
        }
    }
}
/*******************************************************************************************************
 * ��  �� : �����жϷ�����
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void rf_irq() interrupt INTERRUPT_RFIRQ {
    uint8_t  irq_flags;

    irq_flags = hal_nrf_get_clear_irq_flags(); //��ȡ����������жϱ�־

    if(irq_flags & (1 << HAL_NRF_RX_DR)) { //���յ�����?

        while(!hal_nrf_rx_fifo_empty()) { // Read payload
            PipeAndLen = hal_nrf_read_rx_payload(RxPayload);//��ȡ����
        }
        radio_busy = false;
    }

    if(irq_flags & ((1 << HAL_NRF_TX_DS))) {		// transimmter finish
        radio_busy = false;
    }

    if(irq_flags & ((1 << HAL_NRF_MAX_RT))) {		// re-transimmter
        radio_busy = false;
        hal_nrf_flush_tx();
    }
}
/********************************************END FILE*****************************************************/


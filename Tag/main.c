/****************************************Copyright (c)****************************************************
**
**                                 合肥艾克姆电子科技有限公司
**                                  论坛：http://930ebbs.com
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
** Descriptions:		有源RFID实验程序-TAG
**						      发送数据格式
**                  长度：6个字节
**                  ID长度：2字节
字节:    1     2   3    4  5   6
意义:   命令  IDH IDL  VH  VL  校验

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

/*-------------------管脚定义--------------------------------------------------*/
#define  D1    P00  // 开发板上的指示灯D1
#define  D2    P01  // 开发板上的指示灯D2
#define  S2    P12  // 开发板上的按键S2
#define  S3    P13  // 开发板上的按键S3
#define  ADC   P06



/*******************************************************************************************************
 * 变量定义
 *******************************************************************************************************/
xdata bool  radio_busy;
xdata uint8_t  TxPayload[32];
xdata uint8_t  RxPayload[32];
uint16_t PipeAndLen;
uint8_t  TX_ADDRESS[5]  = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // TX address

xdata TagInformation TagInfo;


/*******************************************************************************************************
 * 描  述 : 初始化IO
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void IoInit(void)
{
    P0DIR = 0x00;
    P1DIR = 0x00;
}
#ifdef DEBUG_UART
/*******************************************************************************************************
 * 描  述 : 串口输出字符
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void uart_sendchar(uint8_t dat)
{
    S0BUF = dat;
    while(!TI0);
    TI0 = 0;
}
#endif
/*******************************************************************************************************
 * 描  述 : 串口输出字符串
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void PutString(char *s)
{
	#ifdef DEBUG_UART
    while(*s != 0)
        uart_sendchar(*s++);
    //delay_ms(1);
		#endif
}
/*********************************************************************************************************
** 描  述:  adc初始化
** 入  参:  NONE
** 返回值:  NONE
*********************************************************************************************************/
void adc_init(void)
{
    hal_adc_set_input_channel(HAL_INP_VDD1_3);          //设置通道 检测1/3 VDD电压
    hal_adc_set_reference(HAL_ADC_REF_INT);             //设置参考电压 内部1.22V
    hal_adc_set_input_mode(HAL_ADC_SINGLE);             //单端输入
    hal_adc_set_conversion_mode(HAL_ADC_SINGLE_STEP);   //单次采样模式
    hal_adc_set_sampling_rate(HAL_ADC_2KSPS);           //采样速率为	2ksps
    hal_adc_set_resolution(HAL_ADC_RES_12BIT);          //12位ADC
    hal_adc_set_data_just(HAL_ADC_JUST_RIGHT);          //数据右对齐
}

/*******************************************************************************************************
 * 描  述 : 设置休眠时间
 * 入  参 : period:休眠时间
 * 返回值 : 无
 *******************************************************************************************************/
void set_timer_period(uint16_t period)
{
    if((period < 10) && (period > 65536))period = 32768;

    hal_rtc_start(false);
    hal_rtc_start(true);
    hal_rtc_set_compare_value(period - 1);
}

/*******************************************************************************************************
 * 描  述 : 配置无线参数
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void RfCofig(void)
{
    RFCKEN = 1;	     //使能RF时钟

    hal_nrf_close_pipe(HAL_NRF_ALL);           //先关闭所有的通道.
    hal_nrf_open_pipe(HAL_NRF_PIPE0, false);	  //再打开通道0.

    hal_nrf_set_operation_mode(HAL_NRF_PTX);    // 模式：发送
    hal_nrf_set_rf_channel(TAG_CH);		          // RF信道：50。接收和发送必须处于同一信道
    hal_nrf_set_datarate(HAL_NRF_250KBPS);	    // RF速率：250KBPS
    hal_nrf_set_output_power(HAL_NRF_0DBM);	    // 功率：0DBM
    hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);    //设置CRC校验：16位CRC。必须和接收设备一致。
    hal_nrf_set_address(HAL_NRF_TX, TX_ADDRESS); //设置发射机地址
    hal_nrf_set_auto_retr(0, 1500);			           //自动重发:0

    hal_nrf_set_power_mode(HAL_NRF_PWR_UP);	        //发射机上电
    RF = 1;       //使能无线中断
    EA = 1;	     // 使能全局中断
}

/*******************************************************************************************************
 * 描  述 : 时钟和RTC唤醒配置
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void mcu_init(void)
{
    hal_rtc_start(false);
    hal_clklf_set_source(HAL_CLKLF_RCOSC32K);           // Use 32.768KHz的时钟源为内部RC

    hal_rtc_set_compare_mode(HAL_RTC_COMPARE_MODE_0);   // Use 32 KHz timer mode 0
    set_timer_period(TAG_TIME);	                        // Set the RTC2 time，card sleep time
    hal_clk_set_16m_source(HAL_CLK_XOSC16M);            // Always run on 16MHz crystal oscillator
    hal_clk_regret_xosc16m_on(0);                       // Keep XOSC16M off in register retention

    hal_rtc_start(true);

    while((CLKLFCTRL & 0x80) == 0x80);	                // Wait for the 32kHz to startup (change phase)
    while((CLKLFCTRL & 0x80) != 0x80);
}

/*******************************************************************************************************
 * 描  述 : 组装数据
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void Assemble_Data(void)
{
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
void CellVoltage(){
	uint32_t	ad_value = 0,temp=0;
	hal_adc_start();           //启动ADC
	while( hal_adc_busy())     //等待ADC转换结束
	;	
	TagInfo.CellVoltageH = hal_adc_read_MSB(); //读取ADC采样值
	TagInfo.CellVoltageL = hal_adc_read_LSB();
	ad_value = (ad_value|hal_adc_read_MSB())<<8;
	ad_value = ad_value|hal_adc_read_LSB();
	temp = ad_value*1.2*3*100/4096; //电压扩大100倍
	TagInfo.CellVoltageH = temp/100;//
	TagInfo.CellVoltageL = ((temp%100/10)<<4)|(temp%100%10);
}

void AHT10(void)
{
    uint8_t temp[5];
    uint8_t humidity[5];

    memset(temp, 0, 5);
    memset(humidity, 0, 5);
    temp[2] = '.';
    humidity[2] = '.';

    //while(1)
    {

        //DisableIrq(); //由于是模拟I2C,为读取数据更准确 ，读取之前禁止中断
        Read_AHT10();  //读取温度和湿度 ， 可间隔1.5S读一次
        //EnableIrq(); //恢复中断

				TagInfo.tempH = AHT10Value.tempH ;
        TagInfo.tempL = AHT10Value.tempL ;
        TagInfo.humyH = AHT10Value.humyH ;
        TagInfo.humyL = AHT10Value.humyL ;
				
        temp[0] = (AHT10Value.tempH>>4) + 0x30;
        temp[1] = (AHT10Value.tempH&0x0f) + 0x30;
        temp[3] = (AHT10Value.tempL>>4) + 0x30;
        humidity[0] = (TagInfo.humyH>>4) + 0x30;
        humidity[1] = (TagInfo.humyH&0x0f) + 0x30;
        humidity[3] = (TagInfo.humyL>>4) + 0x30;
			        	 
			PutString("AHT10-温度:");
        	 PutString(temp);
        	 PutString("    ");
        	 PutString("湿度:");
        	 PutString(humidity);
        	 PutString("\r\n");
       /**/ 		
        //delay_ms(1500); //延时1.5S
        //为读取的数据更稳定，还可以使用平均值滤波或者窗口滤波，或者前面读取的值与后面的值相差不能太大。
    }
}
void	Send_data(){
		Assemble_Data();  // 数据打包
		hal_nrf_write_tx_payload(TxPayload, MAX_TX_PAYLOAD);

		CE_PULSE();	            //无线发射数据
		radio_busy = true;
		while(radio_busy)		    //等待操作完成
		;
}
/*******************************************************************************************************
 * 描  述 : 主函数
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void main()
{
    uint8_t RfReceLen, i, second = 0;
    xdata   uint32_t  loopCount = ADC_TIME - 1;
    uint32_t	minute = 0;

    TagInfo.id.id16 = TAG_ID;
    IoInit();
		led_init();
    mcu_init();
    adc_init();
		AHT10_Init();		
    RfCofig();
	
#ifdef DEBUG_UART
    hal_uart_init(UART_BAUD_57K6);  //初始化UART，波特率57600
    while(hal_clk_get_16m_source() != HAL_CLK_XOSC16M) //等待时钟稳定
        ;
#endif
    //EA = 1;             //使能全局中断
	
#ifdef  USE_WDT
    hal_wdog_init(WDT_TIME);//配置看门狗超时时间2s，使能看门狗
#endif
    PutString("reset\r\n");    
		led_flash(led2);
		
		CellVoltage();	
		AHT10();
		Send_data();
		
    while(1)
    {
#ifdef  USE_WDT
        hal_wdog_restart(); //喂狗
#endif
        second++;
        if(second <= MINUTE) //未到1分钟
        {		
#ifdef 			DEBUG_LED
						//led_flash(led1);
#endif
            PWRDWN = 0x04;    // 进入寄存器维持低功耗模式
            PWRDWN = 0x00;
        }
        else
        {
            second = 0;
            minute++;					
#ifdef 			DEBUG_LED
					led_flash(led1);
#endif
					  AHT10();
            if(minute == 3 * HOUR)  //启动后执行一次AD检测，以后，每2小时检测一次
            {
               CellVoltage();
							minute = 0;
            }
						Send_data();
        }
    }
}

/*******************************************************************************************************
 * 描  述 : 无线中断服务函数
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void rf_irq() interrupt INTERRUPT_RFIRQ
{
    uint8_t  irq_flags;

    irq_flags = hal_nrf_get_clear_irq_flags(); //读取并清除无线中断标志

    if(irq_flags & (1 << HAL_NRF_RX_DR)) //接收到数据?
    {

        while(!hal_nrf_rx_fifo_empty())// Read payload
        {
            PipeAndLen = hal_nrf_read_rx_payload(RxPayload);//读取数据
        }
        radio_busy = false;
    }

    if(irq_flags & ((1 << HAL_NRF_TX_DS)))			// transimmter finish
    {
        radio_busy = false;
    }

    if(irq_flags & ((1 << HAL_NRF_MAX_RT)))			// re-transimmter
    {
        radio_busy = false;
        hal_nrf_flush_tx();
    }
}
/********************************************END FILE*****************************************************/


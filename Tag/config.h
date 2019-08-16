#ifndef CONFIG_H__
#define CONFIG_H__

#define IKMSIK 1
#define DIAPER 2

#define BOARD DIAPER//DIAPER//IKMSIK
#define	TRUE	1
#define FALSE	0
/******************************************************************************/
/* 调试信息*/

#define  DEBUG_LED
//#define  DEBUG_UART

/******************************************************************************/
/* Tag 参数  */
#define  TAG_ID          1000	    // ID
#define  TAG_CH          50	      // 信道

#if 1
	#define  TAG_TIME        2*32768U	  // 休眠时间：2*1000ms
#else
	#define  TAG_TIME        2.5*3276U	  // 休眠时间：10ms 测试用
#endif

#define  MAX_TX_PAYLOAD  10//6        // 无线发送数据长度
#define  TAG_ID_LEN      2        // ID字节数
#define  ADC_TIME        7200U      // ADC检测时间间隔

#define MINUTE		0//30=1分钟
#define HOUR		60
/******************************************************************************/
/* Watchdog*/
/*
** 看门狗超时时间计算
**  超时时间 = (WDSV * 256)/32768
**  所以：最小看门狗超时周期 = 7.8125ms
**        最大看门狗超时周期 = 512s
*/
//#define USE_WDT   1//
#define WDT_TIME  256  //小于休眠2S时间重启


/******************************************************************************/
/* ADC */
/*
** Descriptions: 每按下一次按键S2，nRF24LE1对VDD进行一次采样，并将采样值通过串口输出
** ADC配置：
**   基准电压：内部1.2V
**   通道：检测1/3 VDD电压
**   分辨率：12位
**   采样模式：单次采样
**   采样速度：2ksps
**   数据对齐方式：右对齐
** 串口波特率：57600
** 电压计算公式：V =（1.2*3）*采样值/4096
*/

typedef struct Tag_Information {
    union {
        uint16_t id16;
        uint8_t id8[2];
    } id;

    uint8_t   CellVoltageH;	  // 电池电压个位//高字节
    uint8_t   CellVoltageL;		// 高4位电池电压小数点后第1位
    // 低4位电池电压小数点后第2位
    uint8_t		tempH;	//高4位温度十位，低4位温度个位
    uint8_t		tempL;	//温度小数位
    uint8_t		humyH;	//高4位湿度十位，低4位湿度个位
    uint8_t		humyL;	//湿度小数位

} TagInformation;

extern  xdata TagInformation TagInfo;


#endif

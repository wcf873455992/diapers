#include <reg24le1.h>
#include "DHT11.H" 
#include "hal_delay.h"
#include <string.h>

#define DATA_PIN P02

//温湿度定义
uint8_t U8FLAG,uchartemp;
uint8_t u8T_data_H,u8T_data_L,u8RH_data_H,u8RH_data_L,u8checkdata;
uint8_t u8T_data_H_temp,u8T_data_L_temp,u8RH_data_H_temp,uRH_data_L_temp,u8checkdata_temp;
uint8_t ucharcomdata;

DHT11VALUE DHT11Value;
/******************************************************************************
 * 描  述 : 1 us延时
 * 入  参 : 无
 * 返回值 : 无
 *****************************************************************************/
/*void Delay_us() 
{
	#pragma ASM
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");    
    asm("nop");
    asm("nop");
    asm("nop");    
    asm("nop"); 
	#pragma ENDASM
}
*/
/******************************************************************************
 * 描  述 : 10 us延时
 * 入  参 : 无
 * 返回值 : 无
 *****************************************************************************/
void Delay_10us() 
{
  delay_us(1);
  delay_us(1);
  delay_us(1);
  delay_us(1);
  delay_us(1);
  delay_us(1);
  delay_us(1);
  delay_us(1);
  delay_us(1);
  delay_us(1);   
}

/******************************************************************************
 * 描  述 : Ds18b20写操作：写入1个字节
 * 入  参 : 无
 * 返回值 : 无
 *****************************************************************************/
//温湿度传感
void COM(void)    // 温湿写入
{     
    uint8_t i;         
    for(i=0;i<8;i++)    
    {
        U8FLAG=2; 
        while((!DATA_PIN)&&U8FLAG++);
        Delay_10us();
        Delay_10us();
        Delay_10us();
        uchartemp=0;
        if(DATA_PIN)uchartemp=1;
        U8FLAG=2;
        while((DATA_PIN)&&U8FLAG++);   
        if(U8FLAG==1)break;    
        ucharcomdata<<=1;
        ucharcomdata|=uchartemp; 
    }    
}

/******************************************************************************
 * 描  述 : Ds18b20写操作：写入1个字节
 * 入  参 : 无
 * 返回值 : 无
 *****************************************************************************/
void DHT11(void)   //温湿传感启动
{
    PIN_DATA_OUT
	  DATA_PIN=0;
    delay_ms(19);  //>18MS
    DATA_PIN=1; 
    PIN_DATA_IN;  //配置P0.2为输入
    Delay_10us();
    Delay_10us();                        
    Delay_10us();
    Delay_10us();  
    if(!DATA_PIN) 
    {
        U8FLAG=2; 
        while((!DATA_PIN)&&U8FLAG++);
        U8FLAG=2;
        while((DATA_PIN)&&U8FLAG++); 
        COM();
        u8RH_data_H_temp=ucharcomdata;
        COM();
        uRH_data_L_temp=ucharcomdata;
        COM();
        u8T_data_H_temp=ucharcomdata;
        COM();
        u8T_data_L_temp=ucharcomdata;
        COM();
        u8checkdata_temp=ucharcomdata;
        DATA_PIN=1; 
        uchartemp=(u8T_data_H_temp+u8T_data_L_temp+u8RH_data_H_temp+uRH_data_L_temp);
        if(uchartemp==u8checkdata_temp)
        {
            u8RH_data_H=u8RH_data_H_temp;
            u8RH_data_L=uRH_data_L_temp;
            u8T_data_H=u8T_data_H_temp;
            u8T_data_L=u8T_data_L_temp;
            u8checkdata=u8checkdata_temp;
        }
        DHT11Value.tempH = u8T_data_H/10; 
        DHT11Value.tempL = u8T_data_H%10;
				DHT11Value.tempD = u8T_data_L%10;
        
        DHT11Value.humyH = u8RH_data_H/10; 
        DHT11Value.humyL = u8RH_data_H%10; 
				DHT11Value.humyD = u8RH_data_L%10;				
    } 
    else //没用成功读取，返回0
    {
        DHT11Value.tempH = 0; 
        DHT11Value.tempL = 0;
        
        DHT11Value.humyH = 0; 
        DHT11Value.humyL = 0;  
    } 
    
    PIN_DATA_OUT; //配置P0.7为输出 
}
/*********************************END FILE*************************************/
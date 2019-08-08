#include <reg24le1.h>
#include "DHT11.H" 
#include "hal_delay.h"
#include <string.h>

#define DATA_PIN P02

//��ʪ�ȶ���
uint8_t U8FLAG,uchartemp;
uint8_t u8T_data_H,u8T_data_L,u8RH_data_H,u8RH_data_L,u8checkdata;
uint8_t u8T_data_H_temp,u8T_data_L_temp,u8RH_data_H_temp,uRH_data_L_temp,u8checkdata_temp;
uint8_t ucharcomdata;

DHT11VALUE DHT11Value;
/******************************************************************************
 * ��  �� : 1 us��ʱ
 * ��  �� : ��
 * ����ֵ : ��
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
 * ��  �� : 10 us��ʱ
 * ��  �� : ��
 * ����ֵ : ��
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
 * ��  �� : Ds18b20д������д��1���ֽ�
 * ��  �� : ��
 * ����ֵ : ��
 *****************************************************************************/
//��ʪ�ȴ���
void COM(void)    // ��ʪд��
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
 * ��  �� : Ds18b20д������д��1���ֽ�
 * ��  �� : ��
 * ����ֵ : ��
 *****************************************************************************/
void DHT11(void)   //��ʪ��������
{
    PIN_DATA_OUT
	  DATA_PIN=0;
    delay_ms(19);  //>18MS
    DATA_PIN=1; 
    PIN_DATA_IN;  //����P0.2Ϊ����
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
    else //û�óɹ���ȡ������0
    {
        DHT11Value.tempH = 0; 
        DHT11Value.tempL = 0;
        
        DHT11Value.humyH = 0; 
        DHT11Value.humyL = 0;  
    } 
    
    PIN_DATA_OUT; //����P0.7Ϊ��� 
}
/*********************************END FILE*************************************/
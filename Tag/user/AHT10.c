/*******************************************/
/*@��Ȩ���У����ݰ��ɵ������޹�˾          */
/*@���ߣ���ʪ�ȴ�������ҵ��                */
/*@�汾��V1.2                              */
/*******************************************/
/*******************************************/
/*@�汾˵����                              */
/*@�汾�ţ�V1.2 �޸�AC����Ĳ�����         */
/*@�汾�ţ�V1.1 ����У׼���ʹ�ܼ�⡣     */
/*@�汾�ţ�V1.0 ����汾��                 */
/*******************************************/

#include "hal_delay.h"
#include "AHT10.h"

AHT10VALUE AHT10Value;



void flash_P00() {
    P00 = LOW;
    delay_ms(500);
    P00 = HIGH;
}
#if	(BOARD == IKMSIK)
void SDA_Pin_Output_High(void) { //��P15����Ϊ��� �� ������Ϊ�ߵ�ƽ�� P15��ΪI2C��SDA
    P1DIR &= ~0x20;	   //����P1.5Ϊ���
    SDA = HIGH;
}
void SDA_Pin_Output_Low(void) { //��P15����Ϊ���  ������Ϊ�͵�ƽ
    P1DIR &= ~0x20;	   //����P1.5Ϊ���
    SDA = LOW;
}
void SDA_Pin_IN_FLOATING(void) { //SDA����Ϊ��������
    P1DIR |= 0x20;	   //����P1.5Ϊ����
}
void SCL_Pin_Output_High(void) { //SCL����ߵ�ƽ��P14��ΪI2C��SCL
    P1DIR &= ~0x10;	   //����P1.4Ϊ���
    SCL	= HIGH;
}

void SCL_Pin_Output_Low(void) { //SCL����͵�ƽ
    P1DIR &= ~0x10;	   //����P1.4Ϊ���
    SCL	= LOW;
}

void Init_I2C_Sensor_Port(void) { //��ʼ��I2C�ӿ�
    P1DIR &= ~0x10;	   //����P1.4Ϊ���
    SCL	= HIGH;
    P1DIR &= ~0x20;	   //����P1.5Ϊ���
    SDA = HIGH;
}
#else
void SDA_Pin_Output_High(void) { //��P15����Ϊ��� �� ������Ϊ�ߵ�ƽ�� P15��ΪI2C��SDA
    P0DIR &= ~0x40;	   //����P0.6Ϊ���
    SDA = HIGH;
}
void SDA_Pin_Output_Low(void) { //��P15����Ϊ���  ������Ϊ�͵�ƽ
    P0DIR &= ~0x40;	   //����P0.6Ϊ���
    SDA = LOW;
}
void SDA_Pin_IN_FLOATING(void) { //SDA����Ϊ��������
    P0DIR |= 0x40;	   //����P0.6Ϊ����
}


void SCL_Pin_Output_High(void) { //SCL����ߵ�ƽ��P14��ΪI2C��SCL
    P0DIR &= ~0x04;	   //����P0.2Ϊ���
    SCL	= HIGH;
}

void SCL_Pin_Output_Low(void) { //SCL����͵�ƽ
    P0DIR &= ~0x04;	   //����P0.2Ϊ���
    SCL	= LOW;
}

void Init_I2C_Sensor_Port(void) { //��ʼ��I2C�ӿ�
    P0DIR &= ~0x04;	   //����P0.2Ϊ���
    SCL	= HIGH;
		P0DIR &= ~0x40;	   //����P0.6Ϊ���
    SDA = HIGH;
}
#endif


void I2C_Start(void) {	 //I2C��������START�ź�
    SDA_Pin_Output_High();
    delay_us(8);
    SCL_Pin_Output_High();
    delay_us(8);
    SDA_Pin_Output_Low();
    delay_us(8);
    SCL_Pin_Output_Low();
    delay_us(8);
}


void ZSSC_I2C_WR_Byte(uint8_t Byte) { //��AHT10дһ���ֽ�
    uint8_t Data, N, i;
    Data = Byte;
    i = 0x80;
    for(N = 0; N < 8; N++) {
        SCL_Pin_Output_Low();

        delay_us(3);
        if(i & Data) {
            SDA_Pin_Output_High();
        } else {
            SDA_Pin_Output_Low();
        }

        SCL_Pin_Output_High();
        delay_us(3);

        Data <<= 1;

    }
    SCL_Pin_Output_Low();
    delay_us(8);
    SDA_Pin_IN_FLOATING();
    delay_us(8);
}


uint8_t ZSSC_I2C_RD_Byte(void) { //��AHT10��ȡһ���ֽ�
    uint8_t Byte, i, a;
    Byte = 0;
    SCL_Pin_Output_Low();
    SDA_Pin_IN_FLOATING();
    delay_us(8);
    for(i = 0; i < 8; i++) {
        SCL_Pin_Output_High();
        delay_us(1);
        a = 0;
        if(SDA)a = 1;
        Byte = (Byte << 1) | a;
        SCL_Pin_Output_Low();
        delay_us(8);
    }
    SDA_Pin_IN_FLOATING();
    delay_us(8);
    return Byte;
}


uint8_t Receive_ACK(void) { //��AHT10�Ƿ��лظ�ACK
    uint16_t CNT;
    CNT = 0;
    SCL_Pin_Output_Low();
    SDA_Pin_IN_FLOATING();
    delay_us(8);
    SCL_Pin_Output_High();
    delay_us(8);

    while((SDA)  && CNT < 100)
        CNT++;
    if(CNT == 100) {
        return 0;
    }
    SCL_Pin_Output_Low();
    delay_us(8);
    return 1;
}

void Send_ACK(void) {	  //�����ظ�ACK�ź�
    SCL_Pin_Output_Low();
    delay_us(8);
    SDA_Pin_Output_Low();
    delay_us(8);
    SCL_Pin_Output_High();
    delay_us(8);
    SCL_Pin_Output_Low();
    delay_us(8);
    SDA_Pin_IN_FLOATING();
    delay_us(8);
}

void Send_NOT_ACK(void) {	//�������ظ�ACK
    SCL_Pin_Output_Low();
    delay_us(8);
    SDA_Pin_Output_High();
    delay_us(8);
    SCL_Pin_Output_High();
    delay_us(8);
    SCL_Pin_Output_Low();
    delay_us(8);
    SDA_Pin_Output_Low();
    delay_us(8);
}

void Stop_I2C(void) {  //һ��Э�����
    SDA_Pin_Output_Low();
    delay_us(8);
    SCL_Pin_Output_High();
    delay_us(8);
    SDA_Pin_Output_High();
    delay_us(8);
}

uint8_t JH_Read_Status(void) { //��ȡAHT10��״̬�Ĵ���

    uint8_t Byte_first;
    I2C_Start();
    ZSSC_I2C_WR_Byte(0x71);
    Receive_ACK();
    Byte_first = ZSSC_I2C_RD_Byte();


    Send_NOT_ACK();
    Stop_I2C();


    return Byte_first;
}

uint8_t JH_Read_Cal_Enable(void) { //��ѯcal enableλ��û��ʹ�ܣ�
    uint8_t val = 0;

    val = JH_Read_Status();
    if((val & 0x68) == 0x08) //�ж�NORģʽ��У׼����Ƿ���Ч
        return 1;
    else  return 0;
}



void JH_SendAC(void) { //��AHT10����AC����

    I2C_Start();
    ZSSC_I2C_WR_Byte(0x70);
    Receive_ACK();
    ZSSC_I2C_WR_Byte(0xac);
    Receive_ACK();
    ZSSC_I2C_WR_Byte(0x33);
    Receive_ACK();
    ZSSC_I2C_WR_Byte(0x00);
    Receive_ACK();
    Stop_I2C();

}

void JH_Send_BA(void) { //��AHT10����BA����


    I2C_Start();
    ZSSC_I2C_WR_Byte(0x70);
    Receive_ACK();
    ZSSC_I2C_WR_Byte(0xba);
    Receive_ACK();
    Stop_I2C();


}

void Read_AHT10() { //��ȡAHT10���¶Ⱥ�ʪ������
    volatile uint8_t  Byte_1th = 0;
    volatile uint8_t  Byte_2th = 0;
    volatile uint8_t  Byte_3th = 0;
    volatile uint8_t  Byte_4th = 0;
    volatile uint8_t  Byte_5th = 0;
    volatile uint8_t  Byte_6th = 0;
    uint32_t RetuData = 0;
    volatile int   temp = 0;
    uint16_t cnt = 0;

    while(JH_Read_Cal_Enable() == 0) { //�ȵ�У׼���ʹ��λΪ1���Ŷ�ȡ��
        if(AHT10_Init() == 0){//���Ϊ0��ʹ��һ��
					AHT10Value.humyH = 0xff;
					AHT10Value.humyL = 0xff;
					AHT10Value.tempH = 0xff;
					AHT10Value.tempL = 0xff;
					//return;
				};
				if(cnt++ >= 2){
					return;
				}
        delay_ms(30);
			/**/
    }
		

    JH_SendAC();//��AHT10����AC����
    delay_ms(75);//�ȴ�75ms
    cnt = 0;
    while(((JH_Read_Status() & 0x80) == 0x80)) { //�ȴ�æ״̬����
        delay_us(1508);
        if(cnt++ >= 100) {
            break;
					//return;
        }
    }
    I2C_Start();

    ZSSC_I2C_WR_Byte(0x71);//0x70+1   0x70Ϊ�豸��ַ 1Ϊ����λ
    Receive_ACK();
    Byte_1th = ZSSC_I2C_RD_Byte();//״̬��
    Send_ACK();
    Byte_2th = ZSSC_I2C_RD_Byte();//ʪ���ֽ�
    Send_ACK();
    Byte_3th = ZSSC_I2C_RD_Byte();//ʪ���ֽ�
    Send_ACK();
    Byte_4th = ZSSC_I2C_RD_Byte();//��4λΪʪ��  ��4λΪ�¶�
    Send_ACK();
    Byte_5th = ZSSC_I2C_RD_Byte();//�¶��ֽ�
    Send_ACK();
    Byte_6th = ZSSC_I2C_RD_Byte();//�¶��ֽ�
    Send_NOT_ACK();
    Stop_I2C();

    RetuData = (RetuData | Byte_2th) << 8;
    RetuData = (RetuData | Byte_3th) << 8;
    RetuData = (RetuData | Byte_4th);
    RetuData = RetuData >> 4;

    temp = (RetuData * 1000 / 1024 / 1024); //����õ�ʪ��ֵ���Ŵ���10��,���c1=523����ʾ����ʪ��Ϊ52.3%��
    AHT10Value.humyH = (temp / 100) << 4 | (temp % 100 / 10) ;
    AHT10Value.humyL = temp % 100 % 10 << 4;

    RetuData = 0;
    RetuData = (RetuData | Byte_4th) << 8;
    RetuData = (RetuData | Byte_5th) << 8;
    RetuData = (RetuData | Byte_6th);
    RetuData = RetuData & 0xfffff;

    temp = (RetuData * 2000 / 1024 / 1024 - 500); //����õ��¶�ֵ���Ŵ���10�������t1=245����ʾ�����¶�Ϊ24.5�棩
    AHT10Value.tempH = ((temp / 100) << 4) | (temp % 100 / 10) ;
    AHT10Value.tempL = temp % 100 % 10 << 4;
}


uint8_t AHT10_Init(void) { //��ʼ��AHT10
    uint8_t	count;

		memset(&AHT10Value, 0, sizeof(AHT10Value));
    Init_I2C_Sensor_Port();
    delay_us(11038);

    I2C_Start();
    ZSSC_I2C_WR_Byte(0x70);
    Receive_ACK();
    ZSSC_I2C_WR_Byte(0xe1);//дϵͳ���üĴ���
    Receive_ACK();
    ZSSC_I2C_WR_Byte(0x08);
    Receive_ACK();
    ZSSC_I2C_WR_Byte(0x00);
    Receive_ACK();
    Stop_I2C();

    delay_ms(500);//��ʱ0.5S
    while(JH_Read_Cal_Enable() == 0) { //��Ҫ�ȴ�״̬��status��Bit[3]=1ʱ��ȥ�����ݡ����Bit[3]������1 ���������λ0xBA��AHT10�������³�ʼ��AHT10��ֱ��Bit[3]=1

        JH_Send_BA();  //��λ
        delay_ms(100);
        delay_us(11038);

        I2C_Start();
        ZSSC_I2C_WR_Byte(0x70);
        Receive_ACK();
        ZSSC_I2C_WR_Byte(0xe1);//дϵͳ���üĴ���
        Receive_ACK();
        ZSSC_I2C_WR_Byte(0x08);
        Receive_ACK();
        ZSSC_I2C_WR_Byte(0x00);
        Receive_ACK();
        Stop_I2C();
        count++;
        if(count >= 10)return 0;
        delay_ms(500);
    }
    return 1;
}

void AHT10_test(void) {
    uint8_t temp[5];
    uint8_t humidity[5];
    static uint8_t ret = 0;

    //	memset(temp, 0, 5);
    //memset(humidity, 0, 5);
    temp[2] = '.';
    humidity[2] = '.';
    if (ret == 0) {
        ret = AHT10_Init(); //��ʼ��
        if(ret == 0) {
            //PutString("AHT10��ʼ��ʧ��\r\n");
            while(1);
        }
        //PutString("AHT��ʼ���ɹ�\r\n");
    }
    //while(1)
    {

        //DisableIrq(); //������ģ��I2C,Ϊ��ȡ���ݸ�׼ȷ ����ȡ֮ǰ��ֹ�ж�
        Read_AHT10();  //��ȡ�¶Ⱥ�ʪ�� �� �ɼ��1.5S��һ��
        //EnableIrq(); //�ָ��ж�

        temp[0] = AHT10Value.tempH  ;
        temp[1] = AHT10Value.tempL ;
        humidity[0] = AHT10Value.humyH;
        humidity[1] = AHT10Value.humyL;

        /*	 PutString("AHT10-�¶�:");
        	 PutString(temp);
        	 PutString("    ");
        	 PutString("ʪ��:");
        	 PutString(humidity);
        	 PutString("\r\n");
        		*/
        delay_ms(1500); //��ʱ1.5S
        //Ϊ��ȡ�����ݸ��ȶ���������ʹ��ƽ��ֵ�˲����ߴ����˲�������ǰ���ȡ��ֵ������ֵ����̫��
    }
}
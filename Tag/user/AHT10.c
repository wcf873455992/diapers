/*******************************************/
/*@版权所有：广州奥松电子有限公司          */
/*@作者：温湿度传感器事业部                */
/*@版本：V1.2                              */
/*******************************************/
/*******************************************/
/*@版本说明：                              */
/*@版本号：V1.2 修改AC命令的参数。         */
/*@版本号：V1.1 增加校准输出使能检测。     */
/*@版本号：V1.0 最初版本。                 */
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
void SDA_Pin_Output_High(void) { //将P15配置为输出 ， 并设置为高电平， P15作为I2C的SDA
    P1DIR &= ~0x20;	   //配置P1.5为输出
    SDA = HIGH;
}
void SDA_Pin_Output_Low(void) { //将P15配置为输出  并设置为低电平
    P1DIR &= ~0x20;	   //配置P1.5为输出
    SDA = LOW;
}
void SDA_Pin_IN_FLOATING(void) { //SDA配置为悬浮输入
    P1DIR |= 0x20;	   //配置P1.5为输入
}
void SCL_Pin_Output_High(void) { //SCL输出高电平，P14作为I2C的SCL
    P1DIR &= ~0x10;	   //配置P1.4为输出
    SCL	= HIGH;
}

void SCL_Pin_Output_Low(void) { //SCL输出低电平
    P1DIR &= ~0x10;	   //配置P1.4为输出
    SCL	= LOW;
}

void Init_I2C_Sensor_Port(void) { //初始化I2C接口
    P1DIR &= ~0x10;	   //配置P1.4为输出
    SCL	= HIGH;
    P1DIR &= ~0x20;	   //配置P1.5为输出
    SDA = HIGH;
}
#else
void SDA_Pin_Output_High(void) { //将P15配置为输出 ， 并设置为高电平， P15作为I2C的SDA
    P0DIR &= ~0x40;	   //配置P0.6为输出
    SDA = HIGH;
}
void SDA_Pin_Output_Low(void) { //将P15配置为输出  并设置为低电平
    P0DIR &= ~0x40;	   //配置P0.6为输出
    SDA = LOW;
}
void SDA_Pin_IN_FLOATING(void) { //SDA配置为悬浮输入
    P0DIR |= 0x40;	   //配置P0.6为输入
}


void SCL_Pin_Output_High(void) { //SCL输出高电平，P14作为I2C的SCL
    P0DIR &= ~0x04;	   //配置P0.2为输出
    SCL	= HIGH;
}

void SCL_Pin_Output_Low(void) { //SCL输出低电平
    P0DIR &= ~0x04;	   //配置P0.2为输出
    SCL	= LOW;
}

void Init_I2C_Sensor_Port(void) { //初始化I2C接口
    P0DIR &= ~0x04;	   //配置P0.2为输出
    SCL	= HIGH;
		P0DIR &= ~0x40;	   //配置P0.6为输出
    SDA = HIGH;
}
#endif


void I2C_Start(void) {	 //I2C主机发送START信号
    SDA_Pin_Output_High();
    delay_us(8);
    SCL_Pin_Output_High();
    delay_us(8);
    SDA_Pin_Output_Low();
    delay_us(8);
    SCL_Pin_Output_Low();
    delay_us(8);
}


void ZSSC_I2C_WR_Byte(uint8_t Byte) { //往AHT10写一个字节
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


uint8_t ZSSC_I2C_RD_Byte(void) { //从AHT10读取一个字节
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


uint8_t Receive_ACK(void) { //看AHT10是否有回复ACK
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

void Send_ACK(void) {	  //主机回复ACK信号
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

void Send_NOT_ACK(void) {	//主机不回复ACK
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

void Stop_I2C(void) {  //一条协议结束
    SDA_Pin_Output_Low();
    delay_us(8);
    SCL_Pin_Output_High();
    delay_us(8);
    SDA_Pin_Output_High();
    delay_us(8);
}

uint8_t JH_Read_Status(void) { //读取AHT10的状态寄存器

    uint8_t Byte_first;
    I2C_Start();
    ZSSC_I2C_WR_Byte(0x71);
    Receive_ACK();
    Byte_first = ZSSC_I2C_RD_Byte();


    Send_NOT_ACK();
    Stop_I2C();


    return Byte_first;
}

uint8_t JH_Read_Cal_Enable(void) { //查询cal enable位有没有使能？
    uint8_t val = 0;

    val = JH_Read_Status();
    if((val & 0x68) == 0x08) //判断NOR模式和校准输出是否有效
        return 1;
    else  return 0;
}



void JH_SendAC(void) { //向AHT10发送AC命令

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

void JH_Send_BA(void) { //向AHT10发送BA命令


    I2C_Start();
    ZSSC_I2C_WR_Byte(0x70);
    Receive_ACK();
    ZSSC_I2C_WR_Byte(0xba);
    Receive_ACK();
    Stop_I2C();


}

void Read_AHT10() { //读取AHT10的温度和湿度数据
    volatile uint8_t  Byte_1th = 0;
    volatile uint8_t  Byte_2th = 0;
    volatile uint8_t  Byte_3th = 0;
    volatile uint8_t  Byte_4th = 0;
    volatile uint8_t  Byte_5th = 0;
    volatile uint8_t  Byte_6th = 0;
    uint32_t RetuData = 0;
    volatile int   temp = 0;
    uint16_t cnt = 0;

    while(JH_Read_Cal_Enable() == 0) { //等到校准输出使能位为1，才读取。
        if(AHT10_Init() == 0){//如果为0再使能一次
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
		

    JH_SendAC();//向AHT10发送AC命令
    delay_ms(75);//等待75ms
    cnt = 0;
    while(((JH_Read_Status() & 0x80) == 0x80)) { //等待忙状态结束
        delay_us(1508);
        if(cnt++ >= 100) {
            break;
					//return;
        }
    }
    I2C_Start();

    ZSSC_I2C_WR_Byte(0x71);//0x70+1   0x70为设备地址 1为方向位
    Receive_ACK();
    Byte_1th = ZSSC_I2C_RD_Byte();//状态字
    Send_ACK();
    Byte_2th = ZSSC_I2C_RD_Byte();//湿度字节
    Send_ACK();
    Byte_3th = ZSSC_I2C_RD_Byte();//湿度字节
    Send_ACK();
    Byte_4th = ZSSC_I2C_RD_Byte();//高4位为湿度  低4位为温度
    Send_ACK();
    Byte_5th = ZSSC_I2C_RD_Byte();//温度字节
    Send_ACK();
    Byte_6th = ZSSC_I2C_RD_Byte();//温度字节
    Send_NOT_ACK();
    Stop_I2C();

    RetuData = (RetuData | Byte_2th) << 8;
    RetuData = (RetuData | Byte_3th) << 8;
    RetuData = (RetuData | Byte_4th);
    RetuData = RetuData >> 4;

    temp = (RetuData * 1000 / 1024 / 1024); //计算得到湿度值（放大了10倍,如果c1=523，表示现在湿度为52.3%）
    AHT10Value.humyH = (temp / 100) << 4 | (temp % 100 / 10) ;
    AHT10Value.humyL = temp % 100 % 10 << 4;

    RetuData = 0;
    RetuData = (RetuData | Byte_4th) << 8;
    RetuData = (RetuData | Byte_5th) << 8;
    RetuData = (RetuData | Byte_6th);
    RetuData = RetuData & 0xfffff;

    temp = (RetuData * 2000 / 1024 / 1024 - 500); //计算得到温度值（放大了10倍，如果t1=245，表示现在温度为24.5℃）
    AHT10Value.tempH = ((temp / 100) << 4) | (temp % 100 / 10) ;
    AHT10Value.tempL = temp % 100 % 10 << 4;
}


uint8_t AHT10_Init(void) { //初始化AHT10
    uint8_t	count;

		memset(&AHT10Value, 0, sizeof(AHT10Value));
    Init_I2C_Sensor_Port();
    delay_us(11038);

    I2C_Start();
    ZSSC_I2C_WR_Byte(0x70);
    Receive_ACK();
    ZSSC_I2C_WR_Byte(0xe1);//写系统配置寄存器
    Receive_ACK();
    ZSSC_I2C_WR_Byte(0x08);
    Receive_ACK();
    ZSSC_I2C_WR_Byte(0x00);
    Receive_ACK();
    Stop_I2C();

    delay_ms(500);//延时0.5S
    while(JH_Read_Cal_Enable() == 0) { //需要等待状态字status的Bit[3]=1时才去读数据。如果Bit[3]不等于1 ，发软件复位0xBA给AHT10，再重新初始化AHT10，直至Bit[3]=1

        JH_Send_BA();  //复位
        delay_ms(100);
        delay_us(11038);

        I2C_Start();
        ZSSC_I2C_WR_Byte(0x70);
        Receive_ACK();
        ZSSC_I2C_WR_Byte(0xe1);//写系统配置寄存器
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
        ret = AHT10_Init(); //初始化
        if(ret == 0) {
            //PutString("AHT10初始化失败\r\n");
            while(1);
        }
        //PutString("AHT初始化成功\r\n");
    }
    //while(1)
    {

        //DisableIrq(); //由于是模拟I2C,为读取数据更准确 ，读取之前禁止中断
        Read_AHT10();  //读取温度和湿度 ， 可间隔1.5S读一次
        //EnableIrq(); //恢复中断

        temp[0] = AHT10Value.tempH  ;
        temp[1] = AHT10Value.tempL ;
        humidity[0] = AHT10Value.humyH;
        humidity[1] = AHT10Value.humyL;

        /*	 PutString("AHT10-温度:");
        	 PutString(temp);
        	 PutString("    ");
        	 PutString("湿度:");
        	 PutString(humidity);
        	 PutString("\r\n");
        		*/
        delay_ms(1500); //延时1.5S
        //为读取的数据更稳定，还可以使用平均值滤波或者窗口滤波，或者前面读取的值与后面的值相差不能太大。
    }
}
#include "include.h"


uint16_t Cal_C[7];	        //���ڴ��PROM�е�8������
uint32_t D1_Pres,D2_Temp;	// ���ѹ�����¶�
float dT,TEMP;
double OFF_,SENS,T2;
float Pressure;				//����ѹ
float TEMP2,Aux,OFF2,SENS2;	//�¶�У��ֵ

uint32_t ex_Pressure;			//���ڶ���ת��ֵ
uint8_t exchange_num[8];

void delay_5us()
{
    nop();nop();nop();nop();
    nop();nop();nop();nop();
    nop();nop();nop();nop();
    nop();nop();nop();nop();
    nop();nop();nop();nop();
    nop();nop();nop();nop();
    nop();nop();nop();nop();
}

void delay_ms(uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {   
    nCount--;
  }
}
void ms5611_GPIO_init()
{

    GPIO_Init(CSB_GPIO_PORT, (GPIO_Pin_TypeDef)CSB_GPIO_PINS, GPIO_MODE_OUT_OD_HIZ_FAST); 
    GPIO_Init(SCL_GPIO_PORT, (GPIO_Pin_TypeDef)SCL_GPIO_PINS, GPIO_MODE_OUT_OD_HIZ_FAST);
    GPIO_Init(SDA_GPIO_PORT, (GPIO_Pin_TypeDef)SDA_GPIO_PINS, GPIO_MODE_OUT_OD_HIZ_FAST);

    GPIO_Init(SDO_GPIO_PORT, (GPIO_Pin_TypeDef)SDO_GPIO_PINS, GPIO_MODE_IN_FL_NO_IT);

    CSB_HIGH;
    SCL_LOW;

    

}

void send8(uchar data)
{
    uchar i; 
    uint8_t cmd = data;
    CSB_LOW; 
    SCL_LOW; 
    delay_5us();
    for(i=0;i<8;i++)
    {
        if(cmd & 0x80)
            SDA_HIGH;
        else
            SDA_LOW;
        SCL_HIGH;
        delay_5us();
        SCL_LOW;
        cmd = cmd<<1;
        delay_5us();
    }
    CSB_HIGH;
            
}

void convert(uchar data)
{
    uchar i; 
    uint8_t cmd = data;
    CSB_LOW; 
    SCL_LOW; 
    delay_5us();
    for(i=0;i<8;i++)
    {
        if(cmd & 0x80)
            SDA_HIGH;
        else
            SDA_LOW;
        SCL_HIGH;
        delay_5us();
        SCL_LOW;
        cmd = cmd<<1;
        delay_5us();
    }
    delay_ms(0xfff);
    CSB_HIGH;
    SDA_LOW;
            
}


uint16_t read_prom(uchar command)
{
    uchar i; 
    uint16_t seq = 0x0000;
    BitStatus datain=0;
    uint8_t cmd = command;
    CSB_LOW; 
    SCL_LOW; 
    delay_5us();
    for(i=0;i<8;i++)
    {
        if(cmd & 0x80)
            SDA_HIGH;
        else
            SDA_LOW;
        SCL_HIGH;
        delay_5us();
        SCL_LOW;
        cmd = cmd<<1;
        delay_5us();
    }
        SDA_LOW;
    for(i=0;i<16;i++)
    {
        seq = seq <<1;
        SCL_HIGH;
        delay_5us();
        datain = GPIO_ReadInputPin(SDO_GPIO_PORT,(GPIO_Pin_TypeDef)SDO_GPIO_PINS);
        if(datain)
            seq |= 0x01; 
        SCL_LOW;
        delay_5us();
        
    }
    CSB_HIGH;
    return seq;
}

//**************************************
//I2C��ʼ�ź�
//**************************************
void I2C_Start()
{
    SDA_HIGH;                    //����������
    SCL_HIGH;                    //����ʱ����
    delay_5us();                 //��ʱ
    SDA_LOW;                    //�����½���
    delay_5us();                 //��ʱ
    SCL_LOW;                    //����ʱ����
}

//**************************************
//I2Cֹͣ�ź�
//**************************************
void I2C_Stop()
{
    SDA_LOW;                    //����������
    SCL_HIGH;                    //����ʱ����
    delay_5us();                 //��ʱ
    SDA_HIGH;                    //����������
    delay_5us();                 //��ʱ
}
//**************************************
//I2C����Ӧ���ź�
//��ڲ���:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(uchar ack)
{
    
    if(ack)                  //дӦ���ź�
    {
        SDA_HIGH;
    }
    else
    {
        SDA_LOW;
    }
    SCL_HIGH;                    //����ʱ����
    delay_5us();                 //��ʱ
    SCL_LOW;                    //����ʱ����
    delay_5us();                 //��ʱ
}
//**************************************
//I2C����Ӧ���ź�
//**************************************
uint8_t I2C_RecvACK()
{
    uint8_t CY;
    SCL_HIGH;                    //����ʱ����
    delay_5us();                 //��ʱ
    CY = GPIO_ReadInputPin(SDA_GPIO_PORT,(GPIO_Pin_TypeDef)SDA_GPIO_PINS);                   //��Ӧ���ź�
    SCL_LOW;                    //����ʱ����
    delay_5us();                 //��ʱ
    return CY;
}
//**************************************
//��I2C���߷���һ���ֽ�����
//**************************************
void I2C_SendByte(uchar dat)
{
    uchar i;
    uchar cmd = dat;
    for (i=0; i<8; i++)         //8λ������
    {
        if(cmd & 0x80)          //�����ݿ�
            SDA_HIGH;
        else
            SDA_LOW;
        cmd <<= 1;              //�Ƴ����ݵ����λ               
        SCL_HIGH;                //����ʱ����
        delay_5us();             //��ʱ
        SCL_LOW;                //����ʱ����
        delay_5us();             //��ʱ
    }
    I2C_RecvACK();
}
//**************************************
//��I2C���߽���һ���ֽ�����
//**************************************
uchar I2C_RecvByte()
{
    uchar i;
    uchar dat = 0;
    uchar ldata;
    SDA_HIGH;                    //ʹ���ڲ�����,׼����ȡ����,
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        SCL_HIGH;                //����ʱ����
        delay_5us();             //��ʱ
        ldata = GPIO_ReadInputPin(SDA_GPIO_PORT,(GPIO_Pin_TypeDef)SDA_GPIO_PINS);
        if(ldata)
            dat |= 0x01;             //������       
        SCL_LOW;                //����ʱ����
        delay_5us();             //��ʱ
    }
    return dat;
}


void MSC5611_test()
{   
     volatile uint8_t lsdata;
     SDA_HIGH;
     lsdata = GPIO_ReadInputPin(GPIOC,(GPIO_Pin_TypeDef)GPIO_PIN_6);     
  
}

//=========================================================
//******MS561101BA����********
//=========================================================
//����MS5611
void MS561101BA_RESET()
{	
	I2C_Start();
	I2C_SendByte(MS561101BA_SlaveAddress);
//	I2C_RecvACK();
	I2C_SendByte(MS561101BA_RST);
//	I2C_RecvACK();
	I2C_Stop();
}

//
void MS561101BA_PROM_READ()
{
	uchar d1,d2,i;
	for(i=0;i<=6;i++)
	{
		I2C_Start();
		I2C_SendByte(MS561101BA_SlaveAddress);
		I2C_SendByte((MS561101BA_PROM_RD+i*2));


		I2C_Start();
		I2C_SendByte(MS561101BA_SlaveAddress+1);
		d1=I2C_RecvByte();
		I2C_SendACK(0);
		d2=I2C_RecvByte();
		I2C_SendACK(1);
		I2C_Stop();

		delay_5us();

		Cal_C[i]=((uint)d1<<8)|d2;
	}
}

uint32_t MS561101BA_DO_CONVERSION(uchar command)
{
	uint32_t conversion=0;
	uint32_t conv1,conv2,conv3; 
	I2C_Start();
	I2C_SendByte(MS561101BA_SlaveAddress);
	I2C_SendByte(command);
	I2C_Stop();

	delay_ms(0xffff);

	I2C_Start();
	I2C_SendByte(MS561101BA_SlaveAddress);
	I2C_SendByte(0);

	I2C_Start();
	I2C_SendByte(MS561101BA_SlaveAddress+1);
	conv1=I2C_RecvByte();
	I2C_SendACK(0);
	conv2=I2C_RecvByte();
	I2C_SendACK(0);
	conv3=I2C_RecvByte();

	I2C_SendACK(1);
	I2C_Stop();

	conversion=conv1*65535+conv2*256+conv3;

	return conversion;
}



void MS561101BA_getTemperature(uchar OSR_Temp)
{
    
	D2_Temp= MS561101BA_DO_CONVERSION(OSR_Temp);
	delay_ms(10); 
	dT=D2_Temp - (((uint32_t)Cal_C[5])<<8);
	TEMP=2000+dT*((uint32_t)Cal_C[6])/8388608;

}

void MS561101BA_getPressure(uchar OSR_Pres)
{
	
	D1_Pres= MS561101BA_DO_CONVERSION(OSR_Pres);
	delay_ms(10); 
	OFF_=(uint32_t)Cal_C[2]*65536+((uint32_t)Cal_C[4]*dT)/128;
	SENS=(uint32_t)Cal_C[1]*32768+((uint32_t)Cal_C[3]*dT)/256;
	
	if(TEMP<2000)
	{
		// second order temperature compensation when under 20 degrees C
		T2 = (dT*dT) / 0x80000000;
		Aux = TEMP*TEMP;
		OFF2 = 2.5*Aux;
		SENS2 = 1.25*Aux;
		TEMP = TEMP - TEMP2;
		OFF_ = OFF_ - OFF2;
		SENS = SENS - SENS2;	
	}

	Pressure=(D1_Pres*SENS/2097152-OFF_)/32768;
}


void MS5611_icc_init()
{
	MS561101BA_RESET();
	delay_ms(1000);
	MS561101BA_PROM_READ();
	delay_ms(1000);
}

void MS5611_INIT()
{
    delay_ms(1000);
    ms5611_GPIO_init();
    MS5611_icc_init();
}





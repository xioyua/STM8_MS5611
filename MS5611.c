#include "include.h"

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
void ms5611_init()
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
void I2C_SendACK(BitStatus ack)
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



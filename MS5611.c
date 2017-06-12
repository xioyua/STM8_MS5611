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
//I2C起始信号
//**************************************
void I2C_Start()
{
    SDA_HIGH;                    //拉高数据线
    SCL_HIGH;                    //拉高时钟线
    delay_5us();                 //延时
    SDA_LOW;                    //产生下降沿
    delay_5us();                 //延时
    SCL_LOW;                    //拉低时钟线
}

//**************************************
//I2C停止信号
//**************************************
void I2C_Stop()
{
    SDA_LOW;                    //拉低数据线
    SCL_HIGH;                    //拉高时钟线
    delay_5us();                 //延时
    SDA_HIGH;                    //产生上升沿
    delay_5us();                 //延时
}
//**************************************
//I2C发送应答信号
//入口参数:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(BitStatus ack)
{
    
    if(ack)                  //写应答信号
    {
        SDA_HIGH;
    }
    else
    {
        SDA_LOW;
    }
    SCL_HIGH;                    //拉高时钟线
    delay_5us();                 //延时
    SCL_LOW;                    //拉低时钟线
    delay_5us();                 //延时
}
//**************************************
//I2C接收应答信号
//**************************************
uint8_t I2C_RecvACK()
{
    uint8_t CY;
    SCL_HIGH;                    //拉高时钟线
    delay_5us();                 //延时
    CY = GPIO_ReadInputPin(SDA_GPIO_PORT,(GPIO_Pin_TypeDef)SDA_GPIO_PINS);                   //读应答信号
    SCL_LOW;                    //拉低时钟线
    delay_5us();                 //延时
    return CY;
}
//**************************************
//向I2C总线发送一个字节数据
//**************************************
void I2C_SendByte(uchar dat)
{
    uchar i;
    uchar cmd = dat;
    for (i=0; i<8; i++)         //8位计数器
    {
        if(cmd & 0x80)          //送数据口
            SDA_HIGH;
        else
            SDA_LOW;
        cmd <<= 1;              //移出数据的最高位               
        SCL_HIGH;                //拉高时钟线
        delay_5us();             //延时
        SCL_LOW;                //拉低时钟线
        delay_5us();             //延时
    }
    I2C_RecvACK();
}
//**************************************
//从I2C总线接收一个字节数据
//**************************************
uchar I2C_RecvByte()
{
    uchar i;
    uchar dat = 0;
    uchar ldata;
    SDA_HIGH;                    //使能内部上拉,准备读取数据,
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        SCL_HIGH;                //拉高时钟线
        delay_5us();             //延时
        ldata = GPIO_ReadInputPin(SDA_GPIO_PORT,(GPIO_Pin_TypeDef)SDA_GPIO_PINS);
        if(ldata)
            dat |= 0x01;             //读数据       
        SCL_LOW;                //拉低时钟线
        delay_5us();             //延时
    }
    return dat;
}



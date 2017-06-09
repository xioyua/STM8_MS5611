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

void ms5611_init()
{

    GPIO_Init(CSB_GPIO_PORT, (GPIO_Pin_TypeDef)CSB_GPIO_PINS, GPIO_MODE_OUT_PP_LOW_FAST); 
    GPIO_Init(SCL_GPIO_PORT, (GPIO_Pin_TypeDef)SCL_GPIO_PINS, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(SDA_GPIO_PORT, (GPIO_Pin_TypeDef)SDA_GPIO_PINS, GPIO_MODE_OUT_PP_LOW_FAST);

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
        if(data & 0x80)
            SDA_HIGH;
        else
            SDA_LOW;
        SCL_HIGH;
        delay_5us();
        SCL_LOW;
        cmd = cmd<<1;
        delay_5us();
    }
        
            
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
        if(command & 0x80)
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





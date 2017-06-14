#ifndef _MS56ZZ_H_
#define _MS56ZZ_H_

#define SCL_GPIO_PORT (GPIOC)
#define SCL_GPIO_PINS  (GPIO_PIN_5)

#define SDA_GPIO_PORT (GPIOC)
#define SDA_GPIO_PINS  (GPIO_PIN_6)

#define CSB_GPIO_PORT (GPIOB)
#define CSB_GPIO_PINS  (GPIO_PIN_4)

#define SDO_GPIO_PORT (GPIOC)
#define SDO_GPIO_PINS  (GPIO_PIN_7)

#define CSB_LOW GPIO_WriteLow(CSB_GPIO_PORT, (GPIO_Pin_TypeDef)CSB_GPIO_PINS)
#define CSB_HIGH GPIO_WriteHigh(CSB_GPIO_PORT, (GPIO_Pin_TypeDef)CSB_GPIO_PINS)

#define SDA_LOW GPIO_WriteLow(SDA_GPIO_PORT, (GPIO_Pin_TypeDef)SDA_GPIO_PINS)
#define SDA_HIGH GPIO_WriteHigh(SDA_GPIO_PORT, (GPIO_Pin_TypeDef)SDA_GPIO_PINS)

#define SCL_LOW GPIO_WriteLow(SCL_GPIO_PORT, (GPIO_Pin_TypeDef)SCL_GPIO_PINS)
#define SCL_HIGH GPIO_WriteHigh(SCL_GPIO_PORT, (GPIO_Pin_TypeDef)SCL_GPIO_PINS)


void MS5611_INIT(void);
void send8(uint8_t data);
uint16_t read_prom(uint8_t command);
void convert(uint8_t data);
void MSC5611_test(void);
void MS561101BA_getPressure(uint8_t OSR_Pres);
void MS561101BA_getTemperature(uint8_t OSR_Temp);



#define	MS561101BA_SlaveAddress   0xee	  //定义器件在IIC总线中的从地址 

#define	MS561101BA_D1      0x40
#define	MS561101BA_D2      0x50
#define	MS561101BA_RST     0x1E

//#define	MS561101BA_D1_OSR_256    0x40
//#define	MS561101BA_D1_OSR_512    0x42
//#define	MS561101BA_D1_OSR_1024   0x44
//#define	MS561101BA_D1_OSR_2048   0x46
#define	MS561101BA_D1_OSR_4096   0x48

//#define	MS561101BA_D2_OSR_256    0x50
//#define	MS561101BA_D2_OSR_512    0x52
//#define	MS561101BA_D2_OSR_1024   0x54
//#define	MS561101BA_D2_OSR_2048   0x56
#define	MS561101BA_D2_OSR_4096   0x58

#define MS561101BA_ADC_RD     0x00
#define	MS561101BA_PROM_RD 	  0xA0
#define MS561101BA_PROM_CRC   0xAE

static uint16_t Cal_C[7];	        //用于存放PROM中的8组数据
static uint32_t D1_Pres,D2_Temp;	// 存放压力和温度
static float dT,TEMP;
static double OFF_,SENS,T2;
static float Pressure;				//大气压
static float TEMP2,Aux,OFF2,SENS2;	//温度校验值

static uint32_t ex_Pressure;			//串口读数转换值
static uint8_t exchange_num[8];


#endif


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


void ms5611_init(void);
void send8(uint8_t data);
uint16_t read_prom(uint8_t command);




#endif


#ifndef U8G_STM32_HAL_H
#define U8G_STM32_HAL_H

#include "u8g.h"
#include "stm32f4xx_hal.h"

#ifndef F_CPU
#define F_CPU 168000000
#endif

void u8g_Delay(uint16_t val);	//Delay by "val" milliseconds
void u8g_MicroDelay(void);	//Delay be one microsecond
void u8g_10MicroDelay(void);	//Delay by 10 microseconds

void u8g_setSpiIface(SPI_HandleTypeDef * spih, GPIO_TypeDef  *CS_port, uint8_t CS_pin);
//main com function. read on...
uint8_t u8g_com_STM32_hal_hw_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);



#endif //U8G_STM32_HAL_H

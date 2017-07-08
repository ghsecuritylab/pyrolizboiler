#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

typedef struct _servoh
{
    TIM_HandleTypeDef * timh;
    GPIO_TypeDef* enableGPIO;
    uint16_t enablePin;
    osTimerId timer;
    osMutexId mutex;
}ServoHandler;


int initServo(ServoHandler * servoh);
void stopServo(ServoHandler * servoh);
void setServo(ServoHandler * servoh, uint8_t num, uint8_t degree);





#endif

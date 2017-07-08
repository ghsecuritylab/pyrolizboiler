#include "acpowerctrl.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#define RELAY_NUM 4

static inline void setPin(GPIO_TypeDef* port, uint16_t pin, bool state)
{
    if(state) HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

void setRelay(uint8_t num, bool state)
{
    if(num>=RELAY_NUM) return;

    switch (num) {
    case 0: setPin(Relay0_GPIO_Port, Relay0_Pin, state); break;
    case 1: setPin(Relay1_GPIO_Port, Relay1_Pin, state); break;
    case 2: setPin(Relay2_GPIO_Port, Relay2_Pin, state); break;
    case 3: setPin(Relay3_GPIO_Port, Relay3_Pin, state); break;
    }
}

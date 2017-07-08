#include "servo.h"
#include "stm32f4xx_hal_tim.h"


#define TIME_SERVO_POWERON_MS 5000
#define SERVOS_NUM 3

#define servosOn(handler) HAL_GPIO_WritePin(handler->enableGPIO,handler->enablePin,GPIO_PIN_SET)
#define servosOff(handler) HAL_GPIO_WritePin(handler->enableGPIO,handler->enablePin,GPIO_PIN_RESET)

static inline uint16_t degreeToPulse(u_int16_t deg)
{
    return ((deg*1000)/180)+1000;
}

static inline uint32_t chanToHalChan(uint8_t num)
{
    return num*4;
}

static void servoEnd(void const* h)
{
    ServoHandler *servoh = (ServoHandler*)h;
    servosOff(servoh);
}

osMutexDef(servosAcces);
osTimerDef(servosPower, servoEnd);

int initServo(ServoHandler *servoh)
{
    servoh->mutex = osMutexCreate(osMutex(servosAcces));
    if(servoh->mutex == NULL)
        return 0;
    servoh->timer = osTimerCreate(osTimer(servosPower), osTimerOnce, (void *)servoh);
    if(servoh->timer == NULL)
        return 0;

    HAL_TIM_Base_Start(servoh->timh);
    HAL_TIM_PWM_Start(servoh->timh,TIM_CHANNEL_ALL);

    return 1;
}

void stopServo(ServoHandler *servoh)
{
    HAL_TIM_PWM_Stop(servoh->timh,TIM_CHANNEL_ALL);
    HAL_TIM_Base_Stop(servoh->timh);

    osTimerDelete(servoh->timer);
    osMutexDelete(servoh->mutex);
}

void setServo(ServoHandler *servoh, uint8_t num, uint8_t degree)
{
    if(num>=SERVOS_NUM)
        return;

    osMutexWait(servoh->mutex, osWaitForever);

    __HAL_TIM_SET_COMPARE(servoh->timh,num*4,degreeToPulse(degree));
    servosOn(servoh);
    osTimerStart(servoh->timer, TIME_SERVO_POWERON_MS);

    osMutexRelease(servoh->mutex);

}

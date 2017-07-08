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

void servoEnd(void const* h)
{
    //servosOff(((ServoHandler*)h));
    HAL_GPIO_WritePin(Servo_EN_GPIO_Port,Servo_EN_Pin, GPIO_PIN_RESET);
}

osMutexDef(servosAcces);
osTimerDef(servosPower, servoEnd);

int initServo(ServoHandler *servoh)
{
    servoh->mutex = osMutexCreate(osMutex(servosAcces));
    if(servoh->mutex == NULL)
        return 0;
    servoh->timer = osTimerCreate(osTimer(servosPower), osTimerOnce, servoh);
    if(servoh->timer == NULL)
        return 0;

    HAL_TIM_PWM_Start(servoh->timh,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(servoh->timh,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(servoh->timh,TIM_CHANNEL_3);

    return 1;
}

void stopServo(ServoHandler *servoh)
{
    HAL_TIM_PWM_Stop(servoh->timh,TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(servoh->timh,TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(servoh->timh,TIM_CHANNEL_1);

    osTimerDelete(servoh->timer);
    osMutexDelete(servoh->mutex);
}

void setServo(ServoHandler *servoh, uint8_t num, uint8_t degree)
{
    if(num>=SERVOS_NUM)
        return;

    osMutexWait(servoh->mutex, osWaitForever);

    __HAL_TIM_SET_COMPARE(servoh->timh,chanToHalChan(num),degreeToPulse(degree));
    servosOn(servoh);
    osTimerStart(servoh->timer, TIME_SERVO_POWERON_MS);

    osMutexRelease(servoh->mutex);
}

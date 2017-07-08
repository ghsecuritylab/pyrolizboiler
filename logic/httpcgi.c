#include "httpcgi.h"
#include "lwip/apps/httpd.h"
#include "string.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "servo.h"

tCGI cgih[2];
ServoHandler serv;
//tCGI *cgih;
ServoHandler * servh;
extern TIM_HandleTypeDef htim4;

static const char* leds_cgi_handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[]);
static const char* servo_cgi_handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[]);


void initHttpCgiServer()
{

    servh = &serv; //malloc(sizeof (ServoHandler));
    servh->timh = &htim4;
    servh->enableGPIO = Servo_EN_GPIO_Port;
    servh->enablePin = Servo_EN_Pin;
    initServo(servh);

    //cgih = malloc(2*sizeof(tCGI));
    cgih[0].pcCGIName = "/leds";
    cgih[0].pfnCGIHandler = leds_cgi_handler;
    cgih[1].pcCGIName = "/servo";
    cgih[1].pfnCGIHandler = servo_cgi_handler;

    http_set_cgi_handlers((const tCGI*)&cgih, 2);
    httpd_init();
}


static const char* leds_cgi_handler(int iIndex,
                                    int iNumParams,
                                    char *pcParam[],
                                    char *pcValue[])
{
    for(uint8_t i = 0; i<iNumParams; i++)
    {

        if(0==strcmp(pcParam[i], "led1"))
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        else if(0==strcmp(pcParam[i], "led10"))
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
        else if(0==strcmp(pcParam[i], "led2"))
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
        else if(0==strcmp(pcParam[i], "led20"))
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
        else if(0==strcmp(pcParam[i], "led3"))
            HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
        else if(0==strcmp(pcParam[i], "led30"))
            HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
        else if(0==strcmp(pcParam[i], "led4"))
            HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
        else if(0==strcmp(pcParam[i], "led40"))
            HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET);
    }
    return "";//"/index.html";
}

static const char* servo_cgi_handler(int iIndex,
                                     int iNumParams,
                                     char *pcParam[],
                                     char *pcValue[])
{
    int num, val;
    for(uint8_t i = 0; i<iNumParams; i++)
        if(sscanf(pcParam[i], "pwm%d", &num))
            if(sscanf(pcValue[i], "%d", &val))
                setServo(servh, num, val);

    return "";
}



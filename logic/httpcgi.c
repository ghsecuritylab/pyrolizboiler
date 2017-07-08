#include "httpcgi.h"
#include "lwip/apps/httpd.h"
#include "string.h"
#include "main.h"
#include "stm32f4xx_hal.h"

tCGI *cgih;

static const char* leds_cgi_handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[]);
static const char* servo_cgi_handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[]);


void initHttpCgiServer()
{
    cgih = malloc(2*sizeof(tCGI));
    cgih[1].pcCGIName = "/leds";
    cgih[1].pfnCGIHandler = leds_cgi_handler;
    cgih[2].pcCGIName = "/servo";
    cgih[2].pfnCGIHandler = servo_cgi_handler;

    http_set_cgi_handlers(&cgih, sizeof(cgih));
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
    for(uint8_t i = 0; i<iNumParams; i++)
    {

    }
    return "";
}



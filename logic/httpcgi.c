#include "httpcgi.h"
#include "lwip/apps/httpd.h"
#include "string.h"
#include "main.h"
#include "stm32f4xx_hal.h"

static const char* cgi_handler(int iIndex,
                               int iNumParams,
                               char *pcParam[],
                               char *pcValue[])
{
    if(iIndex!=0) return "/index.html";

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

    return "/index.html";
}

tCGI cgih = {"/leds.cgi", cgi_handler};

void initHttpCgiServer()
{
    http_set_cgi_handlers(&cgih, 1);
    httpd_init();
}

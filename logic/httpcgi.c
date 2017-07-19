#include "httpcgi.h"
#include "lwip/apps/httpd.h"
#include "string.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#include "servo.h"
#include "acpowerctrl.h"
#include "max31855_stm32_hal.h"


ServoHandler * servh;
extern TIM_HandleTypeDef htim4;
extern max31855_h maxtc;

#define SSI_TAGS_NUM 2
const char *tags[SSI_TAGS_NUM];
static u16_t SSIHandler(int iIndex, char *pcInsert, int iInsertLen);

#define CGI_HEADERS_NUM 4
tCGI *cgih;
static const char* leds_cgi_handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[]);
static const char* servo_cgi_handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[]);
static const char* acpower_cgi_handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[]);
static const char* triac_cgi_handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[]);


void initHttpCgiServer()
{
    /*** servo init ***/
    servh = (ServoHandler*)malloc(sizeof(ServoHandler));
    if(servh==NULL) return;
    servh->timh = &htim4;
    servh->enableGPIO = Servo_EN_GPIO_Port;
    servh->enablePin = Servo_EN_Pin;
    initServo(servh);
    /*****/

    /*** register cgi handlers ***/
    cgih = (tCGI*)calloc(CGI_HEADERS_NUM, sizeof(tCGI));
    if(cgih==NULL) return;

    cgih[0].pcCGIName = "/leds";
    cgih[0].pfnCGIHandler = leds_cgi_handler;
    cgih[1].pcCGIName = "/servo";
    cgih[1].pfnCGIHandler = servo_cgi_handler;
    cgih[2].pcCGIName = "/relay";
    cgih[2].pfnCGIHandler = acpower_cgi_handler;
    cgih[3].pcCGIName = "/triac";
    cgih[3].pfnCGIHandler = triac_cgi_handler;

    http_set_cgi_handlers(cgih, CGI_HEADERS_NUM);
    /***/

    /*** register ssi handler and tags ***/
    tags[0] = "tc";
    tags[1] = "slf";
    http_set_ssi_handler(&SSIHandler, tags, SSI_TAGS_NUM);
    /*****/

    /*** server main init ***/
    httpd_init();
    /*****/
}

static u16_t SSIHandler(int iIndex, char *pcInsert, int iInsertLen)
{
        if(0==strcmp(tags[iIndex], "tc"))
            return snprintf(pcInsert, iInsertLen, "%d.%0d",
                            (int16_t)max31855_getTemp(&maxtc),
                            (int16_t)(max31855_getTemp(&maxtc)*10)%10);

        else if(0==strcmp(tags[iIndex], "slf"))
            return snprintf(pcInsert, iInsertLen, "%d.%0d",
                            (int16_t)max31855_getSelfTemp(&maxtc),
                            (int16_t)(max31855_getSelfTemp(&maxtc)*10)%10);

        return 0;


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

static const char* acpower_cgi_handler(int iIndex,
                                       int iNumParams,
                                       char *pcParam[],
                                       char *pcValue[])
{
    int num; char b[8];
    for(uint8_t i = 0; i<iNumParams; i++)
        if(sscanf(pcParam[i], "relay%d", &num))
            setRelay(num, sscanf(pcValue[i], "%[TtRrUuEe]", b));


    return "";
}

static const char* triac_cgi_handler(int iIndex,int iNumParams,char *pcParam[],char *pcValue[])
{
    int val;
    if(sscanf(pcValue[0], "%d", &val))
        setTriac(val);
    return "";
}


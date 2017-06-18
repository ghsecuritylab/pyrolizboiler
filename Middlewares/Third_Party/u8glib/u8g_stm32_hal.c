#include "u8g_stm32_hal.h"

SPI_HandleTypeDef *spi;
GPIO_TypeDef  *port;
uint8_t pin;

#define CS_ON()        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)   //Аппаратный CS
#define CS_OFF()       HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
void SPI_Out_LCD12864_Ap(uint8_t data)
{
    HAL_SPI_Transmit(spi, &data, 1, 100);
}
void LCD_OUT(uint8_t data, uint8_t ctrl)
{
    CS_ON();                               //Передача начата.
    SPI_Out_LCD12864_Ap (ctrl);
    SPI_Out_LCD12864_Ap( data & 0xF0);   //Старшая половина и 4 "0".
    SPI_Out_LCD12864_Ap( data << 4);     //Младшая и 4 "0".
    CS_OFF();
}

uint8_t u8g_com_STM32_hal_hw_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
    static uint8_t control;

    switch(msg)
    {
    case U8G_COM_MSG_STOP:
        break;

    case U8G_COM_MSG_INIT:
        break;

    case U8G_COM_MSG_ADDRESS:
        //SWITCH FROM DATA TO COMMAND MODE (arg_val == 0 for command mode)
        //управляющий байт переключает режим на устройстве и устанавливается здесь
        // define cmd (arg_val = 0) or data mode (arg_val = 1)
        // cmd - передача команды ,data mode - режим передачи данных
        if (arg_val == 0)
            control = 0xF8;
        else
            control = 0xFA;
        u8g_10MicroDelay();
        break;

    case U8G_COM_MSG_RESET:
        //TOGGLE THE RESET PIN ON THE DISPLAY BY THE VALUE IN arg_val
        break;

    case U8G_COM_MSG_WRITE_BYTE:
        //WRITE BYTE TO DEVICE
        //ЗАПИСЬ байта НА УСТРОЙСТВО
        LCD_OUT (arg_val, control);
        u8g_MicroDelay();
        break;

    case U8G_COM_MSG_WRITE_SEQ:
    case U8G_COM_MSG_WRITE_SEQ_P:
        //WRITE A SEQUENCE OF BYTES TO THE DEVICE
        //ЗАПИСЬ ПОСЛЕДОВАТЕЛЬНОСТИ БАЙТОВ НА УСТРОЙСТВО
        //register uint8_t *ptr = arg_ptr;
        CS_ON();                               //Передача начата.
        SPI_Out_LCD12864_Ap (control);
        while( arg_val > 0 )
        {
            SPI_Out_LCD12864_Ap( *(uint8_t*)arg_ptr & 0xF0);   //Старшая половина и 4 "0".
            SPI_Out_LCD12864_Ap( *(uint8_t*)arg_ptr << 4);     //Младшая и 4 "0".
            //*
            arg_ptr++;
            arg_val--;
        }
        CS_OFF();
        u8g_MicroDelay();
        break;
    }
    return 1;
}

void u8g_setSpiIface(SPI_HandleTypeDef *spih, GPIO_TypeDef  *CS_port, uint8_t CS_pin)
{
    spi = spih;
    port = CS_port;
    pin = CS_pin;
}

void u8g_Delay(uint16_t val)
{
    HAL_Delay(val);
}

void u8g_MicroDelay()
{
    for(uint8_t i = 0; i<F_CPU/1000000; i++) __NOP();
}

void u8g_10MicroDelay()
{
    for(uint8_t i = 0; i<10; i++) u8g_MicroDelay();
}

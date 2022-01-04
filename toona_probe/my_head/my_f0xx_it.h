#ifndef MY_F0XX_IT_H
#define MY_F0XX_IT_H

#include "stm32f0xx_hal.h"

#ifndef UART_BUF_LEN
#define UART_BUF_LEN		(256)
#define UART_BUF_MSK		(255)
#endif

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim14;

void Start_UART_RX_IT(UART_HandleTypeDef *huart);

void printf_UART1(const char *fmt, ...);
#define my_printf(...) printf_UART1(__VA_ARGS__) 




extern unsigned char aRx1Buf[UART_BUF_LEN];
extern volatile unsigned short aRx1_head;
extern volatile unsigned short aRx1_tail;

#endif

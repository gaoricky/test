#ifndef MY_MAIN_H
#define MY_MAIN_H

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"

#include "my_f0xx_it.h"
/////////////////////////////////////////////////////////
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

//电源开关相关， 
#define GPIO_POWER		GPIOA  
#define PIN_POWER		GPIO_PIN_12  


void my_main(void);


#endif


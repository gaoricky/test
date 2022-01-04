#include "my_main.h"


//定时器的中断初始化
static void my_perip_init(void)
{

#if 1  //测试目的，关闭 pwm
	//ricky 更改后的
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	//htim1.Instance->CCER |= 1<<2;		
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);  //替换上面注释掉的语句

	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);   
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
#endif	

}

static void my_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

#if 0	
	//pwr_on output control
	GPIO_InitStruct.Pin = PIN_POWER;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // ori GPIO_SPEED_FREQ_HIGH GPIO_SPEED_FREQ_LOW
	HAL_GPIO_Init(GPIO_POWER, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIO_POWER,PIN_POWER,GPIO_PIN_RESET);	//高电平供电，开机以前关电              GPIO_PIN_RESET
#endif	
		
}

extern unsigned char is_1s;
void my_main(void)
{
	my_perip_init();

	Start_UART_RX_IT(&huart1);
	
	
	while(1)
	{

		#if 1
		if(is_1s == 1)
		{
			is_1s = 0;
			//my_printf("stm32f030k6 %s\r\n" __DATE__);
		}
		#endif
	}
}
	

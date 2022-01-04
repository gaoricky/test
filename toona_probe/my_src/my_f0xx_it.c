// stm32f1xx_it.c   user code

#include "stm32f0xx_hal.h"
#include "stm32f0xx_it.h"

#include "stdio.h"
#include "stdarg.h"
#include "string.h"

#include "my_f0xx_it.h"



unsigned char aRx1Buf[UART_BUF_LEN];
volatile unsigned short aRx1_head = 0,aRx1_tail = 0;

unsigned char aTx1Buf[UART_BUF_LEN];
volatile unsigned short aTx1_head = 0,aTx1_tail = 0;





void Start_UART_RX_IT(UART_HandleTypeDef *huart)
{
//	if(!(huart->Instance->CR1 & 0x20))
		__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}
void Stop_UART_RX_IT(UART_HandleTypeDef *huart)
{
	__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
}
void Start_UART_TX_IT(UART_HandleTypeDef *huart)
{
	//__HAL_UART_ENABLE_IT(huart, UART_IT_TXE);  UART_IT_TC
	__HAL_UART_ENABLE_IT(huart, UART_IT_TC);  
}
void Stop_UART_TX_IT(UART_HandleTypeDef *huart)
{
	//__HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
	__HAL_UART_DISABLE_IT(huart, UART_IT_TC);
}


void Put2UartBufferAndSend(UART_HandleTypeDef *pUart,uint8_t *buf, uint16_t n)
{
	unsigned short i;
	
	if(!pUart)
		return ;	// invalid port
	if(pUart == &huart1)
	{
		for(i = 0; i < n; i ++)
		{
			//ricky 缓冲区满，就一直等到有数据被取走时。
			while(((aTx1_head + 1) & UART_BUF_MSK) == aTx1_tail)  
			{
				if(!(pUart->Instance->CR1 & 0x40))
					Start_UART_TX_IT(pUart);
			}
			aTx1Buf[aTx1_head] = buf[i];
			aTx1_head ++;
			aTx1_head &= UART_BUF_MSK;
		}
		Start_UART_TX_IT(pUart);
	}

}


void printf_UART1(const char *fmt, ...)
{
	unsigned short len;
	va_list ap;
	char prntstring[128];
		
    va_start(ap, fmt);
    vsprintf(prntstring, fmt, ap);
   	va_end(ap);
	// insert \r 0x0D for \n 0x0A

	len = strlen((const char *)prntstring);
	Put2UartBufferAndSend(&huart1,(uint8_t *)prntstring,len);
}



void USART1_IRQHandler(void)
{
	/* USER CODE BEGIN USART1_IRQn 0 */
	uint32_t stat;	// USART1 state


	/* USER CODE END USART1_IRQn 0 */
//	HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */
	stat = huart1.Instance->ISR;

	//接收
	if(stat & 0x20)
	{
		//ricky  RXNE标志不能直接清除，读取USART->DR（就是读取接收到的数据）来清除RXNE标志
		aRx1Buf[aRx1_head++] = (uint8_t)(huart1.Instance->RDR); //RDR
		aRx1_head = aRx1_head & (UART_BUF_MSK);

		//ricky add 缓冲区已满？则将 aRx1_tail 右移一位
		if( aRx1_tail == aRx1_head)
		{
			aRx1_tail++;
			aRx1_tail &= UART_BUF_MSK;
		}
			
	}

	//发送
	else if((huart1.Instance->CR1 & 0x40) && (stat & 0xC0))
	{// TX complete
		if(aTx1_tail != aTx1_head)
		{
			/* ricky 将数据给到串口的数据寄存器; 发送和接收都是 Instance->DR 
			 * ！！！参考：https://blog.csdn.net/qq_40791635/article/details/83038678
			 * RDR(接收) 和 TDR(发送) 寄存器表示的物理空间是一样的,都是 DR。（发送之前处理接收，就不会有冲突） 
			 * 只要发送寄存器为空，就会一直有中断
			 */
			huart1.Instance->TDR = (uint32_t)(aTx1Buf[aTx1_tail]);  //TDR    
			aTx1_tail ++;
			aTx1_tail &= UART_BUF_MSK;
		}
		else
		{
			Stop_UART_TX_IT(&huart1);
		}
	}
	/* USER CODE END USART1_IRQn 1 */
}


#if 0
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}
#endif




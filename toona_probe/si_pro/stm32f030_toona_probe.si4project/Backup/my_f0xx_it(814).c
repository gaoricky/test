// stm32f1xx_it.c   user code

#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"

#include "stdio.h"
#include "stdarg.h"
#include "string.h"

#include "my_adpcm.h"
#include "my_f1xx_it.h"
#include "my_main.h"
#include "arm_math.h"

#include "fft_ifft.h"





volatile int cnt_200ms;
volatile int cnt_bt_link;  //蓝牙处于连接状态为高电平，但是蓝牙模块刚上电时，有150ms的高电平输出，需滤除掉
volatile unsigned int  cnt_bat_charge = 0;
volatile unsigned int  cnt_bat_low = 0;




int cnt_tim2_power_off;




extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern ADC_HandleTypeDef hadc2;

#define ADCFTLEN_MSK	(1024-1)
unsigned short adcFtBuf[1024];  //2048

//测试 #define ADCFTLEN_MSK	(512-1)
//测试 unsigned short adcFtBuf[512];  //2048`


unsigned int adcFtHead,adcFtTail;

unsigned int nSlidePoint;
unsigned int nBtHead,nBtTail;

int AlgMin1, AlgMin2, AlgMax1, AlgMax2;
int risefall,noBeatTime,AdcAvgCnt;
unsigned int AdcAvgSum;
int sld_sum1, sld_pt1;
int beat_head,beat_tail, beat_tail2;
int slpHead, slpTail;
int sld_buf1[SLD_BUF1_LEN];

int sld_twice_sum ;  //平滑累加值
int sld_twice_pt;  //sld_buf_twice的指针
int sld_buf_twice[SLD_BUFTWICE_LEN];  //再平滑一次
unsigned char is_peak_available;


signed short  sld_buf2[BEAT_N_LEN];


unsigned int adcTime;
unsigned short diff1, diff2;
int goodwave;
unsigned short beat = 0;  //ricky 可能会大于256
void Tim3GetRunFtHeart(void);	// timer 3 interrupt call

unsigned int gAdpcmNibble,gAdpcmCnt,btSeq;
signed short gAdpcmData;
unsigned char cAdpcmBuf[256];

volatile int cnt_tim2_after_cmd = 0;
volatile int cnt_tim2_scan_expire = 0;

uint8_t aRx1Buf[UART_BUF_LEN];
volatile int32_t aRx1_head = 0,aRx1_tail = 0;

uint8_t aTx1Buf[UART_BUF_LEN];
volatile int32_t aTx1_head = 0,aTx1_tail = 0;

uint8_t aRx3Buf[UART_BUF_LEN];
volatile unsigned short aRx3_head = 0,aRx3_tail = 0;


uint8_t aTx3Buf[UART_BUF_LEN];
volatile  unsigned short  aTx3_head = 0,aTx3_tail = 0;

#ifdef OUTPUT_FINAL_DATAS  //输出平滑后的波形数据
static unsigned char is_final_dada_available = 0;
unsigned short final_data = 0;
#endif





//rickky add
unsigned char g_first_beat = 1;
unsigned short ori_samp;
unsigned short ori_samp_max;





/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;





/**
* @brief This function handles TIM2 global interrupt.
*/


void TIM2_IRQHandler(void)
{
	/* USER CODE BEGIN TIM2_IRQn 0 */

	/* USER CODE END TIM2_IRQn 0 */
	HAL_TIM_IRQHandler(&htim2);
	/* USER CODE BEGIN TIM2_IRQn 1 */
	cnt_tim2_power_off++;
	cnt_tim2_after_cmd++;
	cnt_tim2_scan_expire++;
#ifndef VERSION_V2	
	cnt_200ms++;
#endif
	cnt_bt_link++;
	cnt_bat_charge++;
	cnt_bat_low++;
#ifdef CHARGE_VOLTAGE_MODE
#endif



	/* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
	/* USER CODE BEGIN TIM3_IRQn 0 */

	/* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);
	/* USER CODE BEGIN TIM3_IRQn 1 */
	Tim3GetRunFtHeart();

#if 0  //测试中断时间是否正确
	static int cnt_tim3 = 0;
	cnt_tim3 ++;
	if(cnt_tim3 == 2000)
	{
		cnt_tim3 = 0;
		HAL_GPIO_TogglePin(GPIO_LED, PIN_LED_GREEN);
	}
#endif	
	/* USER CODE END TIM3_IRQn 1 */
}


/**
* @brief This function handles USART1 global interrupt.
*/


void USART1_IRQHandler(void)
{
	/* USER CODE BEGIN USART1_IRQn 0 */
	uint32_t stat;	// USART1 state


	/* USER CODE END USART1_IRQn 0 */
//	HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */
	stat = huart1.Instance->SR;

	//接收
	if(stat & 0x20)
	{
		//ricky  RXNE标志不能直接清除，读取USART->DR（就是读取接收到的数据）来清除RXNE标志
		aRx1Buf[aRx1_head++] = (uint8_t)(huart1.Instance->DR);
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
			huart1.Instance->DR = (uint32_t)(aTx1Buf[aTx1_tail]);  
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

/**
* @brief This function handles USART3 global interrupt.
*/


void USART3_IRQHandler(void)
{
	/* USER CODE BEGIN USART3_IRQn 0 */
	uint32_t stat;	// USART1 state

	/* USER CODE END USART3_IRQn 0 */
//	HAL_UART_IRQHandler(&huart3);
	/* USER CODE BEGIN USART3_IRQn 1 */
	stat = huart3.Instance->SR;
	if(stat & 0x20)
	{
		aRx3Buf[aRx3_head++] = (uint8_t)(huart3.Instance->DR);
		aRx3_head = aRx3_head & (UART_BUF_MSK);
	}
	else if((huart3.Instance->CR1 & 0x40) && (stat & 0xC0))
	{// TX complete
		//huart3.Instance->SR &= ~0xC0;
	//	if((huart3.Instance->SR & 0xC0) && (huart3.Instance->SR & 0xC0) && (huart3.Instance->SR & 0xC0))
		{
			if(aTx3_tail != aTx3_head)
			{
				huart3.Instance->DR = (uint32_t)(aTx3Buf[aTx3_tail]);
				aTx3_tail++;
				aTx3_tail = aTx3_tail & UART_BUF_MSK;
			}
			else
			{
				Stop_UART_TX_IT(&huart3);
			}
		}
	}
	
	/* USER CODE END USART3_IRQn 1 */
}



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


void Put2UartBufferAndSend(UART_HandleTypeDef *pUart,uint8_t *buf,int32_t n)
{
	int32_t i;
	
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
	else if(pUart == &huart3)
	{
		for(i = 0; i < n; i ++)
		{
			while(((aTx3_head + 1) & UART_BUF_MSK) == aTx3_tail)
			{
				if(!(pUart->Instance->CR1 & 0x40))
					Start_UART_TX_IT(pUart);
			}
			aTx3Buf[aTx3_head] = buf[i];
			aTx3_head ++;
			aTx3_head &= UART_BUF_MSK;
		}
		Start_UART_TX_IT(pUart);
	}
}

void printf_UART3(const char *fmt, ...)
{
	int32_t len;
	va_list ap;
	char prntstring[128];
		
    va_start(ap, fmt);
    vsprintf(prntstring, fmt, ap);
   	va_end(ap);

	len = strlen((const char *)prntstring);
	Put2UartBufferAndSend(&huart3,(uint8_t *)prntstring,len);
	//Put2UartBufferAndSend(&huart3,(uint8_t *)prntstring,len);
}

void printf_UART1(const char *fmt, ...)
{
	int32_t len;
	va_list ap;
	char prntstring[128];
		
    va_start(ap, fmt);
    vsprintf(prntstring, fmt, ap);
   	va_end(ap);
	// insert \r 0x0D for \n 0x0A

	len = strlen((const char *)prntstring);
	Put2UartBufferAndSend(&huart1,(uint8_t *)prntstring,len);
	//Put2UartBufferAndSend(&huart3,(uint8_t *)prntstring,len);
}


void UART1OutBinCode(uint8_t *pd, uint16_t len)
{
	Put2UartBufferAndSend(&huart1,pd,len);
}

void UART3OutBinCode(uint8_t *pd, uint16_t len)
{
	Put2UartBufferAndSend(&huart3,pd,len);
}



//此时还没开 timer3 中断
void ResetVariables_it(void)
{	
	adcFtHead = 0;
	adcFtTail = 0;
	nSlidePoint = 0;
	nBtHead = 0;
	nBtTail = 0;
}


//定时时间是 0.5ms
void Tim3GetRunFtHeart(void)	// timer 3 interrupt call
{
	//ricky adc 数据已准备好？
	//if(HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_EOC)  //此条件判断，必须先执行 HAL_ADC_PollForConversion; 与下面语句等效
	//if(HAL_ADC_PollForConversion(&hadc1,0) == HAL_OK)  //不做判断
	{
		adcFtBuf[adcFtHead] = HAL_ADC_GetValue(&hadc1);
	
		adcFtHead ++;  //ricky 注意：主循环中用到的全局变量，使用时，需禁止 timer3 中断
	
		//adcFtHead 到 2048 时，归零
		adcFtHead &= ADCFTLEN_MSK;

		//ricky 需要判断  adcFtHead+1的位置，如果与          adcFtTail 位置重叠，将 adcFtTail 右移一位，以免当作数据为空
		// 头尾应该不会相差1个缓冲区的大小，暂时取消
#if 0	
		if(adcFtHead == adcFtTail)
		{
			adcFtTail ++;  //ricky 注意：主循环中用到的全局变量，使用时，需禁止 timer3 中断
			adcFtTail &= ADCFTLEN_MSK;
		}	
#endif
	}
	
	//ricky adc 开始转换
	HAL_ADC_Start(&hadc1);	// run fetal beat ADC sample
}

static inline int GetFtDataLength(void)
{
	unsigned int ret;
	
	unsigned int cache_adcFtHead = adcFtHead;  //尽量与中断同步
	
	if(cache_adcFtHead >= adcFtTail)
	{
		ret = cache_adcFtHead - adcFtTail;
	}
	else
	{
		ret = ADCFTLEN_MSK + 1 + cache_adcFtHead - adcFtTail;
	}
	return ret;	
}

void stop_ft_adc(void)
{
	HAL_TIM_Base_Stop_IT(&htim3);  //定时器3 关闭中断
	HAL_ADC_Stop(&hadc1);	// run fetal beat ADC sample
	adcFtHead = 0;
	adcFtTail = 0;
}





void FetalHeartRate_Sound(void)
{
	int j, i;
	uint8_t *p8, sum;
	
	//q15_t data_in;
	//q15_t data_out;

	q31_t data_in;
	q31_t data_out;

	pBtPack pBk = (pBtPack)cAdpcmBuf;

	//GetFtDataLength 获得 保存在 adcFtBuf 环形缓冲区中的数据长度; adcFtHead 是数据更新到的位置

	// 重要！！！ 除了上电时，采样16次*0.5ms==8ms, 以后每次采样8个数据，则每隔 8*0.5ms == 4ms 进入下面的for循环。
	if(GetFtDataLength() < 16)  //数据太少，不处理
		return;
	// calculate Fetal heart rate at first, length = 128

	//数据太多，则死循环 ？？？
	//if(GetFtDataLength() > 1020)
	{

	//#ifdef DEBUG_HR
	//	printf_UART1("dead ............................\r\n");
	//#endif
	//	while(1);
	}

	//处理掉8个采样数据
	for(j = 0; j < 8; j++)
	{
		CalculateBeat(adcFtBuf[adcFtTail]);

		/* 下面的判断 只是判断 gAdpcmNibble 是奇数或偶数 ？？？，并且 gAdpcmNibble 没有其它作用
		 *  下面的语句，除了 adcFtTail ++; 只是为了将数据编码，打包数据，和发送数据
		 */
		#ifdef OUTPUT_FINAL_DATAS  //输出平滑后的波形数据
		if(is_final_dada_available == 1)
		#else
		if(gAdpcmNibble & 1)
		#endif
		{
			#ifdef OUTPUT_FINAL_DATAS  //输出平滑后的波形数据
			is_final_dada_available = 0;
			pBk->wave[gAdpcmCnt++] = final_data;
			#else
			//将 ADPCM编码的 index 和 predsample 传输到PC端；
			if(gAdpcmCnt == 0)
			{
				pBk->predsample = predsample;
				pBk->index = index;
			}
			
		#if 1
			//低通/带通滤波 >>>
			data_in = gAdpcmData;
			do_fir(&data_in, &data_out);
			gAdpcmData = data_out;	

			data_in = adcFtBuf[adcFtTail];
			do_fir(&data_in, &data_out);			
			
			//低通/带通滤波 <<<
	
			pBk->wave[gAdpcmCnt++] = ADPCM_Encode(gAdpcmData) + (ADPCM_Encode(data_out) << 4);  
		#else
			
			pBk->wave[gAdpcmCnt++] = ADPCM_Encode(gAdpcmData) + (ADPCM_Encode(adcFtBuf[adcFtTail]) << 4);  //编码时，不放大原始采样数据
		#endif
			
			gAdpcmData = 0;
			#endif

			//有了一个包的数据，则发送数据到串口
			if(gAdpcmCnt >= sizeof(pBk->wave)/sizeof(pBk->wave[0]))  //wave数组的大小
			{
				gAdpcmCnt = 0;
				
				pBk->head[0] = 0xCF;  // pBtPack pBk = (pBtPack)cAdpcmBuf;  cAdpcmBuf大小128字节
				pBk->head[1] = 0xF0;
				pBk->seq = (unsigned char)btSeq++;
				pBk->dlen = sizeof(BtPack);  // ori 73
				pBk->fhr[0] = beat & 0xFF;  //取低8位数据
				pBk->fhr[1] = (beat >> 8) & 0xFF;  //取高8位数据
				//ricky del pBk->toco[0] = 1;
				//ricky del pBk->toco[1] = 192;

				//校验和
				p8 = (uint8_t *)cAdpcmBuf;  
				sum = 0;
				for(i = 0; i < sizeof(BtPack) - 1; i++)
					sum += p8[i];
				p8[i] = ~sum;
				
				//ricky 通过串口3，发送PCM编码的音频，串口1为MCU调试用
				//UART1OutBinCode((uint8_t *)cAdpcmBuf,sizeof(BtPack));  //ricky 调试用
				
				if(ptr_cmd_rsp.is_init_complete == 1)

				//测试用
				//if((ptr_cmd_rsp.is_init_complete == 1) && (HAL_GPIO_ReadPin(GPIO_BT_STATE, PIN_BT_STATE)))	
				{
					bt_send_datas((uint8_t *)cAdpcmBuf,sizeof(BtPack));  //输出到蓝牙模块
				//	my_printf("[%d]\r\n", pBk->seq);
					
				}
				// >>> 解码
				#if 0
				signed short decode_array[128];
				for(int itor = 0; itor < sizeof(pBk->wave); itor++)
				{
					decode_array[itor*2] = ADPCM_Decode(pBk->wave[itor] & 0x0f);
					
					printf_UART1("%d ", decode_array[itor*2]);
					
					decode_array[itor*2 + 1] = ADPCM_Decode((pBk->wave[itor] >> 4) & 0x0f);
			
					printf_UART1("%d ", decode_array[itor*2 + 1]);

				}
				
				printf_UART1("\r\n-----------------------------------\r\n");
				#endif
				// <<< 解码
				
			}
		}
		
		#ifdef OUTPUT_FINAL_DATAS  //输出平滑后的波形数据
		#else
		else	
		{
			gAdpcmData = adcFtBuf[adcFtTail];

			//立即压缩
			//gAdpcmData = ADPCM_Encode(gAdpcmData);
		}
		#endif
		gAdpcmNibble++;
		adcFtTail++;  //数组索引 加1；每次执行 FetalHeartRate_Sound 处理8个16位的数据
		adcFtTail &= ADCFTLEN_MSK;
	}

}

/*
#define bt_uartRx_head			aRx3_head
#define bt_uartRx_tail			aRx3_tail
#define bt_uartRx_buf			aRx3Buf

**/
void read_uart_upgrade_cmd(void)
{

	int cnt_rb;

	if(ptr_cmd_rsp.is_init_complete == 0)
	{
		return;
	}


	if (bt_uartRx_head >= bt_uartRx_tail)
		cnt_rb = bt_uartRx_head - bt_uartRx_tail;
	else 
		cnt_rb = UART_BUF_LEN + bt_uartRx_head - bt_uartRx_tail;
	
	if(cnt_rb < 3)
		return;
	//update_led_state(HEART_BEAT_TOO_HIGH);

	for(int i = 0; i < cnt_rb; i++)
	{
		if(bt_uartRx_buf[bt_uartRx_tail] == 0xaa)
		{
			//bt_uartRx_buf[bt_uartRx_tail] = 0;
			
			bt_uartRx_tail++;		
			bt_uartRx_tail &= UART_BUF_MSK;

			if(bt_uartRx_buf[bt_uartRx_tail] == 0x55)
			{
				//bt_uartRx_buf[bt_uartRx_tail] = 0;

				bt_uartRx_tail++;		
				bt_uartRx_tail &= UART_BUF_MSK;
				i++;
			
				if(bt_uartRx_buf[bt_uartRx_tail] == 0x11)
				{
					bt_uartRx_tail++;		
					bt_uartRx_tail &= UART_BUF_MSK;
					i++;

					my_printf("usart iap ...........\r\n");
					start_iap();
				}
				else
				{
					//bt_uartRx_buf[bt_uartRx_tail] = 0;
					bt_uartRx_tail++;
					bt_uartRx_tail &= UART_BUF_MSK;
					i++;
				}
			}
		}
		else
		{
			bt_uartRx_tail++;		
			bt_uartRx_tail &= UART_BUF_MSK;
		}
	}
}

#ifdef DEBUG_BLUETOOTH
int read_uart_bluetooth_debug(void)
{

	int32_t tmp;
	int cnt_rb;
	int ret = -1;
	unsigned char tmp8;

	if (aRx3_head >= aRx3_tail)
		cnt_rb = aRx3_head - aRx3_tail;
	else 
		cnt_rb = UART_BUF_LEN + aRx3_head - aRx3_tail;

	if(aRx3Buf[aRx3_head] == '\n')
	{
		aRx3_head = 0;
		return cnt_rb;
	}
	else
		return -1;

	

	if(cnt_rb > 3)
	{
		tmp8 = aRx3Buf[aRx3_tail];
		aRx3_tail ++;
		return tmp8;
			
		
	}
		
	return -1;
	
	if(cnt_rb < 3)
		return -1;
	//update_led_state(HEART_BEAT_TOO_HIGH);
	tmp = aRx3_tail;
	if(aRx3Buf[aRx3_tail++] == 'g')
	{
		//update_led_state(HEART_BEAT_TOO_LOW);
		aRx3Buf[tmp] = 0;
		tmp = aRx3_tail;

		if(aRx3Buf[aRx3_tail++] == 'a')
		{
			aRx3Buf[tmp] = 0;
			
			switch(aRx3Buf[aRx3_tail])
			{
				case '0':
					ret = 0;
					break;
				case '1':
					ret = 1;
					break;	
				case '2':
					ret = 2;
					break;
				default:
					break;
			}
			aRx3Buf[aRx3_tail] = 0;
			
			aRx3_tail++;
		}

	}
	return ret;
}

#endif

void AlgorithmInit(void)
{
	sld_sum1 = 0;
	sld_pt1 = 0;
	memset(sld_buf1, 0, sizeof(sld_buf1));

	memset(sld_buf2, 0, sizeof(sld_buf2));

	beat_head = 0; beat_tail = 0; beat_tail2 = 2;
	slpHead = 0; slpTail = 0;
	adcTime = 2;

	noBeatTime = 0;
	AdcAvgCnt = 0;
	AdcAvgSum = 0;
	
	gAdpcmNibble = 0;
	gAdpcmCnt = 0;
	gAdpcmData = 0;
	btSeq = 0;

	diff1 = 0; diff2 = 0;
	goodwave = 0;	// ????????;


#ifdef RICKY_ALG
	sld_twice_sum = 0;  //平滑累加值
	sld_twice_pt = 0;  //sld_buf_twice的指针
	memset(sld_buf_twice, 0, sizeof(sld_buf_twice));
	
#endif

	is_peak_available = 0;
	

	//ricky add
	g_first_beat = 1;
	reset_working_flag();
	ori_samp = 0;
	ori_samp_max = 0;

	//必须加上，否则缓冲区可能填满，算法里面会死循环
	HAL_TIM_Base_Stop_IT(&htim3);  //定时器3 关闭中断
	HAL_ADC_Stop(&hadc1);	// run fetal beat ADC sample
	
	adcFtHead = 0;
	adcFtTail = 0;
	HAL_ADC_Start(&hadc1);	// run fetal beat ADC sample
	HAL_TIM_Base_Start_IT(&htim3);  //定时器3 开启中断
	//ricky add
	mute_toggle(0);
	//mute_toggle(1);  // 1为静音
}






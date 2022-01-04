
unsigned short cnt_1s = 0;
unsigned char is_1s = 0;
void HAL_SYSTICK_Callback(void)
{
	cnt_1s++;
	if(cnt_1s == 1000)
	{
		cnt_1s = 0;
		is_1s = 1;
	}
}


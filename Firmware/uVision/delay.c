#include "inc/delay.h"

extern void systick_ms_callback(uint32_t ms);
volatile static uint32_t ms;

void systick_init(void)
{
	SysTick->LOAD = SYST_CLOCK/1000 - 1; // 32 MHz / 32000 = 1 kHz -> 1 ms
	SysTick->VAL = 1; // set current value to anything
	SysTick->CTRL = (1 << 2) | (1 << 1) | (1 << 0); // CLKSOURCE = Processor clock (AHB), assert exception request, SysTick enable
	
	ms = 0;
}

void mft_init(void)
{
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_MTFX1, ENABLE);
	
	MFT_InitType mft_settings;
	MFT_StructInit(&mft_settings);
	mft_settings.MFT_Mode = MFT_MODE_4;
	mft_settings.MFT_Clock1 = MFT_PRESCALED_CLK;
	mft_settings.MFT_Prescaler = 31;
	
	MFT_Init(MFT1, &mft_settings);
}

void delay_us(uint16_t delay)
{
	MFT_Cmd(MFT1, ENABLE);
	MFT_SetCounter1(MFT1, delay);
	while (MFT_GetCounter1(MFT1) > 0);
	MFT_Cmd(MFT1, DISABLE);
}

uint32_t get_ms(void)
{
	return ms;
}

void delay_ms(uint32_t delay)
{
	uint32_t end = ms + delay;
	while (ms < end);
}


void SysTick_Handler(void)
{
  ms++;
  systick_ms_callback(ms);
}


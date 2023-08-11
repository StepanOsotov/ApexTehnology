#include "init_periph.h"

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


void initDebugLed(void)
{
	GPIO_InitTypeDef GPIO_LEDS;
	
  /* GPIOB Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* Configure PB6, PB7 in output pushpull mode */
	
  GPIO_LEDS.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_LEDS.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_LEDS.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_LEDS.GPIO_OType = GPIO_OType_PP;
	GPIO_LEDS.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_LEDS);
	
	GPIOB->ODR |= 1 << 6;
	GPIOB->ODR |= (1 << 7);
	
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
/*init SysTick_Handler.*/
#define	TIME_MS_100			SystemCoreClock/10-1
#define	TIME_MS_10			SystemCoreClock/100-1
#define	TIME_MS_1				SystemCoreClock/1000-1
#define	TIME_MKS_100		SystemCoreClock/10000-1
#define	TIME_MKS_10			SystemCoreClock/100000-1

void initSysTick(void)
{
	SysTick->CTRL = 0;

	// interrupt triggered in 1ms
	//SysTick_Config(SystemCoreClock/1000-1);    // Configure the SYSTICK

	SysTick->LOAD = TIME_MS_1;
	
	SysTick->CTRL |= SysTick_CTRL_COUNTFLAG;
	/*
	 *  CLKSOURCE: Clock source selection
		Selects the clock source.
		0: AHB/8
		1: Processor clock (AHB)
		AHB = 24 MHz
	 */
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE;
	// enable interrupt
	// Counting down to zero to asserts the SysTick exception reques
	SysTick->CTRL |= SysTick_CTRL_TICKINT;
	// ENABLE: Counter enable
	SysTick->CTRL |= SysTick_CTRL_ENABLE;
	//NVIC_SetPriority(SysTick_IRQn,1);

}
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void initClock(void)
{
	
	SystemCoreClock = ( unsigned long ) 2097000;
	
	SystemInit();
	
	//initSysTick();
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

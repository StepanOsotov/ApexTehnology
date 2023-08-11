/**
  ******************************************************************************
  * @file    GPIO/IOToggle/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/

#include "main.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define USE_FULL_ASSERT

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

const char *usart3Str = "123456789 Tx Check\n\r";
const char *start2RTOS = "vTaskStartScheduler\n\r";
const char *usart2DMA = "Enable DMA, rx (default = 5 byte)\n\r";
const char *usart2IRQ = "Enable IRQ, rx end Message = 0x0A 0x0D\n\r";
const char *usart2POOL = "Enable Polling, rx end Message = 0x0A 0x0D\n\r";

const char *usart2ErrorConfig = "Error Config DMA and IRQ (tx, Rx)\n\r";

/* Private function prototypes -----------------------------------------------*/

void USART_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	/*
	GPIO_Debug
		LD3		-	PB7
		LD4		-	PB6
	
	USART2
		PA2	-	TxD
		PA3	-	RxD
	
	USART3
		PC10	-	TxD
		PC11	-	RxD
	*/
	//----------------------------------------
	
	initClock();
	
	usart3to2_usr.isIrqOnRx = 1;
	usart3to2_usr.isIrqOnTx = 1;
	
	USART3_Settings();
	USART2_Settings();
	
	USART_Config();
	
	initDebugLed();
	
	//----------------------------------------
	
	/*
	usart3_usr.length = strlen(usart3Str);
	static paramUsart_t paramUsart3;
	paramUsart3.txString = usart3Str;
	paramUsart3.length = usart3_usr.length;
	
	xTaskCreate(vTaskUsart3Transmit, (const char *)"TxUsart3",
							configMINIMAL_STACK_SIZE, (void*)&paramUsart3,
							tskIDLE_PRIORITY + 1, NULL);
	*/
	vSemaphoreCreateBinary(xBinarySemaphore);
	
	xTaskCreate(vTaskLedToggle, (const char *)"ToggleLeds",
							configMINIMAL_STACK_SIZE, NULL,
							tskIDLE_PRIORITY + 1, NULL);
	
	xTaskCreate(vTaskUsart3Tranceiver, (const char *)"TxRxUsart3",
							configMINIMAL_STACK_SIZE, NULL,
							tskIDLE_PRIORITY + 1, NULL);
	
	xTaskCreate(vTaskUsart2Tranceiver, (const char *)"TxRxUsart2",
							configMINIMAL_STACK_SIZE, NULL,
							tskIDLE_PRIORITY + 1, NULL);
	
  vTaskStartScheduler();  //start FreeRTOS
	
	//----------------------------------------
	
  while(1)
	{
		__asm("nop			\n\r");
	}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

void USART_Config(void)
{
	
	if(!usart3to2_usr.isErrConf)
	{
		usart3to2_usr.length = strlen(start2RTOS);
		
		USART2_SendText((const uint8_t *)start2RTOS, usart3to2_usr.length, TX_WAIT_END);		
		
		if(usart3to2_usr.isDmaOnRx)
		{
			usart3to2_usr.length = strlen(usart2DMA);
		
			USART2_SendText((const uint8_t *)usart2DMA, usart3to2_usr.length, TX_WAIT_END);
		}
		else if(usart3to2_usr.isIrqOnRx)
		{
			usart3to2_usr.length = strlen(usart2IRQ);
		
			USART2_SendText((const uint8_t *)usart2IRQ, usart3to2_usr.length, TX_WAIT_END);
		}
		else
		{
			usart3to2_usr.length = strlen(usart2POOL);
		
			USART2_SendText((const uint8_t *)usart2POOL, usart3to2_usr.length, TX_WAIT_END);
		}
	}
	else
	{
		usart3to2_usr.length = strlen(usart2ErrorConfig);
		
		USART2_SendText((const uint8_t *)usart2ErrorConfig, usart3to2_usr.length, TX_WAIT_END);
	}
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

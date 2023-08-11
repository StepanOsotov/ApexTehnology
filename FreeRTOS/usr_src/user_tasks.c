#include "user_tasks.h"
#include "semphr.h"

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

usart_usr_t usart3_usr;
usart_usr_t usart2_usr;

SemaphoreHandle_t xBinSemaph3;
SemaphoreHandle_t xBinSemaph2;

void clearArray(uint8_t *arr, uint16_t len);

#define WAIT_SEMAPHORE	100

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void vTaskLedToggle(void *pvParameters)
{
	while(pdTRUE)
	{
		vTaskDelay(100);
		
		GPIOB->ODR ^= (1 << 6);
		
		//============================================================
	}
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void vTaskUsart2Tranceiver(void *pvParameters)
{
	
	usart2_usr.status = NO_WORK;
	usart2_usr.length = 0;
	usart2_usr.rxCheck = 0;
	usart2_usr.completeMessage = 0;
	
	usart2_usr.cntRx = 0;
	usart2_usr.isRxOverflow = 0;
	
	while(pdTRUE)
	{
		
		//==========================-HANDLER-SEMAPHORE-FROM-3=============================
		
		xSemaphoreTake(xBinSemaph3, WAIT_SEMAPHORE);
		if(usart3_usr.isRxOverflow)
		{
			usart3_usr.length = snprintf((char *)usart3_usr.bufTx, BUFFER_USART_MAX,
								"overflow %d\n\r", usart3_usr.isRxOverflow);
			
			usart3_usr.isRxOverflow = 0;
			
			USART2_SendText((const uint8_t *)usart3_usr.bufTx, usart3_usr.length, TX_NO_WAIT);
			
			clearArray(usart3_usr.bufTx, BUFFER_USART_MAX);
			clearArray(usart3_usr.bufRx, BUFFER_USART_MAX);
			
		}
		
		if(usart3_usr.completeMessage)
		{
			clearArray(usart3_usr.bufTx, BUFFER_USART_MAX);
			
			usart3_usr.length = snprintf((char *)usart3_usr.bufTx, BUFFER_USART_MAX,
										"rx msg %s", usart3_usr.bufRx);																							//snprintf No Thread Safety
			USART2_SendText((const uint8_t *)usart3_usr.bufTx, usart3_usr.length, TX_NO_WAIT);

			usart3_usr.rxCheck = usart3_usr.cntRx = 0;
			
			clearArray(usart3_usr.bufRx, BUFFER_USART_MAX);
			
			usart3_usr.completeMessage = 0;
			
			GPIOB->ODR ^= (1 << 7);
		}
		
		//==========================-RX-In-IRQ-FOR-USART-2=============================
		if(usart2_usr.isIrqOnRx)
		{
			if(usart2_usr.isRxOverflow)
			{
				xSemaphoreGive(xBinSemaph2);
			}
			else
			{
				if(usart2_usr.rxCheck != usart2_usr.cntRx)
				{
					for(uint16_t i = 0; i < usart2_usr.cntRx ; i++)
					{
						if((usart2_usr.bufRx[i] == 0x0A) && (usart2_usr.bufRx[i+1] == 0x0D))
						{
							usart2_usr.completeMessage = 1;
							
							xSemaphoreGive(xBinSemaph2);
							
							GPIOB->ODR ^= (1 << 7);
								
							break;
						}
					}
					usart2_usr.rxCheck = usart2_usr.cntRx;
				}
			}
		}
		
		vTaskDelay(10);
		
		/*
		//==========================-RX-In-DMA-=============================
		if(usart3_usr.isDmaOnRx)
		{
			if(usart3_usr.dma_wait_rx)
			{
				usart3_usr.dma_wait_rx = 0;
				
				usart3_usr.length = snprintf((char *)usart3_usr.bufTx, BUFFER_USART_MAX,
												"rx msg : %s \n\r", usart3_usr.bufRx);																							//snprintf No Thread Safety
				USART2_SendText((const uint8_t *)usart3_usr.bufTx, usart3_usr.length, TX_WAIT_END);
				
				GPIOB->ODR ^= (1 << 7);
				
				clearArray(usart3_usr.bufTx, BUFFER_USART_MAX);
				clearArray(usart3_usr.bufRx, BUFFER_USART_MAX);
				
				rx_USART2_with_DMA((uint8_t *)usart3_usr.bufRx, DMA_USART_RX_COUNT);
				
			}
			vTaskDelay(10);
		}
		
		//==========================-RX-In-Thread-==========================
		else //if((!usart3_usr.isDmaOnRx) && (!usart3_usr.isIrqOnRx))
		{
			usart3_usr.status = USART2_ReceiveTextWith0x0A0x0D();
			
			if(SUCCESS_MESSAGE == usart3_usr.status)
			{
				GPIOB->ODR ^= (1 << 7);
				
				clearArray(usart3_usr.bufTx, BUFFER_USART_MAX);
				clearArray(usart3_usr.bufRx, BUFFER_USART_MAX);
				
				usart3_usr.cntRx = 0;
			}
			vTaskDelay(10);
		}
		*/
	}
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void vTaskUsart3Tranceiver(void *pvParameters)
{
	usart3_usr.status = NO_WORK;
	usart3_usr.length = 0;
	usart3_usr.rxCheck = 0;
	usart3_usr.completeMessage = 0;
	
	usart3_usr.cntRx = 0;
	usart3_usr.isRxOverflow = 0;
	
	while(pdTRUE)
	{
		//==========================-RX-In-IRQ-FOR-USART-3=============================
		if(usart3_usr.isIrqOnRx)
		{
			if(usart3_usr.isRxOverflow)
			{
				xSemaphoreGive(xBinSemaph3);
			}
			else
			{
				if(usart3_usr.rxCheck != usart3_usr.cntRx)
				{
					for(uint16_t i = 0; i < usart3_usr.cntRx ; i++)
					{
						if((usart3_usr.bufRx[i] == 0x0A) && (usart3_usr.bufRx[i+1] == 0x0D))
						{
							usart3_usr.completeMessage = 1;
							
							xSemaphoreGive(xBinSemaph3);
							
							GPIOB->ODR ^= (1 << 7);
								
							break;
						}
					}
					usart3_usr.rxCheck = usart3_usr.cntRx;
				}
			}
		}
		
		//==========================-HANDLER-SEMAPHORE-FROM-3=============================
		
		xSemaphoreTake(xBinSemaph2, WAIT_SEMAPHORE);
		
		if(usart2_usr.isRxOverflow)
		{
			usart2_usr.length = snprintf((char *)usart2_usr.bufTx, BUFFER_USART_MAX,
								"overflow %d\n\r", usart2_usr.isRxOverflow);
			
			usart2_usr.isRxOverflow = 0;
			
			USART3_SendText((const uint8_t *)usart2_usr.bufTx, usart2_usr.length, TX_NO_WAIT);
			
			clearArray(usart2_usr.bufTx, BUFFER_USART_MAX);
			clearArray(usart2_usr.bufRx, BUFFER_USART_MAX);
			
		}
		
		if(usart2_usr.completeMessage)
		{
			clearArray(usart2_usr.bufTx, BUFFER_USART_MAX);
			
			usart2_usr.length = snprintf((char *)usart2_usr.bufTx, BUFFER_USART_MAX,
										"rx msg %s", usart2_usr.bufRx);																							//snprintf No Thread Safety
			USART3_SendText((const uint8_t *)usart2_usr.bufTx, usart2_usr.length, TX_NO_WAIT);

			usart2_usr.rxCheck = usart2_usr.cntRx = 0;
			
			clearArray(usart2_usr.bufRx, BUFFER_USART_MAX);
			
			usart2_usr.completeMessage = 0;
			
			GPIOB->ODR ^= (1 << 7);
		}
		
		vTaskDelay(10);
		
		/*
		//==========================-RX-In-DMA-=============================
		if(usart3_usr.isDmaOnRx)
		{
			if(usart3_usr.dma_wait_rx)
			{
				usart3_usr.dma_wait_rx = 0;
				
				usart3_usr.length = snprintf((char *)usart3_usr.bufTx, BUFFER_USART_MAX,
											"rx msg %s \n\r", usart3_usr.bufRx);																							//snprintf No Thread Safety
				USART3_SendText((const uint8_t *)usart3_usr.bufTx, usart3_usr.length, TX_WAIT_END);
				
				GPIOB->ODR ^= (1 << 7);
				
				clearArray(usart3_usr.bufTx, BUFFER_USART_MAX);
				clearArray(usart3_usr.bufRx, BUFFER_USART_MAX);
				
				rx_USART3_with_DMA((uint8_t *)usart3_usr.bufRx, DMA_USART_RX_COUNT);
				
			}
			vTaskDelay(10);
		}
		
		//==========================-RX-In-Thread-==========================
		else //if((!usart3_usr.isDmaOnRx) && (!usart3_usr.isIrqOnRx))
		{
			usart3_usr.status = USART3_ReceiveTextWith0x0A0x0D();
			
			if(SUCCESS_MESSAGE == usart3_usr.status)
			{
				GPIOB->ODR ^= (1 << 7);
				
				clearArray(usart3_usr.bufTx, BUFFER_USART_MAX);
				clearArray(usart3_usr.bufRx, BUFFER_USART_MAX);
				
				usart3_usr.cntRx = 0;
			}
			vTaskDelay(10);
		}
		*/
	}
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
/*
*/
//-------------------------------------------------------------------------------------------------

void clearArray(uint8_t *arr, uint16_t len)
{
	//	clear message
	while(len--)
	{
		arr[len] = 0x00;
	}
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
/*

float
S -  1 bit (signed/unsigned)
W -  8 bit (Window)
O - 23 bit (Offset)
 --- ---------------- --------------------------------
| S |     WINDOW     |           OFFSET               |
 --- ---------------- --------------------------------

valFloat = 3.14

valBits.S = 0;
valBits.W = 128;
valBits.O = 4781507

*/

#define	LENGTH_POW_32	10

typedef struct
{
	float valFloat;
	int32_t valUint;
	//2^32 = 4294967296
	uint8_t valArr[LENGTH_POW_32];
	uint8_t iPosit;
	uint32_t precission;
	struct
	{
		uint32_t O : 23;
		uint32_t W : 8;
		uint32_t S : 1;
		
	} valBits;
	
} floatBits_t;

floatBits_t floatBits;

void floatToAscii(uint8_t *arr, uint16_t *position, float val)
{
	floatBits.valUint = val;
	floatBits.valFloat = val;
	floatBits.iPosit = 0;
	
	uint8_t currPosit = 0;
	
	//----------------------------------------------------------------
	//---------------the-integer-part-of-number-----------------------
	//----------------------------------------------------------------
	
	for(floatBits.iPosit = 0; floatBits.iPosit < LENGTH_POW_32; floatBits.iPosit++)
	{
		if(!floatBits.valUint)
		{
			break;
		}
		else
		{
			floatBits.valArr[floatBits.iPosit] = floatBits.valUint % 10;
			floatBits.valUint /= 10;
		}
	}
	while(floatBits.iPosit)
	{
		*(arr + currPosit) = floatBits.valArr[floatBits.iPosit-1] + 0x30;
		floatBits.valArr[floatBits.iPosit-1] = 0x00;
		currPosit++;
		(*position)++;
		floatBits.iPosit--;
	}
	*(arr + currPosit) = '.';
	currPosit++;
	
	//----------------------------------------------------------------
	//---------------fractional-part-of-a-number----------------------
	//----------------------------------------------------------------
	
	floatBits.valUint = val;
	floatBits.valFloat = val - floatBits.valUint;
	floatBits.valFloat *= floatBits.precission;
	floatBits.valUint = floatBits.valFloat;
	
	floatBits.iPosit = 0;
	
	for(floatBits.iPosit = 0; floatBits.iPosit < LENGTH_POW_32; floatBits.iPosit++)
	{
		if(!floatBits.valUint)
		{
			break;
		}
		else
		{
			floatBits.valArr[floatBits.iPosit] = floatBits.valUint % 10;
			floatBits.valUint /= 10;
		}
	}
	while(floatBits.iPosit)
	{
		*(arr + currPosit) = floatBits.valArr[floatBits.iPosit-1] + 0x30;
		floatBits.valArr[floatBits.iPosit-1] = 0x00;
		currPosit++;
		(*position)++;
		floatBits.iPosit--;
	}
	(*position)++;
	*(arr + currPosit) = ' ';
	
	//----------------------------------------------------------------
	//---------------end-convert-a-number-----------------------------
	//----------------------------------------------------------------
	
}

//-------------------------------------------------------------------------------------------------
/*

*/

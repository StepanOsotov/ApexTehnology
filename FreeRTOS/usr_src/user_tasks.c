#include "user_tasks.h"
#include "semphr.h"

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

usart_usr_t usart3to2_usr;

SemaphoreHandle_t xBinarySemaphore;

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

void vTaskUsart2Transmit(void *pvParameters)
{
	paramUsart_t *paramUsart2 = (paramUsart_t *)pvParameters;
	
	while(pdTRUE)
	{
		vTaskDelay(1000);
		
		USART2_SendText((const uint8_t *)paramUsart2->txString, paramUsart2->length, TX_NO_WAIT);
		
		//============================================================
	}
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void vTaskUsart2Tranceiver(void *pvParameters)
{
	
	while(pdTRUE)
	{
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		
		//==========================-RX-In-IRQ-=============================
		if(usart3to2_usr.isRxOverflow)
		{
			usart3to2_usr.length = snprintf((char *)usart3to2_usr.bufTx, BUFFER_USART_MAX,
								"overflow %d\n\r", usart3to2_usr.isRxOverflow);
			
			usart3to2_usr.isRxOverflow = 0;
			
			USART2_SendText((const uint8_t *)usart3to2_usr.bufTx, usart3to2_usr.length, TX_NO_WAIT);
			
			clearArray(usart3to2_usr.bufTx, BUFFER_USART_MAX);
			clearArray(usart3to2_usr.bufRx, BUFFER_USART_MAX);
			
		}
		
		if(usart3to2_usr.completeMessage)
		{
			clearArray(usart3to2_usr.bufTx, BUFFER_USART_MAX);
			
			usart3to2_usr.length = snprintf((char *)usart3to2_usr.bufTx, BUFFER_USART_MAX,
										"rx msg %s", usart3to2_usr.bufRx);																							//snprintf No Thread Safety
			USART2_SendText((const uint8_t *)usart3to2_usr.bufTx, usart3to2_usr.length, TX_NO_WAIT);

			usart3to2_usr.rxCheck = usart3to2_usr.cntRx = 0;
			
			clearArray(usart3to2_usr.bufRx, BUFFER_USART_MAX);
			
			usart3to2_usr.completeMessage = 0;
			
			GPIOB->ODR ^= (1 << 7);
		}
		
		vTaskDelay(10);
		
		/*
		//==========================-RX-In-DMA-=============================
		if(usart3to2_usr.isDmaOnRx)
		{
			if(usart3to2_usr.dma_wait_rx)
			{
				usart3to2_usr.dma_wait_rx = 0;
				
				usart3to2_usr.length = snprintf((char *)usart3to2_usr.bufTx, BUFFER_USART_MAX,
												"rx msg : %s \n\r", usart3to2_usr.bufRx);																							//snprintf No Thread Safety
				USART2_SendText((const uint8_t *)usart3to2_usr.bufTx, usart3to2_usr.length, TX_WAIT_END);
				
				GPIOB->ODR ^= (1 << 7);
				
				clearArray(usart3to2_usr.bufTx, BUFFER_USART_MAX);
				clearArray(usart3to2_usr.bufRx, BUFFER_USART_MAX);
				
				rx_USART2_with_DMA((uint8_t *)usart3to2_usr.bufRx, DMA_USART_RX_COUNT);
				
			}
			vTaskDelay(10);
		}
		
		//==========================-RX-In-Thread-==========================
		else //if((!usart3_usr.isDmaOnRx) && (!usart3_usr.isIrqOnRx))
		{
			usart3to2_usr.status = USART2_ReceiveTextWith0x0A0x0D();
			
			if(SUCCESS_MESSAGE == usart3to2_usr.status)
			{
				GPIOB->ODR ^= (1 << 7);
				
				clearArray(usart3to2_usr.bufTx, BUFFER_USART_MAX);
				clearArray(usart3to2_usr.bufRx, BUFFER_USART_MAX);
				
				usart3to2_usr.cntRx = 0;
			}
			vTaskDelay(10);
		}
		*/
	}
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void vTaskUsart3Transmit(void *pvParameters)
{
	paramUsart_t *paramUsart = (paramUsart_t *)pvParameters;
	
	while(pdTRUE)
	{
		vTaskDelay(1000);
		
		USART3_SendText((const uint8_t *)paramUsart->txString, paramUsart->length, TX_NO_WAIT);
		
		//============================================================
	}
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void vTaskUsart3Tranceiver(void *pvParameters)
{
	usart3to2_usr.status = NO_WORK;
	usart3to2_usr.length = 0;
	usart3to2_usr.rxCheck = 0;
	usart3to2_usr.completeMessage = 0;
	
	usart3to2_usr.cntRx = 0;
	usart3to2_usr.isRxOverflow = 0;
	
	while(pdTRUE)
	{
		//==========================-RX-In-IRQ-=============================
		if(usart3to2_usr.isIrqOnRx)
		{
			if(usart3to2_usr.isRxOverflow)
			{
				xSemaphoreGive(xBinarySemaphore);
			}
			else
			{
				if(usart3to2_usr.rxCheck != usart3to2_usr.cntRx)
				{
					for(uint16_t i = 0; i < usart3to2_usr.cntRx ; i++)
					{
						if((usart3to2_usr.bufRx[i] == 0x0A) && (usart3to2_usr.bufRx[i+1] == 0x0D))
						{
							usart3to2_usr.completeMessage = 1;
							
							xSemaphoreGive(xBinarySemaphore);
							
							GPIOB->ODR ^= (1 << 7);
								
							break;
						}
					}
					usart3to2_usr.rxCheck = usart3to2_usr.cntRx;
				}
			}
		}
		
		vTaskDelay(10);
		
		/*
		//==========================-RX-In-DMA-=============================
		if(usart3to2_usr.isDmaOnRx)
		{
			if(usart3to2_usr.dma_wait_rx)
			{
				usart3to2_usr.dma_wait_rx = 0;
				
				usart3to2_usr.length = snprintf((char *)usart3to2_usr.bufTx, BUFFER_USART_MAX,
											"rx msg %s \n\r", usart3to2_usr.bufRx);																							//snprintf No Thread Safety
				USART3_SendText((const uint8_t *)usart3to2_usr.bufTx, usart3to2_usr.length, TX_WAIT_END);
				
				GPIOB->ODR ^= (1 << 7);
				
				clearArray(usart3to2_usr.bufTx, BUFFER_USART_MAX);
				clearArray(usart3to2_usr.bufRx, BUFFER_USART_MAX);
				
				rx_USART3_with_DMA((uint8_t *)usart3to2_usr.bufRx, DMA_USART_RX_COUNT);
				
			}
			vTaskDelay(10);
		}
		
		//==========================-RX-In-Thread-==========================
		else //if((!usart3to2_usr.isDmaOnRx) && (!usart3to2_usr.isIrqOnRx))
		{
			usart3to2_usr.status = USART3_ReceiveTextWith0x0A0x0D();
			
			if(SUCCESS_MESSAGE == usart3to2_usr.status)
			{
				GPIOB->ODR ^= (1 << 7);
				
				clearArray(usart3to2_usr.bufTx, BUFFER_USART_MAX);
				clearArray(usart3to2_usr.bufRx, BUFFER_USART_MAX);
				
				usart3to2_usr.cntRx = 0;
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

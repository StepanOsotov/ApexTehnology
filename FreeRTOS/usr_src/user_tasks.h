#ifndef USER_TASK_H
#define USER_TASK_H

#include "main.h"
#include "usart3_usr.h"

//--------------------------------------------------------------------------------
#define	BUFFER_USART_MAX	80

#define DMA_USART_RX_COUNT	5

typedef struct
{
	const char *txString;
	uint16_t length;
	
} paramUsart_t;

enum status_serial_t
{
	NO_WORK = -2, NO_RESPONSE, NO_DATA, SUCCESS_BYTE, SUCCESS_MESSAGE, OVERFLOW
	
};

typedef struct
{
	uint8_t bufTx[BUFFER_USART_MAX];
	uint8_t bufRx[BUFFER_USART_MAX];
	volatile uint16_t cntTx;
	volatile uint16_t lengthTx;
	volatile uint16_t cntRx;
	volatile uint8_t isRxOverflow;
	
	enum status_serial_t status;
	
	uint16_t length;
	uint16_t rxCheck;
	uint8_t completeMessage;
	uint8_t isErrConf;
	
	uint8_t isDmaOnTx;
	uint8_t isDmaOnRx;
	uint8_t isIrqOnTx;
	uint8_t isIrqOnRx;
	
	volatile uint8_t dma_wait_tx;
	volatile uint8_t dma_wait_rx;
	
} usart_usr_t;

extern usart_usr_t usart3_usr;
extern usart_usr_t usart2_usr;

extern SemaphoreHandle_t xBinSemaph3to2;
extern SemaphoreHandle_t xBinSemaph2to3;

//--------------------------------------------------------------------------------

void vTaskLedToggle(void *pvParameters);

void vTaskUsart2Tranceiver(void *pvParameters);
void vTaskUsart3Tranceiver(void *pvParameters);

//--------------------------------------------------------------------------------

#endif //USER_TASK_H

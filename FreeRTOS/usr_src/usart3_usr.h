#ifndef __USART_3_DEBUG_H__
#define __USART_3_DEBUG_H__

#include "main.h"
#include "delay_us.h"
#include "user_tasks.h"

#define THRESHOLD_TIMEOUT_3  6000

#define TX_WAIT_END			1
#define TX_NO_WAIT			0

void USART3_Settings(void);
void tx_USART3_with_IRQ(const uint8_t *buffer, uint32_t len);
void USART3_SendByte(const uint8_t data);
void USART3_SendText(const uint8_t *data,uint16_t size, uint8_t isWait);
enum status_serial_t USART3_Rx_Byte(uint8_t *data);
enum status_serial_t USART3_ReceiveTextNoReentrant(uint8_t *data, uint8_t *rxLength, uint8_t size);
enum status_serial_t USART3_ReceiveTextWith0x0A0x0D(void);

void DMA1_Usart3_InitTX(void);
void DMA1_Usart3_InitRX(void);

void tx_USART3_with_DMA(const uint8_t *buffer, uint32_t len);
void rx_USART3_with_DMA(uint8_t *const buffer, uint32_t len);

#endif //__USART_3_DEBUG_H__

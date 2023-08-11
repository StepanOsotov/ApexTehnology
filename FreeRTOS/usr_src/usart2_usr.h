#ifndef __USART_2_H__
#define __USART_2_H__

#include "usart3_usr.h"

void USART2_Settings(void);
void tx_USART2_with_IRQ(const uint8_t *buffer, uint32_t len);
void USART2_SendByte(const uint8_t data);
void USART2_SendText(const uint8_t *data,uint16_t size, uint8_t isWait);
enum status_serial_t USART2_Rx_Byte(uint8_t *data);
enum status_serial_t USART2_ReceiveTextNoReentrant(uint8_t *data, uint8_t *rxLength, uint8_t size);
enum status_serial_t USART2_ReceiveTextWith0x0A0x0D(void);

void DMA1_Usart2_InitTX(void);
void DMA1_Usart2_InitRX(void);

void tx_USART2_with_DMA(const uint8_t *buffer, uint32_t len);
void rx_USART2_with_DMA(uint8_t *const buffer, uint32_t len);

#endif //__USART_2_H__

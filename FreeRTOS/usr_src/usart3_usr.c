#include "usart3_usr.h"

//-------------------------------------------------------------------------------------------------
//--------------------------------------------DATA-USART-3-----------------------------------------
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------USART-3-----------------------------------------
//-------------------------------------------------------------------------------------------------
/*
	USART3
		TxD	-	PC10
		RxD -	PC11
*/
void USART3_Settings(void)
{
	//	PORTC Clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	//	USART3 Clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	//	Bit 0 SYSCFGEN: System configuration controller clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	//
	GPIO_InitTypeDef PORT_UART;
	PORT_UART.GPIO_Mode = GPIO_Mode_AF;
	PORT_UART.GPIO_Pin = GPIO_Pin_10;	//UART3 TX
	PORT_UART.GPIO_Speed = GPIO_Speed_10MHz;
	PORT_UART.GPIO_OType = GPIO_OType_PP;
	PORT_UART.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC,&PORT_UART);

	PORT_UART.GPIO_Mode = GPIO_Mode_AF;
	PORT_UART.GPIO_Pin = GPIO_Pin_11;	//UART3 RX
	PORT_UART.GPIO_Speed = GPIO_Speed_10MHz;
	PORT_UART.GPIO_OType = GPIO_OType_PP;
	PORT_UART.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC,&PORT_UART);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11, GPIO_AF_USART3);
	
	//Procedure:
//-----------------------------------------------------------------------------
	//1.	Enable the USART by writing the UE bit in USART_CR1 register to 1.
	USART3->CR1 |= USART_CR1_UE;
//-----------------------------------------------------------------------------
	//2.	Program the M bit in USART_CR1 to define the word length.
	//USART_CR1_M = Clear(12 BIT) : 0 - 8 bit, 1 - 9 bit
	USART3->CR1 &= ~USART_CR1_M;
//-----------------------------------------------------------------------------
	/*3.		Program the number of stop bits in USART_CR2.
	 * USART_CR2_STOP
	(1)13:12(0)
	 * 00 - 1 Stop bit
	 * 01 - 0.5 Stop bit
	 * 10 - 2 Stop bit
	 * 11 - 1.5 Stop bit
	 */
	USART3->CR2 &= ~USART_CR2_STOP;
//-----------------------------------------------------------------------------
	//DMA off
	//4.	Select DMA enable (DMAT) in USART_CR3 if Multi buffer Communication is to take place. 
	//		Configure the DMA register as explained in multibuffer communication.
	//
	if((!usart3to2_usr.isDmaOnTx) && (!usart3to2_usr.isDmaOnRx))
	{
		USART3->CR3 &= ~USART_CR3_DMAT;
		USART3->CR3 &= ~USART_CR3_DMAR;
	}
	else
	{
		if(1 == usart3to2_usr.isDmaOnTx)
		{
			//Bit 7 DMAT: DMA enable transmitter
			USART3->CR3 |= USART_CR3_DMAT;
			
			DMA1_Usart3_InitTX();
			
		}
		if(1 == usart3to2_usr.isDmaOnRx)
		{
			
			//Bit 6 DMAR: DMA enable receiver
			USART3->CR3 |= USART_CR3_DMAR;
			
			DMA1_Usart3_InitRX();
		}		
	}

//-----------------------------------------------------------------------------
	//5.	Select the desired baud rate using the USART_BRR register.
	//----
	/* USART3->BRR =
	 *   2097000
	 * ---------- = 13.65234375 = 0xD
	 * 16 * 9600
	 *
	 * 0.65234375 * 16 = 10.4375 = 0xA
	 * USART3->BRR = 0xDA << 0;
	 */
	USART3->BRR = 0xDA << 0;
//-----------------------------------------------------------------------------
	//6.	Set the TE bit in USART_CR1 to send an idle frame as first transmission.
	USART3->CR1 |= USART_CR1_TE;
	//
	while(  (!(USART3->SR & USART_SR_TXE))  );
//-----------------------------------------------------------------------------
	//Set the RE bit USART_CR1. This enables the receiver which
	//begins searching for a start bit.
	USART3->CR1 |= USART_CR1_RE;
	//
	if(usart3to2_usr.isIrqOnRx)
	{
		//Bit 5 RXNEIE: RXNE interrupt enable
		USART3->CR1 |= USART_CR1_RXNEIE;
	}
	if(usart3to2_usr.isIrqOnTx)
	{
		//Bit 7 TXEIE: TXE interrupt enable
		USART3->CR1 |= USART_CR1_TXEIE;
		//Bit 6 TCIE: Transmission complete interrupt enable
		USART3->CR1 |= USART_CR1_TCIE;
	}
	if(usart3to2_usr.isIrqOnRx || usart3to2_usr.isIrqOnTx)
	{
		NVIC_EnableIRQ(USART3_IRQn);
	}
	
//-----------------------------------------------------------------------------
	
	if(usart3to2_usr.isDmaOnRx)
	{
		rx_USART3_with_DMA((uint8_t *)usart3to2_usr.bufRx, DMA_USART_RX_COUNT);
	}
}

//-----------------------------------------------------------------------------

void USART3_SendByte(const uint8_t data)
{
/*
Single byte communication
Clearing the TXE bit is always performed by a write to the data register.
*/
	USART3->DR = data;
	//
	while(  !(USART3->SR & USART_SR_TXE)  );
/*
The TXE bit is set by hardware and it indicates:
• The data has been moved from DR to the shift register
	and the data transmission has	started.
• The DR register is empty.
• The next data can be written in the USART_DR register
	without overwriting the previous data.
*/
}

//-----------------------------------------------------------------------------

void USART3_SendText(const uint8_t *data, uint16_t size, uint8_t isWait)
{
	if(usart3to2_usr.isDmaOnTx)
	{
		usart3to2_usr.dma_wait_tx = 0;
		tx_USART3_with_DMA(data, size);
		if(isWait)
		{
			while(!usart3to2_usr.dma_wait_tx);
		}
	}
	else if(usart3to2_usr.isIrqOnTx)
	{
		tx_USART3_with_IRQ(data, size);
		if(isWait)
		{
			while(USART3->CR1 & (USART_CR1_TXEIE | USART_CR1_TCIE));
		}
	}
	else
	{
		//
		while(size--)
		{
			USART3_SendByte(*(data++));
		}
	}
}

//-----------------------------------------------------------------------------

void tx_USART3_with_IRQ(const uint8_t *buffer, uint32_t len)
{
	if(len > BUFFER_USART_MAX)
		len = BUFFER_USART_MAX;
	// copies data
	for(uint32_t i = 0; i < len; ++i)
	{
		*(usart3to2_usr.bufTx + i) = *(buffer + i);
	}
	usart3to2_usr.cntTx = 1;
	usart3to2_usr.lengthTx = len;
	//tx one byte
	
	//Bit 7 TXE: Transmit data register empty
	//while(!(USART3->SR & USART_SR_TXE));
	
	USART3->DR = *(usart3to2_usr.bufTx + 0);
	
	if(usart3to2_usr.isIrqOnTx)
	{
		//Bit 7 TXEIE: TXE interrupt enable
		USART3->CR1 |= USART_CR1_TXEIE;
		//Bit 6 TCIE: Transmission complete interrupt enable
		USART3->CR1 |= USART_CR1_TCIE;
	}
	
}

//-----------------------------------------------------------------------------

enum status_serial_t USART3_Rx_Byte(uint8_t *data)
{
	//When a character is received
	//The RXNE bit is set.
	//This bit is set by hardware when the content of the DR shift register
  //has been transferred to the UART_DR register.
	if(USART3->SR & USART_SR_RXNE)
	{
    //0: Data is not received
		//1: Received data is ready to be read.
		*data = USART3->DR;
		return SUCCESS_BYTE;
	}
	//
	return NO_DATA;
}

//-----------------------------------------------------------------------------
// 1 byte => this byte length all wait rx array byte

enum status_serial_t USART3_ReceiveTextNoReentrant(uint8_t *data, uint8_t *rxLength, uint8_t size)
{
  static uint8_t countSerial = 0;
  static uint32_t timeNewSerial = 0;
  static uint32_t timeOldSerial = 0;
  
	uint8_t llRxData = 0;
  
	if(size == 0)
  {
    countSerial = 0;
		return NO_DATA;
  }
  
  if(countSerial > 0)
  {
    timeNewSerial = get_cyccnt();
    if((timeNewSerial - timeOldSerial) >= THRESHOLD_TIMEOUT_3)
    {
      timeNewSerial = timeOldSerial = get_cyccnt();
      countSerial = 0;
      return NO_RESPONSE;
    }
  }
	
	if(!USART3_Rx_Byte(&llRxData))
	{
		return NO_DATA;
	}
  timeNewSerial = timeOldSerial = get_cyccnt();
  
  *(data + countSerial) = llRxData;
	countSerial++;
  *rxLength = countSerial;
	if(countSerial >= size)
	{
		//very big data
		countSerial = 0;
		return OVERFLOW;
	}
	if(*rxLength == size)
	{
		return SUCCESS_MESSAGE;
	}
	return NO_WORK;
}

//-----------------------------------------------------------------------------
//end receive Message Text on 0x0D 0x0A
//Transmit in Terminal
//Hello Stepan Vitalievich$0A$0D

enum status_serial_t USART3_ReceiveTextWith0x0A0x0D(void)
{
	enum status_serial_t ret = NO_WORK;
	
	ret = USART3_Rx_Byte(&usart3to2_usr.bufRx[usart3to2_usr.cntRx]);
	//
	if(SUCCESS_BYTE == ret)
	{
		if(usart3to2_usr.cntRx < BUFFER_USART_MAX)
		{
			if((0x0D == usart3to2_usr.bufRx[usart3to2_usr.cntRx]) && (0x0A == usart3to2_usr.bufRx[usart3to2_usr.cntRx-1]))
			{
				ret = SUCCESS_MESSAGE;
			}
			else
			{
				usart3to2_usr.cntRx++;
			}
		}
		else
		{
			ret = OVERFLOW;
		}
	}
	
	return ret;
}

//-------------------------------------------------------------------------------------------------
//---------------------------------------------DMA-USART-3-----------------------------------------
//-------------------------------------------------------------------------------------------------

void DMA1_Usart3_InitTX(void)
{
	if(!(RCC->AHBENR & RCC_AHBENR_DMA1EN))
	{
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	}
	
	//1. Set the peripheral register address in the DMA_CPARx register. The data will be
	//moved from/ to this address to/ from the memory after the peripheral event.
	//DMA channel x peripheral address register (DMA_CPARx) (x = 1..7)
	//--------------------------------------------------------------------------
	//--------------------------PERIPHERAL-ADDRESS------------------------------
	//--------------------------------------------------------------------------
	DMA1_Channel2->CPAR = (uint32_t)&(USART3->DR);

	//2. Set the memory address in the DMA_CMARx register. The data will be written to or
	//read from this memory after the peripheral event.
	//DMA channel x memory address register (DMA_CMARx) (x = 1..7)
	//--------------------------------------------------------------------------
	//----------------------MEMORY-ADDRESS--------------------------------------
	//--------------------------------------------------------------------------
	DMA1_Channel2->CMAR = 0;

	//3. Configure the total number of data to be transferred in the DMA_CNDTRx register.
	//After each peripheral event, this value will be decremented.
	//DMA channel x number of data register (DMA_CNDTRx) (x = 1..7)
	//--------------------------------------------------------------------------
	//----------------------NUMBER-OF-DATA--------------------------------------
	//--------------------------------------------------------------------------
	DMA1_Channel2->CNDTR = 0;

	//
	DMA1_Channel2->CCR = 0;

	//4. Configure the channel priority using the PL[1:0] bits in the DMA_CCRx register
	//DMA channel x configuration register (DMA_CCRx) (x = 1..7)
	//00: Low
	//01: Medium
	//10: High
	//11: Very high
	DMA1_Channel2->CCR &= ~DMA_CCR2_PL;

	//5. Configure data transfer direction, circular mode, peripheral & memory incremented
	//mode, peripheral & memory data size, and interrupt after half and/or full transfer in the
	//DMA_CCRx register
	//Memory to memory mode
	//0: Memory to memory mode disabled
	//1: Memory to memory mode enabled
	DMA1_Channel2->CCR &= ~DMA_CCR2_MEM2MEM;
	//Memory size
	//00: 8-bits
	//01: 16-bits
	//10: 32-bits
	//11: Reserved
	DMA1_Channel2->CCR &= ~DMA_CCR2_MSIZE;
	//Peripheral size
	//00: 8-bits
	//01: 16-bits
	//10: 32-bits
	//11: Reserved
	DMA1_Channel2->CCR &= ~(DMA_CCR2_PSIZE_0 | DMA_CCR2_PSIZE_1);
	//Memory increment mode
	DMA1_Channel2->CCR |= DMA_CCR2_MINC;
	//Peripheral increment mode
	DMA1_Channel2->CCR &= ~DMA_CCR2_PINC;

	//Circular mode
	DMA1_Channel2->CCR &= ~DMA_CCR2_CIRC;

	//Data transfer direction
	//0: Read from peripheral
	//1: Read from memory
	DMA1_Channel2->CCR |= DMA_CCR2_DIR;
	//Half transfer interrupt enable
	//0: HT interrupt disabled
	//1: HT interrupt enabled
	DMA1_Channel2->CCR &= ~DMA_CCR2_HTIE;
	//TCIE: Transfer complete interrupt enable
	//0: TC interrupt disabled
	//1: TC interrupt enabled
	DMA1_Channel2->CCR |= DMA_CCR2_TCIE;
	//NVIC
	//NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	NVIC->ISER[0] |= 1<<DMA1_Channel2_IRQn;
	//6. Activate the channel by setting the ENABLE bit in the DMA_CCRx register.
	DMA1_Channel2->CCR &= ~DMA_CCR2_EN;

}

//-----------------------------------------------------------------------------

void tx_USART3_with_DMA(const uint8_t *buffer, uint32_t len)
{
	//Write the USART_DR register address in the DMA control
	//register to configure it as the destination of the transfer.
	//DMA1_Channel2->CPAR = (uint32_t)&(USART3->DR);

	//Write the memory address in the DMA control register
	//to configure it as the source of the transfer.
	DMA1_Channel2->CMAR = (uint32_t)buffer;

	//Configure the total number of bytes to be
	//transferred to the DMA control register.
	DMA1_Channel2->CNDTR = len;

	//Configure the channel priority in the DMA register
	//DMA1_Channel7->CCR &= ~DMA_CCR7_PL;

	//Configure DMA interrupt generation after half/ full
	//transfer as required by the application.
	//Half transfer interrupt enable
	//0: HT interrupt disabled
	//1: HT interrupt enabled
	//DMA1_Channel7->CCR &= ~DMA_CCR7_HTIE;
	//TCIE: Transfer complete interrupt enable
	//0: TC interrupt disabled
	//1: TC interrupt enabled
	//DMA1_Channel2->CCR |= DMA_CCR2_TCIE;

	//Clear the TC bit in the SR register by writing 0 to it.
	//USART3->SR &= ~USART_SR_TC;

	//Activate the channel in the DMA register
	DMA1_Channel2->CCR |= DMA_CCR2_EN;

}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void DMA1_Usart3_InitRX(void)
{
	if(!(RCC->AHBENR & RCC_AHBENR_DMA1EN))
	{
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	}

	//1. Set the peripheral register address in the DMA_CPARx register. The data will be
	//moved from/ to this address to/ from the memory after the peripheral event.
	//DMA channel x peripheral address register (DMA_CPARx) (x = 1..7)
	//--------------------------------------------------------------------------
	//--------------------------PERIPHERAL-ADDRESS------------------------------
	//--------------------------------------------------------------------------
	DMA1_Channel3->CPAR = (uint32_t)&(USART3->DR);

	//2. Set the memory address in the DMA_CMARx register. The data will be written to or
	//read from this memory after the peripheral event.
	//DMA channel x memory address register (DMA_CMARx) (x = 1..7)
	//--------------------------------------------------------------------------
	//----------------------MEMORY-ADDRESS--------------------------------------
	//--------------------------------------------------------------------------
	DMA1_Channel3->CMAR = 0;

	//3. Configure the total number of data to be transferred in the DMA_CNDTRx register.
	//After each peripheral event, this value will be decremented.
	//DMA channel x number of data register (DMA_CNDTRx) (x = 1..7)
	//--------------------------------------------------------------------------
	//----------------------NUMBER-OF-DATA--------------------------------------
	//--------------------------------------------------------------------------
	DMA1_Channel3->CNDTR = 0;

	DMA1_Channel3->CCR = 0;
	//DMA1->IFCR |= DMA_IFCR_CTCIF3;

	//4. Configure the channel priority using the PL[1:0] bits in the DMA_CCRx register
	//DMA channel x configuration register (DMA_CCRx) (x = 1..7)
	//00: Low
	//01: Medium
	//10: High
	//11: Very high
	DMA1_Channel3->CCR &= ~DMA_CCR3_PL;

	//5. Configure data transfer direction, circular mode, peripheral & memory incremented
	//mode, peripheral & memory data size, and interrupt after half and/or full transfer in the
	//DMA_CCRx register
	//Memory to memory mode
	//0: Memory to memory mode disabled
	//1: Memory to memory mode enabled
	DMA1_Channel3->CCR &= ~DMA_CCR3_MEM2MEM;
	//Memory size
	//00: 8-bits
	//01: 16-bits
	//10: 32-bits
	//11: Reserved
	DMA1_Channel3->CCR &= ~DMA_CCR3_MSIZE;
	//Peripheral size
	//00: 8-bits
	//01: 16-bits
	//10: 32-bits
	//11: Reserved
	DMA1_Channel3->CCR &= ~(DMA_CCR3_PSIZE_0 | DMA_CCR3_PSIZE_1);
	//Memory increment mode
	DMA1_Channel3->CCR |= DMA_CCR3_MINC;
	//Peripheral increment mode
	DMA1_Channel3->CCR &= ~DMA_CCR3_PINC;

	//Circular mode
	DMA1_Channel3->CCR &= ~DMA_CCR3_CIRC;

	//Data transfer direction
	//0: Read from peripheral
	//1: Read from memory
	DMA1_Channel3->CCR &= ~DMA_CCR3_DIR;
	//Half transfer interrupt enable
	//0: HT interrupt disabled
	//1: HT interrupt enabled
	DMA1_Channel3->CCR &= ~DMA_CCR3_HTIE;
	//TCIE: Transfer complete interrupt enable
	//0: TC interrupt disabled
	//1: TC interrupt enabled
	DMA1_Channel3->CCR |= DMA_CCR3_TCIE;
	//NVIC
	//NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	NVIC->ISER[0] |= 1<<DMA1_Channel3_IRQn;
	//6. Activate the channel by setting the ENABLE bit in the DMA_CCRx register.
	DMA1_Channel3->CCR &= ~DMA_CCR3_EN;

}

//-----------------------------------------------------------------------------

void rx_USART3_with_DMA(uint8_t *const buffer, uint32_t len)
{
	//Write the memory address in the DMA control register
	//to configure it as the source of the transfer.
	DMA1_Channel3->CMAR = (uint32_t)buffer;

	//Configure the total number of bytes to be
	//transferred to the DMA control register.
	DMA1_Channel3->CNDTR = len;

	//Configure DMA interrupt generation after half/ full
	//transfer as required by the application.
	//DMA1_Channel3->CCR |= DMA_CCR3_TCIE;
	//DMA1_Channel3->CCR |= DMA_CCR3_HTIE;

	//Clear the TC bit in the SR register by writing 0 to it.
	//USART3->SR &= ~USART_SR_TC;

	//Activate the channel in the DMA register
	DMA1_Channel3->CCR |= DMA_CCR3_EN;

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

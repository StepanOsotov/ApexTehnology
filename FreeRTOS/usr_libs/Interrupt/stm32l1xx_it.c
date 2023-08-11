/**
  ******************************************************************************
  * @file    Project/STM32L1xx_StdPeriph_Template/stm32l1xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0RC1
  * @date    07/02/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_it.h"

#include "usart3_usr.h"
#include "usart2_usr.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//}

/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx_md.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

//-------------------------------------------------------------------------------------------------
//---------------------------------------------USART-2---------------------------------------------
//-------------------------------------------------------------------------------------------------


void DMA1_Channel7_IRQHandler(void)
{
	//
	if((DMA1->ISR & DMA_ISR_TCIF7) == DMA_ISR_TCIF7)
	{
		//1: Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register
		DMA1->IFCR |= DMA_IFCR_CGIF7;

		DMA1_Channel7->CCR &= ~DMA_CCR2_EN;
		
		usart2_usr.dma_wait_tx = 1;
	}
}

//-----------------------------------------------------------------------------

void DMA1_Channel6_IRQHandler(void)
{
	//static uint8_t countRxDMA = 0;

	if((DMA1->ISR & DMA_ISR_TCIF6) == DMA_ISR_TCIF6)
	{
		//1: Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register
		DMA1->IFCR |= DMA_IFCR_CGIF6;

		DMA1_Channel6->CCR &= ~DMA_CCR3_EN;
		
		usart2_usr.dma_wait_rx = 1;
	}
}

//-----------------------------------------------------------------------------

void USART2_IRQHandler(void)
{
	uint32_t status = USART2->SR;
	
	if(status & USART_SR_TXE)
	{
		//Bit 7 TXE: Transmit data register empty
		//This bit is set by hardware when the content of the TDR register
		//has been transferred into the shift register.
		
		if(usart2_usr.cntTx && usart2_usr.lengthTx)
		{
			if(usart2_usr.cntTx < usart2_usr.lengthTx)
			{
				USART2->DR = *(usart2_usr.bufTx + usart2_usr.cntTx);
			}
			usart2_usr.cntTx++;
		}
	}
	if(status & USART_SR_TC)
	{
		usart2_usr.cntTx = 0;
		usart2_usr.lengthTx = 0;
		//Bit 7 TXEIE: TXE interrupt enable
		USART2->CR1 &= ~USART_CR1_TXEIE;
		//Bit 6 TCIE: Transmission complete interrupt enable
		USART2->CR1 &= ~USART_CR1_TCIE;
	}
	if(status & USART_SR_RXNE)
	{
		if(usart2_usr.isRxOverflow)
		{
			(void)USART2->DR;
		}
		else
		{
			if(usart2_usr.cntRx < BUFFER_USART_MAX)
			{
				usart2_usr.bufRx[usart2_usr.cntRx++] = USART2->DR;
			}
			else
			{
				(void)USART2->DR;
				usart2_usr.isRxOverflow = 1;
				usart2_usr.cntRx = 0;
			}
		}
		//USART2->SR &= ~USART_SR_RXNE;
	}
}

//-------------------------------------------------------------------------------------------------
//---------------------------------------------USART-3---------------------------------------------
//-------------------------------------------------------------------------------------------------

void DMA1_Channel2_IRQHandler(void)
{
	//
	if((DMA1->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2)
	{
		//1: Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register
		DMA1->IFCR |= DMA_IFCR_CGIF2;

		DMA1_Channel2->CCR &= ~DMA_CCR2_EN;
		
		usart3_usr.dma_wait_tx = 1;
	}
}

//-----------------------------------------------------------------------------

void DMA1_Channel3_IRQHandler(void)
{
	//static uint8_t countRxDMA = 0;

	if((DMA1->ISR & DMA_ISR_TCIF3) == DMA_ISR_TCIF3)
	{
		//1: Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register
		DMA1->IFCR |= DMA_IFCR_CGIF3;

		DMA1_Channel3->CCR &= ~DMA_CCR3_EN;
		
		usart3_usr.dma_wait_rx = 1;
	}
}

//-----------------------------------------------------------------------------

void USART3_IRQHandler(void)
{
	uint32_t status = USART3->SR;
	
	if(status & USART_SR_TXE)
	{
		//Bit 7 TXE: Transmit data register empty
		//This bit is set by hardware when the content of the TDR register
		//has been transferred into the shift register.
		
		if(usart3_usr.cntTx && usart3_usr.lengthTx)
		{
			if(usart3_usr.cntTx < usart3_usr.lengthTx)
			{
				USART3->DR = *(usart3_usr.bufTx + usart3_usr.cntTx);
			}
			usart3_usr.cntTx++;
		}
	}
	if(status & USART_SR_TC)
	{
		usart3_usr.cntTx = 0;
		usart3_usr.lengthTx = 0;
		//Bit 7 TXEIE: TXE interrupt enable
		USART3->CR1 &= ~USART_CR1_TXEIE;
		//Bit 6 TCIE: Transmission complete interrupt enable
		USART3->CR1 &= ~USART_CR1_TCIE;
	}
	if(status & USART_SR_RXNE)
	{
		if(usart3_usr.isRxOverflow)
		{
			(void)USART3->DR;
		}
		else
		{
			if(usart3_usr.cntRx < BUFFER_USART_MAX)
			{
				usart3_usr.bufRx[usart3_usr.cntRx++] = USART3->DR;
			}
			else
			{
				(void)USART3->DR;
				usart3_usr.isRxOverflow = 1;
				usart3_usr.cntRx = 0;
			}
		}
		//USART3->SR &= ~USART_SR_RXNE;
	}
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

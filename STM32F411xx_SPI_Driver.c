/*
 * STM32F411xx_SPI_Driver.c
 *
 *  Created on: 07-Jan-2024
 *      Author: ramdo
 */
#include "STM32F411xx.h"
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
			else if(pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
			else if(pSPIx == SPI5)
			{
				SPI5_PCLK_EN();
			}
		}
	if(EnorDi == DISABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
			else if(pSPIx == SPI4)
			{
				SPI4_PCLK_DI();
			}
			else if(pSPIx == SPI5)
			{
				SPI5_PCLK_DI();
			}
		}


}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
		}
		else
		{
			pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
		}
}
/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//first let's configure the SPI_CR1 register
	uint32_t tempreg = 0;

	//Enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1.Configure the device mode
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);
	//pSPIHandle->pSPIx->CR1 |= tempreg;
	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE );
		//pSPIHandle->pSPIx->CR1 |= tempreg;
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI mode should be set
		tempreg |= ( 1 << SPI_CR1_BIDIMODE );
		//pSPIHandle->pSPIx->CR1 |= tempreg;
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE );
		//pSPIHandle->pSPIx->CR1 |= tempreg;
		//RXONLY bit must be set
		tempreg |= ( 1 << SPI_CR1_RXONLY );
		//pSPIHandle->pSPIx->CR1 |= tempreg;
	}
	// 3. Configure the SPI serial clock speed (baud rate)
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);
	//pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);
	// 4. Configure the DFF
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);
	//pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);
	// 5. Configure the CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
	//pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
	// 6. Configure the CPOH
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);
	//pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);
	tempreg |= (pSPIHandle->SPIConfig.SPI_BIDIOE << SPI_CR1_BIDIOE);
	pSPIHandle->pSPIx->CR1 |= tempreg;

}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}


/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
			{
				if(IRQNumber <= 31)
				{
					//Program ISER0 register
					*NVIC_ISER0 |= (1 << IRQNumber);
				}
				else if(IRQNumber > 31 && IRQNumber < 64)
				{
					//Program ISER1 register
					*NVIC_ISER1 |= (1 << (IRQNumber % 32));
				}
				else if(IRQNumber >=64 && IRQNumber < 96)
				{
					//Program ISER2 register
					*NVIC_ISER2 |= (1 << (IRQNumber % 64));
				}
			}
		if(EnorDi == DISABLE)
			{
			if(IRQNumber <= 31)
				{
				*NVIC_ICER0 |= (1 << IRQNumber);
				}
			else if(IRQNumber > 31 && IRQNumber < 64)
				{
				*NVIC_ICER1 |= (1 << (IRQNumber % 32));
				}
			else if(IRQNumber >=64 && IRQNumber < 96)
				{
				*NVIC_ICER2 |= (1 << (IRQNumber % 64));
				}
			}


}
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{

	uint8_t iprx = IRQNumber/4;
	uint8_t ipr_section = IRQNumber%4;
	uint8_t shift_amount = (8*ipr_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	//this prevents interrupts from setting up of TXE flag
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	//this prevents interrupts from setting up of TXE flag
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


//implementing helper functions
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// 2. Check the DFF bit in CR1
			if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
			{
				//16 BIT DFF
				//1.Load the data into the data register
				pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->TxLen--;
			    (uint16_t*)pSPIHandle->pTxBuffer++;
			}
			else
			{
				//8 BIT DFF
				*((volatile uint8_t*)&pSPIHandle->pSPIx->DR) = *(pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}
			if(!pSPIHandle->TxLen)
			{
				//TxLen is zero, so close the SPI transmission and the application that
				//TX is over
				//this prevents interrupts from setting up of TXE flag
				SPI_CloseTransmission(pSPIHandle);
				SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
			}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// 2. Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
	{
		//16 BIT DFF
		//1.Load the data from the data register to RxBuffer address
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR ;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	}
	else
	{
		//8 BIT DFF
		*(pSPIHandle->pRxBuffer) = *((volatile uint8_t*)&pSPIHandle->pSPIx->DR);
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(!pSPIHandle->RxLen)
	{
		//TxLen is zero, so close the SPI transmission and the application that
		//TX is over
		//this prevents interrupts from setting up of TXE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1.clear the ovr flag
	uint8_t temp;
	if(pSPIHandle -> TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2.inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_IRQHandling(SPI_Handle_t *pHandle )
{
	uint8_t temp1, temp2;
	//let's check for TXE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}
	//let's check for RXE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		//handle TXE
		spi_rxne_interrupt_handle(pHandle);
	}
	//let's check for OVERRUN flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		//handle TXE
		spi_ovr_err_interrupt_handle(pHandle);
	}
}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len>0)
	{
		// 1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)  == FLAG_RESET );
		// 2. Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 BIT DFF
			//1.Load the data into the data register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
		    (uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 BIT DFF
			*((volatile uint8_t*)&pSPIx->DR) = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
		}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len>0)
		{
			// 1. Wait until TXE is set
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == FLAG_RESET );
			// 2. Check the DFF bit in CR1
			if(pSPIx->CR1 & (1 << SPI_CR1_DFF) )
			{
				//16 BIT DFF
				//1.Load the data from the data register to RxBuffer address
				*((uint16_t*)pRxBuffer) = pSPIx->DR ;
				Len--;
				Len--;
			    (uint16_t*)pRxBuffer++;
			}
			else
			{
				//8 BIT DFF
				*(pRxBuffer) = *((volatile uint8_t*)&pSPIx->DR);
				Len--;
				pRxBuffer++;
			}
			}
}

/*
 * SPI Peripheral Control
 */
void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
	//1 . Save the Tx buffer address and Len information in some global variables
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;
	//2.  Mark the SPI state as busy in transmission so that
	//    no other code can take over same SPI peripheral until transmission is over
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
	//4. Data Transmission will be handled by the ISR code (will implement later)
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
	//1 . Save the Tx buffer address and Len information in some global variables
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Len;
	//2.  Mark the SPI state as busy in transmission so that
	//    no other code can take over same SPI peripheral until transmission is over
	pSPIHandle->RxState = SPI_BUSY_IN_RX;

	//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
	//4. Data Transmission will be handled by the ISR code (will implement later)

}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//This is a weak implementation, the application may override this function

}


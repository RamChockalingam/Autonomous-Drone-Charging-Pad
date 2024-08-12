/*
 * STM32F411xx_GPIO_Driver.c
 *
 *  Created on: 29-Dec-2023
 *      Author: ramdo
 */
#include "STM32F411xx.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	if(EnorDi == DISABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}

}


/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	// Enable the peripheral here itself
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	//1. Configure the mode of GPIO Pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//non interrupt mode
		 temp =  (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		 pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else{
		//this part we will code later
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1.configure the Falling Trigger Register
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1.configure the Rising Trigger Register
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else
		{
			//1.configure both FTSR and RTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2.Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = (portcode<<(4*temp2));
		//3.Enable the EXTI Interrupt delivery using IMR
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;
	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	//3. Configure the PUPD settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	//4. Configure the OPType
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	//5. Configure the alternate functionality

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		  uint8_t temp1,temp2;
		  temp1 = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/8);
		  temp2 =  ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%8);
	      pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0x0F << 4*temp2); //clearing
	      pGPIOHandle->pGPIOx->AFR[temp1] |= ((pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode)<<(4 * temp2));
	}

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}
	else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}
	else if(pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}
	else if(pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}
	else if(pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}
	else if(pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}
}


/*
 * Data read and write
 */

int GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value ;
	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;
}
uint8_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	{
		uint16_t value ;
		value = (uint16_t)(pGPIOx->IDR);
		return value;
	}
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}
/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t ipr_section = IRQNumber%4;
	uint8_t shift_amount = (8*ipr_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);

}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI Pending Register corresponding to the pin number
	if(EXTI->PR &(1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}


/*
 * Peripheral Clock setup
 */

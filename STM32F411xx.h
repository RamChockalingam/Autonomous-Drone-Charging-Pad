/*
 * STM32F411xx.h
 *
 *  Created on: Dec 27, 2023
 *      Author: ramdo
 */
#include <stdio.h>
#include <stdint.h>
#include "STM32F411xx_GPIO_Driver.h"
#include "STM32F411xx_SPI_Driver.h"
#include "STM32F411xx_I2C_Driver.h"
#include "STM32F411xx_USART_Driver.h"
#include "STM32F411xx_RCC_Driver.h"
#include <string.h>
#include <stddef.h>
#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_
#define __vo volatile
#define __weak __attribute__((weak))
/*
 * ARM CORTEX Mx Processor NVIC ISERx register Addresses
 */
#define 	NVIC_ISER0				(__vo uint32_t *)(0xE000E100)
#define 	NVIC_ISER1				(__vo uint32_t *)(0xE000E104)
#define 	NVIC_ISER2				(__vo uint32_t *)(0xE000E108)
#define 	NVIC_ISER3				(__vo uint32_t *)(0xE000E10C)
/*
 * ARM CORTEX Mx processor NVIC ICERx register Addresses
 */
#define 	NVIC_ICER0				(__vo uint32_t *)(0xE000E180)
#define 	NVIC_ICER1				(__vo uint32_t *)(0xE000E184)
#define 	NVIC_ICER2				(__vo uint32_t *)(0xE000E188)
#define 	NVIC_ICER3				(__vo uint32_t *)(0xE000E18C)
/*
 * ARM CORTEX Mx processor NVIC Interrupt Priority register Addresses
 */
#define		NVIC_PR_BASE_ADDR		(__vo uint32_t *)(0xE000E400)
/*
 * ARM CORTEX Mx Processor number of priority bits implemented in a Priority register
 */
#define NO_PR_BITS_IMPLEMENTED		4

/*Base address of FLASH and SRAM memories */
#define 	FLASH_BASEADDR	 		0x08000000U
#define 	SRAM1_BASEADDR			0x20000000U
#define		ROM_BASEADDR			0x1FFF0000U
#define 	SRAM  		  			SRAM1_BASEADDR

/* APBx and AHBx Bus Peripheral base addresses*/
#define 	PERIPH_BASEADDR   		0x40000000U
#define 	APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define 	APB2PERIPH_BASEADDR		0x40010000U
#define 	AHB1PERIPH_BASEADDR		0x40020000U
#define 	AHB2PERIPH_BASEADDR		0x50000000U

/*Base addresses of all peripherals which are hanging on AHB1 bus*/
#define 	GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define 	GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define 	GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define 	GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define 	GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define 	GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define		RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

/*Base addresses of all peripherals which are hanging on APB1 bus*/
#define		I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define		I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define		I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define 	SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define 	SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)


#define		USART1_BASEADDR			(APB1PERIPH_BASEADDR + 0x1000)
#define		USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define		USART6_BASEADDR			(APB1PERIPH_BASEADDR + 0x1400)

#define		TIM2_BASEADDR			(APB1PERIPH_BASEADDR)
#define		TIM3_BASEADDR			(APB1PERIPH_BASEADDR + 0x0400)
#define		TIM4_BASEADDR			(APB1PERIPH_BASEADDR + 0x0800)
#define		TIM5_BASEADDR			(APB1PERIPH_BASEADDR + 0x0C00)
#define		TIM9_BASEADDR			(APB2PERIPH_BASEADDR + 0x4000)
#define		TIM10_BASEADDR			(APB2PERIPH_BASEADDR + 0x4400)
#define		TIM11_BASEADDR			(APB2PERIPH_BASEADDR + 0x4800)


/*Base addresses of all peripherals which are hanging on APB2 bus*/
#define 	SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0X3000)
#define 	SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)
#define 	SPI5_BASEADDR			(APB2PERIPH_BASEADDR + 0x5000)

#define 	EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)

#define 	SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

#define		TIM1_BASEADDR			(APB2PERIPH_BASEADDR)

/* GPIO Register structure definition */
typedef struct
{
 __vo uint32_t MODER;
 __vo uint32_t OTYPER;
 __vo uint32_t OSPEEDR;
 __vo uint32_t PUPDR;
 __vo uint32_t IDR;
 __vo uint32_t ODR;
 __vo uint32_t BSRR;
 __vo uint32_t LCKR;
 __vo uint32_t AFR[2];
} GPIO_RegDef_t;

/* SPI Register structure definition
 *
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;

/*
 * I2C Register structure definition
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
} I2C_RegDef_t;

/*
 * USART Register structure definition
 */
typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
} USART_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
	  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
	  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
	  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
	  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
	  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
	  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
	  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
	  __vo uint32_t APB1RSTR;
	  __vo uint32_t APB2RSTR;
	  uint32_t      RESERVED2[2];     /*!< Reserved, 0x3C                                                       */
	  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
	  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
	  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
	  __vo uint32_t APB1ENR;      /*!< TODO,     										Address offset: 0x20 */
	  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
	  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
	  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
	  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
	  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
	  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
	  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
	  uint32_t      RESERVED7[2];  /*!< Reserved, 0x78-0x7C                                                  */
	  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
	  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
	  uint32_t      RESERVED8[2];  /*!< Reserved, 0x78-0x7C                                                  */
	  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
	  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
	  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
} RCC_RegDef_t;


/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
  __vo uint32_t IMR;
  __vo uint32_t EMR;
  __vo uint32_t RTSR;
  __vo uint32_t FTSR;
  __vo uint32_t SWIER;
  __vo uint32_t PR;
} EXTI_RegDef_t;

/*
 * Peripheral register definition structure for TIMER
 */
typedef struct
{
  __vo uint32_t CR1;
  __vo uint32_t CR2;
  __vo uint32_t SMCR;
  __vo uint32_t DIER;
  __vo uint32_t SR;
  __vo uint32_t EGR;
  __vo uint32_t CCMR1;
  __vo uint32_t CCMR2;
  __vo uint32_t CCER;
  __vo uint32_t CNT;
  __vo uint32_t PSC;
  __vo uint32_t ARR;
  uint32_t RESERVED1;
  __vo uint32_t CCR1;
  __vo uint32_t CCR2;
  __vo uint32_t CCR3;
  __vo uint32_t CCR4;
  uint32_t RESERVED2;
  __vo uint32_t DCR;
  __vo uint32_t DMAR;
  __vo uint32_t OR;
} TIM_RegDef_t;

/*
 *  Peripheral register definition structure for SYSCFG
 */
typedef struct
{
 __vo uint32_t MEMRMP;
 __vo uint32_t PMC;
 __vo uint32_t EXTICR[4];
 uint32_t 	   Reserved[2];
 __vo uint32_t CMPCR;
} SYSCFG_RegDef_t;

/*GPIO peripheral definitions (Peripherals base addresses type casted to xxx_RegDef_t) */
#define GPIOA							((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB							((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC							((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD							((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE							((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH							((GPIO_RegDef_t*)GPIOH_BASEADDR)

/* SPI peripheral definitions (Peripherals base addresses type casted to xxx_RegDef_t) */
#define SPI1							((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2							((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3							((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4							((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5							((SPI_RegDef_t*)SPI5_BASEADDR)

/* I2C peripheral definitions (Peripherals base addresses type casted to xxx_RegDef_t) */
#define I2C1							((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2							((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3							((I2C_RegDef_t*)I2C3_BASEADDR)

#define RCC								((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI							((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG							((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/* Timer peripheral definitions (Peripherals base addresses type casted to xxx_RegDef_t) */
#define TIM1							((TIM_RegDef_t*)TIM1_BASEADDR)
#define TIM2							((TIM_RegDef_t*)TIM2_BASEADDR)
#define TIM3							((TIM_RegDef_t*)TIM3_BASEADDR)
#define TIM4							((TIM_RegDef_t*)TIM4_BASEADDR)
#define TIM5							((TIM_RegDef_t*)TIM5_BASEADDR)
#define TIM9							((TIM_RegDef_t*)TIM9_BASEADDR)
#define TIM10							((TIM_RegDef_t*)TIM10_BASEADDR)
#define TIM11							((TIM_RegDef_t*)TIM11_BASEADDR)

/* USART peripheral definitions (Peripherals base addresses type casted to xxx_RegDef_t) */
#define USART1							((USART_RegDef_t*)USART1_BASEADDR)
#define USART2							((USART_RegDef_t*)USART2_BASEADDR)
#define USART6							((USART_RegDef_t*)USART6_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define	GPIOA_PCLK_EN() 		(RCC->AHB1ENR |= (1<<0))
#define	GPIOB_PCLK_EN() 		(RCC->AHB1ENR |= (1<<1))
#define	GPIOC_PCLK_EN() 		(RCC->AHB1ENR |= (1<<2))
#define	GPIOD_PCLK_EN() 		(RCC->AHB1ENR |= (1<<3))
#define	GPIOE_PCLK_EN() 		(RCC->AHB1ENR |= (1<<4))
#define	GPIOH_PCLK_EN() 		(RCC->AHB1ENR |= (1<<7))

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1<<23))

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1<<13))
#define SPI5_PCLK_EN()			(RCC->APB2ENR |= (1<<20))

/*
 * Clock enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1<<17))
#define USART6_PCLK_EN()		(RCC->APB1ENR |= (1<<5))




/*
 * Clock Enable Macros for TIMx peripherals
 */
#define TIM1_PCLK_EN()			(RCC->APB2ENR |= (1<<0))
#define TIM2_PCLK_EN()			(RCC->APB1ENR |= (1<<0))
#define TIM3_PCLK_EN()			(RCC->APB1ENR |= (1<<1))
#define TIM4_PCLK_EN()			(RCC->APB1ENR |= (1<<2))
#define TIM5_PCLK_EN()			(RCC->APB1ENR |= (1<<3))
#define TIM9_PCLK_EN()			(RCC->APB2ENR |= (1<<16))
#define TIM10_PCLK_EN()			(RCC->APB2ENR |= (1<<17))
#define TIM11_PCLK_EN()			(RCC->APB2ENR |= (1<<18))


/*
 * Clock Enable Macros for SYSCFG Peripheral
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1<<14))

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define	GPIOA_PCLK_DI() 		(RCC->AHB1ENR &= ~(1<<0))
#define	GPIOB_PCLK_DI() 		(RCC->AHB1ENR &= ~(1<<1))
#define	GPIOC_PCLK_DI() 		(RCC->AHB1ENR &= ~(1<<2))
#define	GPIOD_PCLK_DI() 		(RCC->AHB1ENR &= ~(1<<3))
#define	GPIOE_PCLK_DI() 		(RCC->AHB1ENR &= ~(1<<4))
#define	GPIOH_PCLK_DI() 		(RCC->AHB1ENR &= ~(1<<7))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DI()			(RCC->APB2ENR &= ~(1<<20))

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR &= ~(1<<23))

/*
 * Clock enable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<17))
#define USART6_PCLK_DI()		(RCC->APB1ENR &= ~(1<<5))



/*
 * Clock Disable Macros for SYSCFG Peripheral
 */
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1<<14))

/*
 * Macros to reset GPIOx peripherals
 */
#define	GPIOA_REG_RESET()		do{(RCC->AHB1RSTR &= ~(1<<0));(RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define	GPIOB_REG_RESET()		do{(RCC->AHB1RSTR &= ~(1<<1));(RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define	GPIOC_REG_RESET()		do{(RCC->AHB1RSTR &= ~(1<<2));(RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define	GPIOD_REG_RESET()		do{(RCC->AHB1RSTR &= ~(1<<3));(RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define	GPIOE_REG_RESET()		do{(RCC->AHB1RSTR &= ~(1<<4));(RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define	GPIOH_REG_RESET()		do{(RCC->AHB1RSTR &= ~(1<<7));(RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * IRQ(Interrupt Request) Numbers of STM32F411x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15

/*
 *  IRQ(Interrupt Request) Numbers of all SPI peripherals
 */
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			94
#define IRQ_NO_SPI5			85

/*
 *  IRQ Numbers of all I2C peripherals
 */
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73

/*
 *  IRQ Numbers of all USART peripherals
 */
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART6	    71

//some generic MACROS
#define ENABLE 						1
#define DISABLE 					0
#define SET 						ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									 (x == GPIOB) ? 1 :\
									 (x == GPIOC) ? 2 :\
									 (x == GPIOD) ? 3 :\
									 (x == GPIOE) ? 4 :\
									 (x == GPIOH) ? 7 :0)
#define FLAG_RESET 					RESET
#define FLAG_SET					SET
/*********************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/**/

/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/*********************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/**/

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

#endif /* INC_STM32F411XX_H_ */


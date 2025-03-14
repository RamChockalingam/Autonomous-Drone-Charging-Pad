/*
 * STM32F411xx_RCC_Driver.h
 *
 *  Created on: Mar 10, 2024
 *      Author: Ram Chockalingam
 */

#ifndef INC_STM32F411XX_RCC_DRIVER_H_
#define INC_STM32F411XX_RCC_DRIVER_H_

#include "STM32F411xx.h"

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);
//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);
uint32_t  RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F411XX_RCC_DRIVER_H_ */

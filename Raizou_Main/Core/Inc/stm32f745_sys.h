/*
 * stm32f745_sys.h
 *
 *  Created on: Dec 19, 2025
 *      Author: neoki
 */

#ifndef INC_STM32F745_SYS_H_
#define INC_STM32F745_SYS_H_

#include <stm32f7xx.h>

#define SYSCLK		216000000
#define AHBCLK		216000000
#define APB1CLK		54000000
#define APB2CLK		108000000

#define ABS_DIFF(a, b)			((a > b) ? a - b : b - a)
#define FULLTICK(carry, tick)	(((uint64_t)(carry) << 32) | (tick))

typedef enum
{
	SYS_OK		= 0UL,
	SYS_ERROR,
	SYS_BUSY,
	SYS_TIMEOUT
}SysError_t;

typedef void (*CallbackFunc_t)(void);

void RCC_Init(void);

void IncTick(void);
uint64_t GetTick(void);

void delay_ms(uint32_t ms);

#endif /* INC_STM32F745_SYS_H_ */

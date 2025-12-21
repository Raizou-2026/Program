/*
 * stm32f745_interrupt.cpp
 *
 *  Created on: Dec 19, 2025
 *      Author: neoki
 */

#include <stm32f745_sys.h>
#include <stm32f745_can.h>

extern "C" {

extern CAN Can1;

void SysTick_Handler(void)
{
	IncTick();
}

void CAN1_RX0_IRQHandler(void)
{
	Can1.RX0_IRQHander();
}

void CAN1_RX1_IRQHandler(void)
{
	Can1.RX1_IRQHander();
}

}

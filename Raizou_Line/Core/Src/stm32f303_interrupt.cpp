/*
 * stm32f303_interrupt.cpp
 *
 *  Created on: Dec 13, 2025
 *      Author: neoki
 */

#include <stm32f303_sys.h>
#include <stm32f303_can.h>

extern bxCAN Can;

extern "C" {

void SysTick_Handler(void)
{
	IncTick();
}

void CAN_TX_IRQHandler(void)
{

}

void CAN_RX0_IRQHandler(void)
{
	Can.RX0_IRQHandler();
}

void CAN_RX1_IRQHandler(void)
{
	Can.RX1_IRQHandler();
}

}




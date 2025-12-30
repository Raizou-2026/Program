/*
 * stm32f745_adc.h
 *
 *  Created on: Dec 29, 2025
 *      Author: neoki
 */

#ifndef INC_STM32F745_ADC_H_
#define INC_STM32F745_ADC_H_

#include <stm32f745_sys.h>
#include <stm32f745_gpio.h>

void ADC1_Init(void);
void ADC1_DMA_Init(uint8_t* adc_ch, uint8_t size, uint32_t* membufA, uint32_t* membufB);

#endif /* INC_STM32F745_ADC_H_ */

/*
 * stm32f745_adc.cpp
 *
 *  Created on: Dec 29, 2025
 *      Author: neoki
 */

#include <stm32f745_adc.h>

static CallbackFunc_t dma_callback = NULL;
uint8_t can_read_buf = 0;

void ADC1_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	ADC123_COMMON->CCR &= ~ADC_CCR_ADCPRE;
	ADC123_COMMON->CCR |= 0b01UL << ADC_CCR_ADCPRE_Pos;

	ADC1->CR1 &= ~ADC_CR1_RES;
	ADC1->CR2 |= ADC_CR2_EOCS;
}

void ADC1_DMA_Init(uint8_t* adc_ch, uint8_t size, uint32_t* membufA, uint32_t* membufB)
{
	/*DMA config*/
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	while(DMA2_Stream0->CR & DMA_SxCR_EN);
	DMA2_Stream0->CR = 0;

	DMA2_Stream0->CR |= (0b000UL << DMA_SxCR_CHSEL_Pos) |
						DMA_SxCR_DBM 					|
						(0b11UL << DMA_SxCR_PL_Pos)		|
						DMA_SxCR_MSIZE_0				|
						DMA_SxCR_PSIZE_0				|
						DMA_SxCR_MINC					|
						DMA_SxCR_CIRC					|
						DMA_SxCR_TCIE;
//						DMA_SxCR_TEIE;

    DMA2_Stream0->NDTR = (uint32_t)size;

    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)membufA;
    DMA2_Stream0->M1AR = (uint32_t)membufB;

    /*ADC1 config*/
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC123_COMMON->CCR &= ~ADC_CCR_ADCPRE;
    ADC123_COMMON->CCR |= 0b01UL << ADC_CCR_ADCPRE_Pos;

    ADC1->CR1 |= ADC_CR1_SCAN;
    ADC1->CR2 |= ADC_CR2_EOCS | ADC_CR2_DDS | ADC_CR2_DMA | ADC_CR2_CONT | ADC_CR2_ADON;

    ADC1->SQR1 = ADC1->SQR2 = ADC1->SQR3 = 0;
    ADC1->SQR1 |= ((size - 1) << ADC_SQR1_L_Pos);

    for(uint8_t i = 0; i < size; i++) {
    	if(adc_ch[i] >= 10) {
    		ADC1->SMPR1 |= (0b111UL << ((adc_ch[i] - 10) * 3));
    	} else {
    		ADC1->SMPR2 |= (0b111UL << (adc_ch[i] * 3));
    	}

    	if(i < 6) {
    		ADC1->SQR3 |= (adc_ch[i] << (i * 5));
    	} else if(i < 12){
    		ADC1->SQR2 |= (adc_ch[i] << ((i - 7) * 5));
    	} else {
    		ADC1->SQR1 |= (adc_ch[i] << ((i - 13) * 5));
    	}
    }

    NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(3, 0, 0));
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void ADC1_DMA_Start(void)
{
	DMA2_Stream0->CR |= DMA_SxCR_EN;
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

void SetCallbackFunc(CallbackFunc_t func)
{
	dma_callback = func;
}

void ADC1_DMA2_IRQHandler(void)
{
	uint32_t isr = DMA2->LISR;
	uint32_t sxcr = DMA2_Stream0->CR;

	if(isr & DMA_LISR_TCIF0_Msk) {
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;

		if(sxcr & DMA_SxCR_CT_Msk) {
			can_read_buf = 0;
		} else {
			can_read_buf = 1;
		}

		if(dma_callback != NULL) {
			dma_callback();
		}
	}
}

/*
 * stm32f745_timer.cpp
 *
 *  Created on: Dec 31, 2025
 *      Author: neoki
 */

#include <stm32f745_timer.h>

//TIM::TIM(TIM_TypeDef* htim)
//{
//	inst = htim;
//
//	for(uint8_t i = 0; i < 4; i++) {
//		chMode[i]					= TIM_NONE_MODE;
//		CCx_OverCaptureCallback[i]	= NULL;
//	}
//}
//
//TIM::TIM(TIM_TypeDef* htim, uint32_t freqency, uint32_t resolution)
//{
//	inst = htim;
//	freq = frequency;
//	res  = resolution;
//
//	for(uint8_t i = 0; i < 4; i++) {
//		chMode[i]					= TIM_NONE_MODE;
//		CCx_OverCaptureCallback[i]	= NULL;
//	}
//}
//
//SysError_t TIM::init(void)
//{
//	instInit();
//}
//
//void TIM::instInit(void)
//{
//	switch((uint32_t)inst) {
//		case TIM1_BASE:
//			RCC->APB1ENR |= RCC_APB2ENR_TIM1EN;
//			TIMx_IRQn				= TIM1_BRK_TIM9_IRQn;
//			APBxTIMERCLK			= APB2TIMERCLK;
//			afNum					= AF1;
//			ccmax					= 4;
//			break;
//
//		case TIM8_BASE:
//			RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
//			TIMx_IRQn				= TIM8_BRK_TIM12_IRQn;
//			APBxTIMERCLK			= APB2TIMERCLK;
//			afNum					= AF3;
//			ccmax					= 4;
//			break;
//
//		case TIM2_BASE:
//			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
//			TIMx_IRQn				= TIM2_IRQn;
//			APBxTIMERCLK			= APB1TIMERCLK;
//			afNum					= AF1;
//			ccmax					= 4;
//			break;
//
//		case TIM3_BASE:
//			RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
//			TIMx_IRQn				= TIM3_IRQn;
//			APBxTIMERCLK			= APB1TIMERCLK;
//			afNum					= AF2;
//			ccmax					= 4;
//			break;
//
//		case TIM4_BASE:
//			RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
//			TIMx_IRQn				= TIM4_IRQn;
//			APBxTIMERCLK			= APB1TIMERCLK;
//			afNum					= AF2;
//			ccmax					= 4;
//			break;
//
//		case TIM5_BASE:
//			RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
//			TIMx_IRQn				= TIM5_IRQn;
//			APBxTIMERCLK			= APB2TIMERCLK;
//			afNum					= AF2;
//			ccmax					= 4;
//			break;
//
//		case TIM9_BASE:
//			TIMx_IRQn				= TIM1_BRK_TIM9_IRQn;
//			APBxTIMERCLK			= APB1TIMERCLK;
//			afNum					= AF3;
//			ccmax					= 2;
//			break;
//
//		case TIM10_BASE:
//			TIMx_IRQn				= TIM1_UP_TIM10_IRQn;
//			APBxTIMERCLK			= APB1TIMERCLK;
//			afNum					= AF3;
//			ccmax					= 1;
//			break;
//
//		case TIM11_BASE:
//			TIMx_IRQn				= TIM1_TRG_COM_TIM11_IRQn;
//			APBxTIMERCLK			= APB1TIMERCLK;
//			afNum					= AF3;
//			ccmax					= 1;
//			break;
//
//		case TIM12_BASE:
//			TIMx_IRQn				= TIM8_BRK_TIM12_IRQn;
//			APBxTIMERCLK			= APB2TIMERCLK;
//			afNum					= AF9;
//			ccmax					= 2;
//			break;
//
//		case TIM13_BASE:
//			TIMx_IRQn				= TIM8_UP_TIM13_IRQn;
//			APBxTIMERCLK			= APB2TIMERCLK;
//			afNum					= AF9;
//			ccmax					= 1;
//			break;
//
//		case TIM14_BASE:
//			TIMx_IRQn				= TIM8_TRG_COM_TIM14_IRQn;
//			APBxTIMERCLK			= APB2TIMERCLK;
//			afNum					= AF9;
//			ccmax					= 1;
//			break;
//
//		case TIM6_BASE:
//			TIMx_IRQn				= TIM6_DAC_IRQn;
//			APBxTIMERCLK			= APB2TIMERCLK;
//			afNum					= GPIOAF_t(AF15 + 1);
//			ccmax					= 0;
//			break;
//
//		case TIM7_BASE:
//			TIMx_IRQn				= TIM7_IRQn;
//			APBxTIMERCLK			= APB2TIMERCLK;
//			afNum					= GPIOAF_t(AF15 + 1);
//			ccmax					= 0;
//			break;
//	}
//}

void TIM_Init(TIM_TypeDef* htim, uint32_t freq, uint32_t resolution)
{
	if((uint32_t)htim == TIM2_BASE) {
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	} else {
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	}

	htim->PSC = APB1TIMERCLK / (freq * (resolution + 1));
	htim->ARR = resolution;

	htim->CR1 |= TIM_CR1_ARPE;
}

void TIM_PWM_Init(TIM_TypeDef* htim, GPIOPin_t pin, uint8_t ch)
{
	pinMode(pin, OTHER);
	if((uint32_t)htim == TIM2_BASE) {
		AFSelect(pin, AF1);
	} else {
		AFSelect(pin, AF2);
	}

	switch(ch) {
		case 1:
			htim->CCMR1 &= ~TIM_CCMR1_OC1M;
			htim->CCMR1 |= (0b0110UL << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
			htim->CCER |= TIM_CCER_CC1E;
			htim->CCR1 = 0;
			break;

		case 2:
			htim->CCMR1 &= ~TIM_CCMR1_OC2M;
			htim->CCMR1 |= (0b0110UL << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
			htim->CCER |= TIM_CCER_CC2E;
			htim->CCR2 = 0;
			break;

		case 3:
			htim->CCMR2 &= ~TIM_CCMR2_OC3M;
			htim->CCMR2 |= (0b0110UL << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
			htim->CCER |= TIM_CCER_CC3E;
			htim->CCR3 = 0;
			break;

		case 4:
			htim->CCMR2 &= ~TIM_CCMR2_OC4M;
			htim->CCMR2 |= (0b0110UL << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
			htim->CCER |= TIM_CCER_CC4E;
			htim->CCR4 = 0;
			break;
	}
}

void TIM_Start(TIM_TypeDef* htim)
{
	htim->EGR |= TIM_EGR_UG;
	htim->CR1 |= TIM_CR1_CEN;
}

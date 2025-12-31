/*
 * stm32f745_timer.h
 *
 *  Created on: Dec 31, 2025
 *      Author: neoki
 */

#ifndef INC_STM32F745_TIMER_H_
#define INC_STM32F745_TIMER_H_

#include <stm32f745_sys.h>
#include <stm32f745_gpio.h>

//typedef enum {
//	TIM_NONE_MODE,
//	TIM_UPCOUNT_MODE,
//	TIM_DOWNCOUNT_MODE,
//	TIM_UPDOWNCOUNT_MODE,
//	TIM_PWM1_MODE,
//	TIM_PWM2_MODE
//} TIMMode_t;
//
//class TIM {
//	public:
//		TIMMode_t			chMode[4];
//
//		/*setting function*/
//		TIM(TIM_TypeDef* htim);
//		TIM(TIM_TypeDef* htim, uint32_t freqency, uint32_t resolution);
//		SysError_t init(void);
//		SysError_t init(uint32_t freqency, uint32_t resolution);
//		SysError_t pwmInit(GPIOPin_t pin, uint8_t ch);
//		SysError_t start(void);
//
//		/*Alternate handler*/
//		void IRQ_Handler(void);
//
//	private:
//		/*TIMx settings*/
//		TIM_TypeDef*		inst;
//		IRQn_Type			TIMx_IRQn;
//		uint32_t			APBxTIMERCLK;
//		uint32_t			freq;
//		uint32_t			res;
//		GPIOAF_t			afNum;
//
//		/*TIMx_CHy settings*/
//		uint8_t				ccmax;
//		CallbackFunc_t		CCx_OverCaptureCallback[4];
//
//		void instInit(void);
//};

void TIM_Init(TIM_TypeDef* htim, uint32_t freq, uint32_t resolution);
void TIM_PWM_Init(TIM_TypeDef* htim, GPIOPin_t pin, uint8_t ch);
void TIM_Start(TIM_TypeDef* htim);

#endif /* INC_STM32F745_TIMER_H_ */

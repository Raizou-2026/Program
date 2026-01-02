/*
 * user_main.h
 *
 *  Created on: Jan 2, 2026
 *      Author: neoki
 */

#ifndef INC_USER_MAIN_H_
#define INC_USER_MAIN_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stm32f745_sys.h>
#include <stm32f745_gpio.h>
#include <stm32f745_usart.h>
#include <stm32f745_can.h>
#include <stm32f745_adc.h>
#include <stm32f745_timer.h>

#define ARM_MATH_CM7
extern "C" {
	#include "arm_math.h"
}

/*trigonometric constant & macro*/
#define DEGTORAD(x)			((x) * PI / 180.0)
#define RADTODEG(x)			((x) * 180.0 / PI)

/*GPIO define*/
/*INPUT/OUTPUT*/
#define LED1				PD2
#define LED2				PD3

/*CAN*/
#define CAN1_RX				PB8
#define CAN1_TX				PB9

/*TIMx_PWM output*/
#define TIM2_CH1			PA0
#define TIM2_CH2			PA1
#define TIM2_CH3			PA2
#define TIM2_CH4			PA3
#define TIM3_CH1			PA6
#define TIM3_CH2			PA7
#define TIM3_CH3			PC8
#define TIM3_CH4			PC9

/*ADC channel*/
#define BALL_ADCIN4			PA4
#define BALL_ADCIN5			PA5
#define BALL_ADCIN10		PC0
#define BALL_ADCIN11		PC1
#define BALL_ADCIN12		PC2
#define BALL_ADCIN13		PC3
#define BALL_ADCIN14		PC4
#define BALL_ADCIN15		PC5
#define DISP_ADCIN8			PB0
#define DISP_ADCIN9			PB1

/*CAN ID*/
#define CANID_LINERAW_T		(0x010)
#define CANID_LINEANG_T		(0x011)
#define CANID_LINERAW_R		(0x100)
#define CANID_LINEANG_R		(0x101)

#endif /* INC_USER_MAIN_H_ */

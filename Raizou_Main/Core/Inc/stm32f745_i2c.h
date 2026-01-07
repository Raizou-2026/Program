/*
 * stm32f745_i2c.h
 *
 *  Created on: Jan 8, 2026
 *      Author: neoki
 */

#ifndef INC_STM32F745_I2C_H_
#define INC_STM32F745_I2C_H_

#include <stm32f745_sys.h>
#include <stm32f745_gpio.h>

class I2C {
	public:
		/*timing setting*/
		uint32_t			speedFreq;
		uint16_t			riseTime;
		uint16_t			fallTime;
		uint8_t				dFilter;
		bool				aFilter;

		/*setting*/
		I2C(I2C_TypeDef* hi2c, GPIOPin_t scl, GPIOPin_t sda);
		SysError_t init(void);
//		void setCallback(void);

		/*master transmit & receive*/
		SysError_t masterTransmit(uint8_t address, uint8_t* buf, uint16_t size, uint64_t timeout);
//		SysError_t masterTransmitIT(uint8_t address, uint8_t* buf, uint16_t size);
//		SysError_t masterTransmitDMA(uint8_t address, uint8_t* buf, uint16_t size);
		SysError_t masterReceive(uint8_t address, uint8_t* buf, uint16_t size, uint64_t timeout);
//		SysError_t masterReceiveIT(uint8_t address, uint8_t* buf, uint16_t size);
//		SysError_t masterReceiveDMA(uint8_t address, uint8_t* buf, uint16_t size);

	private:
		I2C_TypeDef*		ch;
		IRQn_Type			I2Cx_IRQn;
		GPIOPin_t			scl_pin, sda_pin;

		uint32_t calcTiming(void);
		void pinInit(void);
};

#endif /* INC_STM32F745_I2C_H_ */

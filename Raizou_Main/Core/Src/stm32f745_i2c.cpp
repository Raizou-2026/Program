/*
 * stm32f745_i2c.cpp
 *
 *  Created on: Jan 8, 2026
 *      Author: neoki
 */

#include <stm32f745_i2c.h>

I2C::I2C(I2C_TypeDef* hi2c, GPIOPin_t scl, GPIOPin_t sda)
{
	ch				= hi2c;
	scl_pin			= scl;
	sda_pin			= sda;
	speedFreq		= 100000;
	riseTime		= 0;
	fallTime		= 0;
	dFilter			= 0;
	aFilter			= ENABLE;
}

SysError_t I2C::init(void)
{
	switch((uint32_t)ch) {
		case I2C1_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
			break;
		case I2C2_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
			break;
		case I2C3_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
			break;
		default:
			return SYS_ERROR;
	}

	ch->CR1 = 0;
	ch->TIMINGR = 0x20404768;

	pinInit();

	ch->CR1 |= I2C_CR1_PE;

	return SYS_OK;
}

SysError_t I2C::masterTransmit(uint8_t address,
                               uint8_t* buf,
                               uint16_t size,
                               uint64_t timeout)
{
    uint64_t start = GetTick() / 1000;
    uint16_t i;

    if (ch->ISR & I2C_ISR_STOPF) {
        ch->ICR = I2C_ICR_STOPCF;
    }
    if (ch->ISR & I2C_ISR_NACKF) {
        ch->ICR = I2C_ICR_NACKCF;
    }

    while (ch->ISR & I2C_ISR_BUSY) {
        if ((GetTick() / 1000 - start) >= timeout) {
            return SYS_TIMEOUT;
        }
    }

    ch->CR2 =
        (address << 1) |
        (size << I2C_CR2_NBYTES_Pos) |
        I2C_CR2_START |
        I2C_CR2_AUTOEND;

    for (i = 0; i < size; i++) {
        while (!(ch->ISR & I2C_ISR_TXIS)) {
            if ((GetTick() / 1000 - start) >= timeout) {
                ch->CR2 |= I2C_CR2_STOP;
                return SYS_TIMEOUT;
            }
            if (ch->ISR & I2C_ISR_NACKF) {
                ch->ICR = I2C_ICR_NACKCF;
                ch->CR2 |= I2C_CR2_STOP;
                return SYS_ERROR;
            }
        }
        ch->TXDR = buf[i];
    }

    while (!(ch->ISR & I2C_ISR_STOPF)) {
        if ((GetTick() / 1000 - start) >= timeout) {
            return SYS_TIMEOUT;
        }
    }

    ch->ICR = I2C_ICR_STOPCF;

    return SYS_OK;
}

SysError_t I2C::masterReceive(uint8_t address,
                              uint8_t* buf,
                              uint16_t size,
                              uint64_t timeout)
{
    uint64_t start = GetTick() / 1000;
    uint16_t i;

    if (ch->ISR & I2C_ISR_STOPF) {
        ch->ICR = I2C_ICR_STOPCF;
    }
    if (ch->ISR & I2C_ISR_NACKF) {
        ch->ICR = I2C_ICR_NACKCF;
    }

    while (ch->ISR & I2C_ISR_BUSY) {
        if ((GetTick() / 1000 - start) >= timeout) {
            return SYS_TIMEOUT;
        }
    }

    ch->CR2 =
        (address << 1) |
        I2C_CR2_RD_WRN |
        (size << I2C_CR2_NBYTES_Pos) |
        I2C_CR2_START |
        I2C_CR2_AUTOEND;

    for (i = 0; i < size; i++) {
        while (!(ch->ISR & I2C_ISR_RXNE)) {
            if ((GetTick() / 1000 - start) >= timeout) {
                ch->CR2 |= I2C_CR2_STOP;
                return SYS_TIMEOUT;
            }
            if (ch->ISR & I2C_ISR_NACKF) {
                ch->ICR = I2C_ICR_NACKCF;
                ch->CR2 |= I2C_CR2_STOP;
                return SYS_ERROR;
            }
        }
        buf[i] = ch->RXDR;
    }

    while (!(ch->ISR & I2C_ISR_STOPF)) {
        if ((GetTick() / 1000 - start) >= timeout) {
            return SYS_TIMEOUT;
        }
    }

    ch->ICR = I2C_ICR_STOPCF;
    return SYS_OK;
}

uint32_t I2C::calcTiming(void)
{
	return 0;
}

void I2C::pinInit(void)
{
	pinMode(scl_pin, OTHER);		pinMode(sda_pin, OTHER);
	pinSpeed(scl_pin, HIGHSPEED);	pinSpeed(sda_pin, HIGHSPEED);
	AFSelect(scl_pin, AF4);			AFSelect(sda_pin, AF4);
}

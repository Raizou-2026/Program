#ifndef __MAIN_H
#define __MAIN_H

#include <stm32f303_sys.h>
#include <stm32f303_gpio.h>
#include <stm32f303_can.h>

#define LINEIN0		PB0
#define LINEIN1		PB1
#define LINEIN2		PB3
#define LINEIN3		PB4
#define LINEIN4		PB5
#define LINEIN5		PB6
#define LINEIN6		PB7
#define LINEIN7		PA0
#define LINEIN8		PA1
#define LINEIN9		PA2
#define LINEIN10	PA3
#define LINEIN11	PA4
#define LINEIN12	PA5
#define LINEIN13	PA6
#define LINEIN14	PA7
#define LINEIN15	PA8
#define LINEIN16	PA9
#define LINEIN17	PA10
#define CAN_RX		PA11
#define CAN_TX		PA12
#define DEBUG_LED	PA15

#define CAN_BITRATE	(1000000)

#endif /* __MAIN_H */

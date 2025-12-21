#include <stdio.h>
#include <stm32f745_sys.h>
#include <stm32f745_gpio.h>
#include <stm32f745_usart.h>
#include <stm32f745_can.h>

#define LED1			PD2
#define LED2			PD3
#define CAN1_RX			PB8
#define CAN1_TX			PB9

#define CAN1_BITRATE	(1000000)

CAN Can1(CAN1, CAN1_RX, CAN1_TX);

void CAN1_Init(void);
void GPIO_Init(void);

void CAN1_FIFO0ReceivedCallback(void);

bool can1_rx0_state = false;

int main(void)
{
	RCC_Init();
	CAN1_Init();
	GPIO_Init();

	Serial.init(115200);

	setvbuf(stdout, NULL, _IONBF, 0);

	CANRxHeader_t rxdata;

	while(1)
	{
		if(can1_rx0_state == false) {
			Can1.receiveIT(&rxdata, FIFO0);
		} else {
			printf("ID:%lx   ", (uint32_t)rxdata.id);
			for(uint8_t i = 0; i < 8; i++) {
				printf("%lx  ", (uint32_t)rxdata.data[i]);
			}
			printf("\n");

			can1_rx0_state = false;
		}
	}

	return 0;
}

void CAN1_Init(void)
{
	CANFilter_t filter0 = {
			.filter_num 	= 0,
			.mode			= MASK_MODE,
			.scale			= SINGLE32BIT,
			.fifo_assign	= FIFO0,
			.active			= true,

			.id_type		= STID,
			.frame_type		= DATA_FRAME,
			.id				= 0x100,
			.mask			= 0x7FF
	};

	Can1.init(CAN1_BITRATE);
	Can1.setCallback(CAN_RX0_COMPLETE, CAN1_FIFO0ReceivedCallback);
	Can1.singleFilter(filter0);
	Can1.start();
}

void GPIO_Init(void)
{
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
}

void CAN1_FIFO0ReceivedCallback(void)
{
	can1_rx0_state = true;
}

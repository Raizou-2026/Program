#include "main.h"

bxCAN Can(CAN, CAN_RX, CAN_TX);

void CAN_Init(void);
void GPIO_Init(void);

int main(void)
{
	RCC_Init();
	SysTick_Init();
	CAN_Init();
	GPIO_Init();

	CANTxHeader_t txdata = {
			.id			= 0x100,
			.id_type	= STID,
			.rtr		= DATA_FRAME,
			.dlc		= 8
	};
	txdata.data[0] = 0x00;
	txdata.data[1] = 0x11;
	txdata.data[2] = 0x22;
	txdata.data[3] = 0x33;
	txdata.data[4] = 0x44;
	txdata.data[5] = 0x55;
	txdata.data[6] = 0x66;
	txdata.data[7] = 0x77;

	while(1)
	{
		Can.transmit(&txdata, 1000);
		delay(5000);
	}

	return 0;
}

void CAN_Init(void)
{
	CANFilter_t filter0 = {
			.filter_num 	= 0,
			.mode			= MASK_MODE,
			.scale			= SINGLE32BIT,
			.fifo_assign	= FIFO0,
			.active			= true,

			.id_type		= STID,
			.frame_type		= DATA_FRAME,
			.id				= 0x120,
			.mask			= 0x7FF
	};

	Can.init(CAN_BITRATE);
	Can.singleFilter(filter0);
	Can.start();
}

void GPIO_Init(void)
{
	pinMode(LINEIN0, INPUT);
	pinMode(LINEIN1, INPUT);
	pinMode(LINEIN2, INPUT);
	pinMode(LINEIN3, INPUT);
	pinMode(LINEIN4, INPUT);
	pinMode(LINEIN5, INPUT);
	pinMode(LINEIN6, INPUT);
	pinMode(LINEIN7, INPUT);
	pinMode(LINEIN8, INPUT);
	pinMode(LINEIN9, INPUT);
	pinMode(LINEIN10, INPUT);
	pinMode(LINEIN11, INPUT);
	pinMode(LINEIN12, INPUT);
	pinMode(LINEIN13, INPUT);
	pinMode(LINEIN14, INPUT);
	pinMode(LINEIN15, INPUT);
	pinMode(LINEIN16, INPUT);
	pinMode(LINEIN17, INPUT);

	pinMode(DEBUG_LED, OUTPUT);
}

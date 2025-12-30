#include "main.h"

bxCAN Can(CAN, CAN_RX, CAN_TX);

void CAN_Init(void);
void GPIO_Init(void);

void Line_DataRead(uint8_t* buf);
double Line_AngleRead(uint8_t* buf);

void CAN_FIFO0ReceivedCallback(void);

static bool can_rx0_state = false;

int main(void)
{
	CANTxHeader_t linedata = {
			.id			= 0x100,
			.id_type	= STID,
			.rtr		= DATA_FRAME,
			.dlc		= 3
	};

	CANRxHeader_t rx0header;
	double angle;
	uint8_t rawdata[3];
	uint8_t* ptemp = (uint8_t*)(&angle);

	RCC_Init();
	SysTick_Init();
	CAN_Init();
	GPIO_Init();

	Line_DataRead(rawdata);
	angle = Line_AngleRead(rawdata);

	while(1)
	{
		static bool is_setfunc = false;

		Line_DataRead(rawdata);
		angle = Line_AngleRead(rawdata);

		if(can_rx0_state == false && is_setfunc == false) {
			is_setfunc = true;
			Can.receiveIT(&rx0header, FIFO0);
		} else if(can_rx0_state == true) {
			can_rx0_state = is_setfunc = false;

			if(rx0header.rtr == REMOTE_FRAME) {
				linedata.id = 0x100 | (rx0header.id & 0x00F);
				linedata.dlc = rx0header.dlc;

				switch(rx0header.id) {
					case CANID_RAWDATA_R:
						linedata.data[0] = rawdata[0];
						linedata.data[1] = rawdata[1];
						linedata.data[2] = rawdata[2];
						break;
					case CANID_ANGDATA_R:
						for(uint8_t i = 0; i < 8; i++) {
							linedata.data[i] = ptemp[i];
						}
						break;
				}
			}

			Can.transmit(&linedata, 1000);
		}

		delay(1);
	}

	return 0;
}

void CAN_Init(void)
{
	CANFilter_t filter = {
			.filter_num 	= 0,
			.mode			= MASK_MODE,
			.scale			= SINGLE32BIT,
			.fifo_assign	= FIFO0,
			.active			= true,

			.id_type		= STID,
			.frame_type		= REMOTE_FRAME,
			.id				= CANID_RAWDATA_R,
			.mask			= 0x7FF
	};

	Can.init(CAN_BITRATE);
	Can.setCallback(CAN_RX0_COMPLETE, CAN_FIFO0ReceivedCallback);

	Can.singleFilter(filter);
	filter.filter_num 	= 1;
	filter.id			= CANID_ANGDATA_R;
	Can.singleFilter(filter);

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

void Line_DataRead(uint8_t* buf)
{
	uint16_t portA, portB;

	portA = GPIOA->IDR;
	portB = GPIOB->IDR;

	buf[0] = (portA & 0b11111111000) >> 3;
	buf[1] = ((portA & 0b111) << 5) | ((portB & 0b11111000) >> 3);
	buf[2] = portB & 0b11;
}

double Line_AngleRead(uint8_t* buf)
{
	double x = 0.0;
	double y = 0.0;
	double angle;
	uint8_t pos;

	for(uint8_t i = 0; i < 8; i++) {
		pos = (1 << i);

		if(!(buf[0] & pos)) {
			angle = (360.0 - (i * 22.5)) * PI / 180.0;
			x += cos(angle);
			y += sin(angle);
		}
		if(!(buf[1] & pos)) {
			angle = (360.0 - ((i + 8) * 22.5)) * PI / 180.0;
			x += cos(angle);
			y += sin(angle);
		}
	}

	return atan2(y, x) * 180.0 / PI;
}

void CAN_FIFO0ReceivedCallback(void)
{
	can_rx0_state = true;
	GPIOA->ODR ^= (1 << 15);
}

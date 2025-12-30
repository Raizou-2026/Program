#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stm32f745_sys.h>
#include <stm32f745_gpio.h>
#include <stm32f745_usart.h>
#include <stm32f745_can.h>
#include <stm32f745_adc.h>

#define LED1			PD2
#define LED2			PD3
#define CAN1_RX			PB8
#define CAN1_TX			PB9

#define CAN1_BITRATE	(1000000)

#define PI				3.1415926535

void CAN1_Init(void);
void GPIO_Init(void);

uint16_t ADC1_Read(uint8_t ch);

double Ball_Angle(void);

uint32_t Line_DataRequest(uint32_t timeout);
double Line_AngleRequest(uint32_t timeout);

void CAN1_FIFO0ReceivedCallback(void);

CAN Can1(CAN1, CAN1_RX, CAN1_TX);
bool can1_rx0_state = false;

const uint8_t ball_ch[8] = {15, 4, 14, 5, 13, 10, 12, 11};
uint16_t ballVal[2][8] = {0};

int main(void)
{
	RCC_Init();
	CAN1_Init();
	GPIO_Init();
	ADC1_DMA_Init((uint8_t*)ball_ch, 8, (uint32_t*)&ballVal[0][0], (uint32_t*)&ballVal[1][0]);

	Serial.init(115200);

	setvbuf(stdout, NULL, _IONBF, 0);

	while(1)
	{
//		double lineAngle = Line_AngleRequest(1000);
//		printf("%f\n", lineAngle);
//
//		pinToggle(LED1);
//		pinToggle(LED2);

		printf("%d %d %d %d\n", (int)ballVal[0][0], (int)ballVal[0][1], (int)ballVal[1][0], (int)ballVal[1][1]);
		delay_ms(500);
	}

	return 0;
}

void CAN1_Init(void)
{
	CANFilter_t filter = {
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

	Can1.singleFilter(filter);
	filter.filter_num = 1;	filter.id = 0x101;
	Can1.singleFilter(filter);

	Can1.start();
}

void GPIO_Init(void)
{
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	pinWrite(LED1, HIGH);
	pinWrite(LED2, LOW);

	pinMode(PA4, ANALOG);
	pinMode(PA5, ANALOG);
	pinMode(PB0, ANALOG);
	pinMode(PB1, ANALOG);
	pinMode(PC0, ANALOG);
	pinMode(PC1, ANALOG);
	pinMode(PC2, ANALOG);
	pinMode(PC3, ANALOG);
	pinMode(PC4, ANALOG);
	pinMode(PC5, ANALOG);
}

double Ball_Angle(void)
{
	double x = 0.0;
	double y = 0.0;
	uint16_t val;

	for(uint8_t i = 0; i < 8; i++) {
		val = ADC1_Read(ball_ch[i]);
		x += cos(45.0 * i * PI / 180.0) * (4095 - val);
		y += sin(45.0 * i * PI / 180.0) * (4095 - val);
	}

	return atan2(y, x) * 180.0 / PI;
}

uint32_t Line_DataRequest(uint32_t timeout)
{
	CANTxHeader_t reqdata = {0x010, STID, REMOTE_FRAME, 3};
	CANRxHeader_t rx0header;
	uint32_t data = 0;
	uint64_t start = GetTick() / 1000;

	can1_rx0_state = false;
	Can1.transmit(&reqdata, 100);
	Can1.receiveIT(&rx0header, FIFO0);

	while(can1_rx0_state == false) {
		if((GetTick() / 1000 - start) >= timeout) {
			return data;
		}
	}

	if(rx0header.id == 0x100) {
		data = (rx0header.data[0] << 10) | (rx0header.data[1] << 2) | (rx0header.data[2] & 0b11);
	}

	return data;
}

double Line_AngleRequest(uint32_t timeout)
{
	CANTxHeader_t reqdata = {0x011, STID, REMOTE_FRAME, 8};
	CANRxHeader_t rx0header;
	double angle = 300.0;
	uint64_t start = GetTick() / 1000;

	can1_rx0_state = false;
	Can1.transmit(&reqdata, 100);
	Can1.receiveIT(&rx0header, FIFO0);

	while(can1_rx0_state == false) {
		if((GetTick() / 1000 - start) >= timeout) {
			return angle;
		}
	}

	if(rx0header.id == 0x101) {
		memcpy(&angle, rx0header.data, sizeof(double));
	}

	return angle;
}

void CAN1_FIFO0ReceivedCallback(void)
{
	can1_rx0_state = true;
}

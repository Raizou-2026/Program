#include <user_main.h>

#define CAN1_BITRATE	(1000000)

#define TIM2_FREQ		(1000)
#define TIM3_FREQ		(1000)
#define TIM2_RES		(1023)
#define TIM3_RES		(1023)

CAN Can1(CAN1, CAN1_RX, CAN1_TX);
bool can1_rx0_state = false;

double motorPhase[4];

const uint8_t ball_ch[8] = {15, 4, 14, 5, 13, 10, 12, 11};
extern uint8_t can_read_buf;
uint16_t ballVal[2][8] = {0};
double ballPhase[8];

void CAN1_Init(void);
void Motor_Init(void);
void GPIO_Init(void);

void Motor_Start(void);
void Motor_StraightSpeed_d(uint16_t speed, double angle, double* pspeed);
void Motor_StraightSpeed(uint16_t speed, float angle, float* pspeed);

float Ball_Angle(void);

uint32_t Line_DataRequest(uint32_t timeout);
double Line_AngleRequest(uint32_t timeout);

void CAN1_FIFO0ReceivedCallback(void);

int main(void)
{
	RCC_Init();
	CAN1_Init();
	Motor_Init();
	GPIO_Init();
	ADC1_DMA_Init((uint8_t*)ball_ch, 8, (uint32_t*)&ballVal[0][0], (uint32_t*)&ballVal[1][0]);

	Serial.init(115200);
	setvbuf(stdout, NULL, _IONBF, 0);

	for(uint8_t i = 0; i < 8; i++) {
		ballPhase[i] = DEGTORAD(45.0 * i);
		if(i % 2 == 1) {
			motorPhase[i / 2] = ballPhase[i];
		}
	}

	ADC1_DMA_Start();
	Motor_Start();

	while(1)
	{
//		double lineAngle, ballAngle;
//		lineAngle = Line_AngleRequest(1000);
//		ballAngle = Ball_Angle();
//
//		printf("BufNum:%d BAngle:%f LAngle:%f\n", (int)can_read_buf, ballAngle, lineAngle);
//
//		pinToggle(LED1);
//		pinToggle(LED2);
//
//		for(uint8_t i = 0; i < 1024; i++) {
//			TIM2->CCR1 = TIM2->CCR2 = TIM2->CCR3 = TIM2->CCR4 = i;
//			TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = i;
//			delay_ms(10);
//		}

		float mspeed[4];
		uint64_t start, end;

		start = GetTick();
		for(uint16_t i = 0; i < 360; i++) {
			//Motor_StraightSpeed_d(1023, (double)i, mspeed);
			Motor_StraightSpeed(1023, (float)i, mspeed);
		}
		end = GetTick();

		printf("%ld\n", (long)(end - start));
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

	Can1.rx0_priority = NVIC_EncodePriority(3, 1, 0);
	Can1.rx1_priority = NVIC_EncodePriority(3, 2, 0);

	Can1.init(CAN1_BITRATE);
	Can1.setCallback(CAN_RX0_COMPLETE, CAN1_FIFO0ReceivedCallback);

	Can1.singleFilter(filter);
	filter.filter_num = 1;	filter.id = 0x101;
	Can1.singleFilter(filter);

	Can1.start();
}

void Motor_Init(void)
{
	TIM_Init(TIM2, TIM2_FREQ, TIM2_RES);
	TIM_Init(TIM3, TIM3_FREQ, TIM3_RES);

	TIM_PWM_Init(TIM2, TIM2_CH1, 1);
	TIM_PWM_Init(TIM2, TIM2_CH2, 2);
	TIM_PWM_Init(TIM2, TIM2_CH3, 3);
	TIM_PWM_Init(TIM2, TIM2_CH4, 4);
	TIM_PWM_Init(TIM3, TIM3_CH1, 1);
	TIM_PWM_Init(TIM3, TIM3_CH2, 2);
	TIM_PWM_Init(TIM3, TIM3_CH3, 3);
	TIM_PWM_Init(TIM3, TIM3_CH4, 4);
}

void GPIO_Init(void)
{
	pinMode(LED1, OUTPUT);	pinWrite(LED1, HIGH);
	pinMode(LED2, OUTPUT);	pinWrite(LED2, LOW);

	pinMode(BALL_ADCIN4, ANALOG);
	pinMode(BALL_ADCIN5, ANALOG);
	pinMode(BALL_ADCIN10, ANALOG);
	pinMode(BALL_ADCIN11, ANALOG);
	pinMode(BALL_ADCIN12, ANALOG);
	pinMode(BALL_ADCIN13, ANALOG);
	pinMode(BALL_ADCIN14, ANALOG);
	pinMode(BALL_ADCIN15, ANALOG);
	pinMode(DISP_ADCIN8, ANALOG);
	pinMode(DISP_ADCIN9, ANALOG);
}

void Motor_Start(void)
{
	TIM_Start(TIM2);
	TIM_Start(TIM3);
}

void Motor_StraightSpeed_d(uint16_t speed, double angle, double* pspeed)
{
	uint8_t max = 0;
	double __angle, gain;

	__angle = DEGTORAD(angle);
	for(uint8_t i = 0; i < 4; i++) {
		pspeed[i] = sin(__angle - motorPhase[i]) * speed;
		if(abs(pspeed[max]) <= abs(pspeed[i])) {
			max = i;
		}
	}

	gain = speed / abs(pspeed[max]);
	for(uint8_t i = 0; i < 4; i++) {
		pspeed[i] *= gain;
	}
}

void Motor_StraightSpeed(uint16_t speed, float angle, float* pspeed)
{
	uint8_t max = 0;
	float __angle, gain;

	__angle = DEGTORAD(angle);
	for(uint8_t i = 0; i < 4; i++) {
		pspeed[i] = arm_sin_f32(__angle - motorPhase[i]) * speed;
		if(fabsf(pspeed[max]) <= fabsf(pspeed[i])) {
			max = i;
		}
	}

	gain = (float)speed / abs(pspeed[max]);
	for(uint8_t i = 0; i < 4; i++) {
		pspeed[i] *= gain;
	}
}

float Ball_Angle(void)
{
	float x = 0.0;
	float y = 0.0;
	uint16_t val;

	for(uint8_t i = 0; i < 8; i++) {
		val = 4095 - ballVal[0][i];
		x += arm_cos_f32(ballPhase[i]) * val;
		y += arm_sin_f32(ballPhase[i]) * val;
	}

	return RADTODEG(atan2f(y, x));
}

uint32_t Line_DataRequest(uint32_t timeout)
{
	CANTxHeader_t reqdata = {CANID_LINERAW_T, STID, REMOTE_FRAME, 3};
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

	if(rx0header.id == CANID_LINERAW_R) {
		data = (rx0header.data[0] << 10) | (rx0header.data[1] << 2) | (rx0header.data[2] & 0b11);
	}

	return data;
}

double Line_AngleRequest(uint32_t timeout)
{
	CANTxHeader_t reqdata = {CANID_LINEANG_T, STID, REMOTE_FRAME, 8};
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

	if(rx0header.id == CANID_LINEANG_R) {
		memcpy(&angle, rx0header.data, sizeof(double));
	}

	return angle;
}

void CAN1_FIFO0ReceivedCallback(void)
{
	can1_rx0_state = true;
}

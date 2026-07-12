#include <user_main.h>

#define CAN1_BITRATE	(1000000)

#define BNO_ADDRESS		0x28
#define BNO_EULER		0x1A

#define TIM2_FREQ		(1000)
#define TIM3_FREQ		(1000)
#define TIM2_RES		(1023)
#define TIM3_RES		(1023)

typedef struct
{
	float	radius;
	float	angle;
	bool	valid;
} BallLineInfo_t;

CAN Can1(CAN1, CAN1_RX, CAN1_TX);
bool can1_rx0_state = false;

I2C i2c1(I2C1, PB6, PB7);

float motorPhase[4];

const uint8_t ball_ch[8] = {15, 4, 14, 5, 13, 10, 12, 11};
extern uint8_t can_read_buf;
uint16_t ballVal[2][8] = {0};
float ballPhase[8];

bool bnoOK = true;

void CAN1_Init(void);
void Motor_Init(void);
void GPIO_Init(void);

int16_t BNO_Marge(uint8_t low, uint8_t high);
void BNO_Write(uint8_t reg, uint8_t val, uint32_t delay);
void BNO_Init(void);
float BNO_GetYaw(void);

void Motor_Start(void);
void Motor_StraightSpeed_d(uint16_t speed, double angle, double* pspeed);
void Motor_StraightSpeed(uint16_t speed, float angle, float* pspeed);
void Motor_SpinSpeed(float* pspeed);
void Motor_Control(float* spin, float* straight, uint16_t max);

void Ball_Kick(void);

void Ball_Angle(BallLineInfo_t* pData);

uint32_t Line_DataRequest(uint32_t timeout);
void Line_AngleRequest(BallLineInfo_t* pData, uint32_t timeout);

void CAN1_FIFO0ReceivedCallback(void);

int main(void)
{
	uint8_t cnt = 0;

	RCC_Init();
	CAN1_Init();
	i2c1.init(400000);
	Serial.init(115200);
	Motor_Init();
	GPIO_Init();
	ADC1_DMA_Init((uint8_t*)ball_ch, 8, (uint32_t*)&ballVal[0][0], (uint32_t*)&ballVal[1][0]);

	setvbuf(stdout, NULL, _IONBF, 0);

	for(uint8_t i = 0; i < 8; i++) {
		ballPhase[i] = DEGTORAD(45.0 * i);
		if(i % 2 == 1) {
			motorPhase[i / 2] = ballPhase[i];
		}
	}

	for(uint8_t i = 0; i <= 0x7F; i++) {
		if(i2c1.addressMatch(i) == true) {
			printf("Find %x\n", i);
			cnt++;
		}
	}
	if(cnt == 0) {
		printf("not found\n");
	}

	BNO_Init();
	ADC1_DMA_Start();
	Motor_Start();

	while(1)
	{
		float yaw = BNO_GetYaw();

		printf("yaw:%f\n", yaw);
		delay_ms(100);
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
	TIM_Init(TIM12, 500, 255);

	TIM_PWM_Init(TIM2, TIM2_CH1, 1);
	TIM_PWM_Init(TIM2, TIM2_CH2, 2);
	TIM_PWM_Init(TIM2, TIM2_CH3, 3);
	TIM_PWM_Init(TIM2, TIM2_CH4, 4);
	TIM_PWM_Init(TIM3, TIM3_CH1, 1);
	TIM_PWM_Init(TIM3, TIM3_CH2, 2);
	TIM_PWM_Init(TIM3, TIM3_CH3, 3);
	TIM_PWM_Init(TIM3, TIM3_CH4, 4);
	TIM_PWM_Init(TIM12, PB15, 2);
}

void GPIO_Init(void)
{
	pinMode(LED1, OUTPUT);	pinWrite(LED1, LOW);
	pinMode(LED2, OUTPUT);	pinWrite(LED2, LOW);
	pinMode(SW1, INPUT);
	pinMode(SW2, INPUT);
	pinMode(SW3, INPUT);
	pinMode(BALL_CATCH, INPUT);
	pinMode(KICK_CHARGE, OUTPUT);
	pinMode(KICK_PUSH, OUTPUT);

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

int16_t BNO_Marge(uint8_t low, uint8_t high)
{
	uint16_t data = (high << 8) | low;

	if(data > 32767) {
		return data - 65536;
	}
	return data;
}

void BNO_Write(uint8_t reg, uint8_t val, uint32_t delay)
{
	SysError_t err;
	err = i2c1.memWrite(BNO_ADDRESS, reg, false, &val, 1, 1000);
	delay_ms(delay);
}

void BNO_Init(void)
{
	uint8_t bnoCheck = 0x00;
	uint8_t is_bno = 0x00;

	i2c1.memRead(BNO_ADDRESS, bnoCheck, false, &is_bno, 1, 1000);

	if(is_bno == 0xA0) {
		BNO_Write(0x3D, 0x00, 80);
		BNO_Write(0x3F, 0x20, 1000);
		BNO_Write(0x3E, 0x00, 80);
		BNO_Write(0x3F, 0x80, 1000);
		BNO_Write(0x3D, 0x0C, 80);

		printf("BNO055 found.\n");
	} else {
		while(1) {
			printf("BNO055 not found.\n");
			delay_ms(1000);
		}
	}
}

float BNO_GetYaw(void)
{
	static float preYaw = 0.0;
	float bnoyaw;
	uint8_t dataBuf[2] = {0};

	if(i2c1.memRead(BNO_ADDRESS, BNO_EULER, false, dataBuf, 2, 1000) == SYS_OK) {
		bnoyaw = (float)(BNO_Marge(dataBuf[0], dataBuf[1])) / 16.0;

		if(bnoyaw >= 180.0) {
			bnoyaw -= 360;
		}
		preYaw = bnoyaw;

		return bnoyaw;
	}

	printf("timeout %lx\n", I2C1->ISR);

	return preYaw;
}

void Motor_Start(void)
{
	TIM_Start(TIM2);
	TIM_Start(TIM3);
	TIM_Start(TIM12);
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
	float __angle, valabs, maxabs, gain;

	if(angle >= 180.0) {
		angle -= 360;
	}

	__angle = DEGTORAD(angle);
	maxabs = 0.0;
	for(uint8_t i = 0; i < 4; i++) {
		pspeed[i] = arm_sin_f32(__angle - motorPhase[i]) * speed;
		valabs = fabsf(pspeed[i]);
		if(maxabs <= valabs) {
			maxabs = valabs;
		}
	}

	gain = (float)speed / maxabs;
	for(uint8_t i = 0; i < 4; i++) {
		pspeed[i] *= gain;
	}
}

void Motor_Control(float* spin, float* straight, uint16_t max)
{
	int16_t mspeed[4];
	uint16_t valabs, maxabs;
	float gain;

	if(max > 1023) {
		TIM2->CCR1 = TIM2->CCR2 = TIM2->CCR3 = TIM2->CCR4 = 1023;
		TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = 1023;
	} else {
		maxabs = 0;
		for(uint8_t i = 0; i < 4; i++) {
			mspeed[i] = (int16_t)(spin[i] + straight[i]);
			valabs = (mspeed[i] < 0) ? mspeed[i] * -1 : mspeed[i];
			if(maxabs <= valabs) {
				maxabs = valabs;
			}
		}

		gain = (float)max / (float)maxabs;
		for(uint8_t i = 0; i < 4; i++) {
			mspeed[i] *= gain;
		}

		if(mspeed[0] >= 0 && mspeed[0] <= 1023) {
			TIM3->CCR3 = 0;
			TIM3->CCR4 = mspeed[0];
		} else if(mspeed[0] <= 0 && mspeed[0] >= -1023) {
			TIM3->CCR3 = -mspeed[0];
			TIM3->CCR4 = 0;
		} else {
			TIM3->CCR3 = TIM3->CCR4 = 0;
		}
		if(mspeed[1] >= 0 && mspeed[1] <= 1023) {
			TIM3->CCR1 = mspeed[1];
			TIM3->CCR2 = 0;
		} else if(mspeed[1] <= 0 && mspeed[1] >= -1023) {
			TIM3->CCR1 = 0;
			TIM3->CCR2 = -mspeed[1];
		} else {
			TIM3->CCR1 = TIM3->CCR2 = 0;
		}
		if(mspeed[2] >= 0 && mspeed[2] <= 1023) {
			TIM2->CCR3 = mspeed[2];
			TIM2->CCR4 = 0;
		} else if(mspeed[2] <= 0 && mspeed[2] >= -1023) {
			TIM2->CCR3 = 0;
			TIM2->CCR4 = -mspeed[2];
		} else {
			TIM2->CCR3 = TIM2->CCR4 = 0;
		}
		if(mspeed[3] >= 0 && mspeed[3] <= 1023) {
			TIM2->CCR1 = 0;
			TIM2->CCR2 = mspeed[3];
		} else if(mspeed[3] <= 0 && mspeed[3] >= -1023) {
			TIM2->CCR1 = -mspeed[3];
			TIM2->CCR2 = 0;
		} else {
			TIM2->CCR1 = TIM2->CCR2 = 0;
		}
	}
}

void Ball_Kick(void)
{
	pinWrite(KICK_CHARGE, LOW);	delay_ms(5);
	pinWrite(KICK_PUSH, HIGH);	delay_ms(50);
	pinWrite(KICK_PUSH, LOW);	delay_ms(5);
	pinWrite(KICK_CHARGE, HIGH);
}

void Ball_Angle(BallLineInfo_t* pData)
{
	float x = 0.0;
	float y = 0.0;
	uint16_t total, val;

	total = 0;
	for(uint8_t i = 0; i < 8; i++) {
		val = 4095 - ballVal[can_read_buf][i];
		total += val;
		x += arm_cos_f32(ballPhase[i]) * val;
		y += arm_sin_f32(ballPhase[i]) * val;
	}

	if(total <= 850) {
		pData->valid = false;
	} else {
//		printf("%f %f ", x, y);
		pData->radius = sqrtf((x * x) + (y * y));
		pData->angle = RADTODEG(atan2f(y, x));
		pData->valid = true;
	}
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

void Line_AngleRequest(BallLineInfo_t* pData, uint32_t timeout)
{
	CANTxHeader_t reqdata = {CANID_LINEANG_T, STID, REMOTE_FRAME, 8};
	CANRxHeader_t rx0header;
	double angle;
	uint64_t start = GetTick() / 1000;

	can1_rx0_state = false;
	Can1.transmit(&reqdata, 100);
	Can1.receiveIT(&rx0header, FIFO0);

	while(can1_rx0_state == false) {
		if((GetTick() / 1000 - start) >= timeout) {
			pData->valid = false;
			return;
		}
	}

	if(rx0header.id == CANID_LINEANG_R) {
		memcpy(&angle, rx0header.data, sizeof(double));

		pData->angle = (float)angle;
		pData->valid = true;
	} else {
		pData->valid = false;
	}
}

void CAN1_FIFO0ReceivedCallback(void)
{
	can1_rx0_state = true;
}

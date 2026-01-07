#include <user_main.h>

#define CAN1_BITRATE	(1000000)

#define BNO_ADDRESS		0x28
#define BNO_EULER		0x1A

#define TIM2_FREQ		(1000)
#define TIM3_FREQ		(1000)
#define TIM2_RES		(1023)
#define TIM3_RES		(1023)

CAN Can1(CAN1, CAN1_RX, CAN1_TX);
bool can1_rx0_state = false;

I2C i2c1(I2C1, PB6, PB7);

float motorPhase[4];

const uint8_t ball_ch[8] = {15, 4, 14, 5, 13, 10, 12, 11};
extern uint8_t can_read_buf;
uint16_t ballVal[2][8] = {0};
float ballPhase[8];

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
void Motor_Control(float* pspeed);

float Ball_Angle(void);

uint32_t Line_DataRequest(uint32_t timeout);
double Line_AngleRequest(uint32_t timeout);

void LCD_Reset(void);
void LCD_Init(void);
void LCD_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void LCD_FillRed(void);

void CAN1_FIFO0ReceivedCallback(void);

int main(void)
{
	RCC_Init();
	CAN1_Init();
	i2c1.init();
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

	BNO_Init();
	ADC1_DMA_Start();
	Motor_Start();

	while(1)
	{
		float yaw = BNO_GetYaw();

		printf("%f\n", yaw);

		delay_ms(10);
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

	pinMode(LCD_RST, OUTPUT);
	pinMode(FMC_D0, OTHER);		AFSelect(FMC_D0, AF12);
	pinMode(FMC_D1, OTHER);		AFSelect(FMC_D1, AF12);
	pinMode(FMC_D2, OTHER);		AFSelect(FMC_D2, AF12);
	pinMode(FMC_D3, OTHER);		AFSelect(FMC_D3, AF12);
	pinMode(FMC_D4, OTHER);		AFSelect(FMC_D4, AF12);
	pinMode(FMC_D5, OTHER);		AFSelect(FMC_D5, AF12);
	pinMode(FMC_D6, OTHER);		AFSelect(FMC_D6, AF12);
	pinMode(FMC_D7, OTHER);		AFSelect(FMC_D7, AF12);
	pinMode(FMC_D8, OTHER);		AFSelect(FMC_D8, AF12);
	pinMode(FMC_D9, OTHER);		AFSelect(FMC_D9, AF12);
	pinMode(FMC_D10, OTHER);	AFSelect(FMC_D10, AF12);
	pinMode(FMC_D11, OTHER);	AFSelect(FMC_D11, AF12);
	pinMode(FMC_D12, OTHER);	AFSelect(FMC_D12, AF12);
	pinMode(FMC_D13, OTHER);	AFSelect(FMC_D13, AF12);
	pinMode(FMC_D14, OTHER);	AFSelect(FMC_D14, AF12);
	pinMode(FMC_D15, OTHER);	AFSelect(FMC_D15, AF12);
	pinMode(FMC_NOE, OTHER);	AFSelect(FMC_NOE, AF12);
	pinMode(FMC_NWE, OTHER);	AFSelect(FMC_NWE, AF12);
	pinMode(FMC_NE1, OTHER);	AFSelect(FMC_NE1, AF12);
	pinMode(FMC_A16, OTHER);	AFSelect(FMC_A16, AF12);
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
	uint8_t ptemp[2] = {reg, val};

	i2c1.masterTransmit(BNO_ADDRESS, ptemp, 2, 1000);
	delay_ms(delay);
}

void BNO_Init(void)
{
	uint8_t bnoCheck = 0x00;
	uint8_t is_bno = 0x00;

	i2c1.masterTransmit(BNO_ADDRESS, &bnoCheck, 1, 500);
	i2c1.masterReceive(BNO_ADDRESS, &is_bno, 1, 500);
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
	uint8_t reqData = BNO_EULER;
	uint8_t dataBuf[6] = {0};

	i2c1.masterTransmit(BNO_ADDRESS, &reqData, 1, 100);
	i2c1.masterReceive(BNO_ADDRESS, dataBuf, 6, 100);

	return (float)(BNO_Marge(dataBuf[0], dataBuf[1])) / 16.0;
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
	float __angle, valabs, maxabs, gain;

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

void Motor_Control(float* pspeed)
{
	int16_t mspeed[4];

	for(uint8_t i = 0; i < 4; i++) {
		mspeed[i] = (int16_t)pspeed[i];
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

void LCD_Reset(void)
{
	pinWrite(LCD_RST, LOW);
    delay_ms(15);
    pinWrite(LCD_RST, HIGH);
    delay_ms(150);
}

void LCD_Init(void)
{
    /* ハードリセット */
    LCD_Reset();

    /* Sleep Out */
    LCD_WRITECMD(0x11);
    delay_ms(120);

    /* Pixel Format : RGB565 */
    LCD_WRITECMD(0x3A);
    LCD_WRITEDATA(0x55);

    /* Memory Access Control
       0x60 : 横向き / RGB
       必要に応じて変更 */
    LCD_WRITECMD(0x36);
    LCD_WRITEDATA(0x60);

    /* Porch Setting */
    LCD_WRITECMD(0xB2);
    LCD_WRITEDATA(0x0C);
    LCD_WRITEDATA(0x0C);
    LCD_WRITEDATA(0x00);
    LCD_WRITEDATA(0x33);
    LCD_WRITEDATA(0x33);

    /* Gate Control */
    LCD_WRITECMD(0xB7);
    LCD_WRITEDATA(0x35);

    /* VCOM Setting */
    LCD_WRITECMD(0xBB);
    LCD_WRITEDATA(0x1F);

    /* LCM Control */
    LCD_WRITECMD(0xC0);
    LCD_WRITEDATA(0x2C);

    /* VDV and VRH Command Enable */
    LCD_WRITECMD(0xC2);
    LCD_WRITEDATA(0x01);

    /* VRH Set */
    LCD_WRITECMD(0xC3);
    LCD_WRITEDATA(0x0F);

    /* VDV Set */
    LCD_WRITECMD(0xC4);
    LCD_WRITEDATA(0x20);

    /* Frame Rate Control */
    LCD_WRITECMD(0xC6);
    LCD_WRITEDATA(0x0F);

    /* Power Control */
    LCD_WRITECMD(0xD0);
    LCD_WRITEDATA(0xA4);
    LCD_WRITEDATA(0xA1);

    /* Display ON */
    LCD_WRITECMD(0x29);
    delay_ms(20);

    /* GRAM Write Ready */
//    LCD_WRITECMD(0x2C);
}

void LCD_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    /* Column Address Set */
    LCD_WRITECMD(0x2A);
    LCD_WRITEDATA(x0 >> 8);
    LCD_WRITEDATA(x0 & 0xFF);
    LCD_WRITEDATA(x1 >> 8);
    LCD_WRITEDATA(x1 & 0xFF);

    /* Row Address Set */
    LCD_WRITECMD(0x2B);
    LCD_WRITEDATA(y0 >> 8);
    LCD_WRITEDATA(y0 & 0xFF);
    LCD_WRITEDATA(y1 >> 8);
    LCD_WRITEDATA(y1 & 0xFF);

    /* Memory Write */
    LCD_WRITECMD(0x2C);
}

void LCD_FillRed(void)
{
    uint32_t i;

    LCD_WRITECMD(0x2A);
    LCD_WRITEDATA(0);
    LCD_WRITEDATA(0);
    LCD_WRITEDATA(0);
    LCD_WRITEDATA(239);

    LCD_WRITECMD(0x2B);
    LCD_WRITEDATA(0);
    LCD_WRITEDATA(0);
    LCD_WRITEDATA(1);
    LCD_WRITEDATA(63); // 319

    LCD_WRITECMD(0x2C);

    for(i = 0; i < 240UL * 320UL; i++) {
        LCD_WRITEDATA(0xF800); // Red
    }
}

void CAN1_FIFO0ReceivedCallback(void)
{
	can1_rx0_state = true;
}

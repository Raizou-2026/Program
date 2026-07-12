#include <stm32f745_sys.h>

static volatile uint32_t preTick = 0;
static volatile uint32_t tickCarry = 0;

void RCC_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR1 |= 0b11UL << PWR_CR1_VOS_Pos;

	FLASH->ACR &= FLASH_ACR_LATENCY;
	FLASH->ACR |= 8UL << FLASH_ACR_LATENCY_Pos;

	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM |
			          RCC_PLLCFGR_PLLN |
					  RCC_PLLCFGR_PLLP |
					  RCC_PLLCFGR_PLLQ |
					  RCC_PLLCFGR_PLLSRC);
	RCC->PLLCFGR |= 8UL << RCC_PLLCFGR_PLLM_Pos |
			        432UL << RCC_PLLCFGR_PLLN_Pos |
					0b00UL << RCC_PLLCFGR_PLLP_Pos |
					9UL << RCC_PLLCFGR_PLLQ_Pos |
					RCC_PLLCFGR_PLLSRC_HSE;

	RCC->CFGR &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY));
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));

	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);

	if((DWT->LSR & 1UL << 0) != 0) {
		if((DWT->LSR & 1UL << 1) != 0) {
			DWT->LAR = 0xC5ACCE55;
		}
	}
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	preTick = DWT->CYCCNT;

	SysTick->LOAD = (AHBCLK / 1000) - 1;
	SysTick->VAL = 0;
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk |
				     SysTick_CTRL_TICKINT_Msk |
					 SysTick_CTRL_ENABLE_Msk;
}

void IncTick(void)
{
	if(DWT->CYCCNT < preTick) {
		tickCarry++;
	}
	preTick = DWT->CYCCNT;
}

uint64_t GetTick(void)
{
	uint32_t tick;
	uint32_t prev = preTick;
	uint32_t carry = tickCarry;

	tick = DWT->CYCCNT;
	if(tick < prev) {
		carry++;
	}

	return FULLTICK(carry, tick);
}

uint64_t millis(void)
{
	return GetTick() / 216000;
}

uint64_t micros(void)
{
	return GetTick() / 216;
}

void delay_ms(uint64_t ms)
{
	uint64_t start = millis();
	while((millis() - start) < ms);
}

void delay_us(uint64_t us)
{
	uint64_t start = micros();
	while((micros() - start) < us);
}

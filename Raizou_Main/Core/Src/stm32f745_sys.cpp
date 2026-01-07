#include <stm32f745_sys.h>

static uint32_t preTick = 0;
static int32_t tickCarry = 0;

void MPU_Config_FMC_LCD(void)
{
    /* MPU 無効化 */
    MPU->CTRL = 0;

    /* リージョン番号選択（未使用なら 0 でOK） */
    MPU->RNR = 0;

    /* ベースアドレス設定
       - FMC Bank1: 0x6000_0000
       - RBAR の下位5bitは無視される
    */
    MPU->RBAR = 0x60000000;

    /*
     * RASR 構成
     *
     * XN  = 1  : 実行禁止
     * AP  = 3  : フルアクセス
     * TEX = 0
     * C   = 0  : キャッシュ無効
     * B   = 0  : バッファ無効
     * S   = 1  : Shareable
     * SIZE = 23 : 16MB (2^(23+1))
     * ENABLE = 1
     */
    MPU->RASR =
        (1 << MPU_RASR_XN_Pos) |
        (3 << MPU_RASR_AP_Pos) |
        (0 << MPU_RASR_TEX_Pos) |
        (0 << MPU_RASR_C_Pos) |
        (1 << MPU_RASR_B_Pos) |   // ← ここ重要
        (0 << MPU_RASR_S_Pos) |
        (23 << MPU_RASR_SIZE_Pos) |
        (1 << MPU_RASR_ENABLE_Pos);

    /* MPU 有効化
       PRIVDEFENA = 1 → 未定義領域はデフォルトマップ使用
    */
    MPU->CTRL =
        MPU_CTRL_ENABLE_Msk |
        MPU_CTRL_PRIVDEFENA_Msk;

    /* MPU 設定反映のためのバリア */
    __DSB();
    __ISB();
}

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

	SysTick->LOAD = (AHBCLK / 100) - 1;
	SysTick->VAL = 0;
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk |
				     SysTick_CTRL_TICKINT_Msk |
					 SysTick_CTRL_ENABLE_Msk;
}

void delay_ms(uint32_t ms)
{
	int64_t start = FULLTICK(tickCarry, DWT->CYCCNT);
	while((((int64_t)FULLTICK(tickCarry, DWT->CYCCNT) - start) / 216000) < ms);
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
	return FULLTICK(tickCarry, DWT->CYCCNT) / 216;
}

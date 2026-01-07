/*
 * stm32f745_fmc.cpp
 *
 *  Created on: Jan 3, 2026
 *      Author: neoki
 */

#include <stm32f745_fmc.h>

void FMC_LCD_Init(void)
{
    RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;
    __DSB();

    FMC_Bank1->BTCR[0] = 0;
    FMC_Bank1->BTCR[1] = 0;

    FMC_Bank1->BTCR[1] =
        (5 << FMC_BTR1_ADDSET_Pos) |
        (9 << FMC_BTR1_DATAST_Pos);

    FMC_Bank1->BTCR[0] =
        FMC_BCR1_MBKEN |
        FMC_BCR1_WREN  |
        FMC_BCR1_MWID_0 |
        FMC_BCR1_MTYP_0;
}



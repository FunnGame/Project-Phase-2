/**
 ******************************************************************************
 * @file    drv_clock.c
 * @brief   Clock configuration and introspection implementation (STM32F103).
 ******************************************************************************
 */
#include "drv_clock.h"

/* AHB prescaler encodings (RCC_CFGR[HPRE]) -> divide value. */
static const uint16_t s_ahb_div[16] = {
    1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 8, 16, 64, 128, 256, 512
};
/* APB prescaler encodings (RCC_CFGR[PPREx]) -> divide value. */
static const uint8_t s_apb_div[8] = { 1, 1, 1, 1, 2, 4, 8, 16 };

#define CLOCK_LOCK_TIMEOUT  0x00080000UL

DRV_Status DRV_Clock_Init72MHz(uint32_t hse_hz)
{
    if (hse_hz == 0u) {
        /* Stay on HSI; just refresh the cached frequency. */
        SystemCoreClockUpdate();
        return DRV_OK;
    }
    /* The F103 PLL multiplies by 2..16; 72 MHz must be reachable from HSE. */
    if ((72000000u % hse_hz) != 0u) {
        return DRV_UNSUPPORTED;
    }
    const uint32_t mul = 72000000u / hse_hz;
    if (mul < 2u || mul > 16u) {
        return DRV_UNSUPPORTED;
    }

    /* 1. Start HSE and wait for it to stabilise. */
    DRV_SET_BITS(RCC->CR, RCC_CR_HSEON);
    if (DRV_WaitFlag(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY,
                     CLOCK_LOCK_TIMEOUT) != DRV_OK) {
        return DRV_TIMEOUT;
    }

    /* 2. Flash: 2 wait states are required above 48 MHz, plus prefetch. */
    DRV_MODIFY(FLASH->ACR, FLASH_ACR_LATENCY,
               FLASH_ACR_LATENCY_2 | FLASH_ACR_PRFTBE);

    /* 3. Bus prescalers: AHB /1 (72), APB1 /2 (36 max), APB2 /1 (72). */
    DRV_MODIFY(RCC->CFGR,
               RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2,
               RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1);

    /* 4. PLL: source = HSE (no /2 predivider), multiplier = 72MHz / hse. */
    DRV_MODIFY(RCC->CFGR,
               RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL,
               RCC_CFGR_PLLSRC | ((mul - 2u) << RCC_CFGR_PLLMULL_Pos));

    /* 5. Enable the PLL and wait for lock. */
    DRV_SET_BITS(RCC->CR, RCC_CR_PLLON);
    if (DRV_WaitFlag(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY,
                     CLOCK_LOCK_TIMEOUT) != DRV_OK) {
        return DRV_TIMEOUT;
    }

    /* 6. Switch SYSCLK to the PLL and confirm the switch happened. */
    DRV_MODIFY(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
    if (DRV_WaitFlag(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL,
                     CLOCK_LOCK_TIMEOUT) != DRV_OK) {
        return DRV_TIMEOUT;
    }

    SystemCoreClockUpdate();
    return DRV_OK;
}

uint32_t DRV_Clock_GetSysClk(void)
{
    SystemCoreClockUpdate();   /* recomputes from RCC->CFGR */
    return SystemCoreClock;
}

uint32_t DRV_Clock_GetHCLK(void)
{
    const uint32_t hpre = (RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;
    return DRV_Clock_GetSysClk() / s_ahb_div[hpre & 0xFu];
}

uint32_t DRV_Clock_GetPCLK1(void)
{
    const uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    return DRV_Clock_GetHCLK() / s_apb_div[ppre1 & 0x7u];
}

uint32_t DRV_Clock_GetPCLK2(void)
{
    const uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    return DRV_Clock_GetHCLK() / s_apb_div[ppre2 & 0x7u];
}

uint32_t DRV_Clock_GetTimerClock(TIM_TypeDef *tim)
{
    /* TIM1 hangs off APB2; TIM2..TIM4 off APB1. In both cases the timer clock
     * is doubled whenever that bus prescaler is not /1. */
    if (tim == TIM1) {
        const uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
        const uint32_t pclk2 = DRV_Clock_GetPCLK2();
        return (s_apb_div[ppre2 & 0x7u] == 1u) ? pclk2 : pclk2 * 2u;
    }

    const uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    const uint32_t pclk1 = DRV_Clock_GetPCLK1();
    return (s_apb_div[ppre1 & 0x7u] == 1u) ? pclk1 : pclk1 * 2u;
}

DRV_ResetReason DRV_Clock_ResetReason(void)
{
    const uint32_t csr = RCC->CSR;
    DRV_ResetReason r;

    /* Most specific first. PINRSTF is set alongside a power-on reset on this
     * part, so it has to be tested LAST or everything looks like a pin reset. */
    if (csr & RCC_CSR_LPWRRSTF) {
        r = DRV_RESET_LOW_POWER;
    } else if (csr & RCC_CSR_WWDGRSTF) {
        r = DRV_RESET_WWDG;
    } else if (csr & RCC_CSR_IWDGRSTF) {
        r = DRV_RESET_IWDG;
    } else if (csr & RCC_CSR_SFTRSTF) {
        r = DRV_RESET_SOFTWARE;
    } else if (csr & RCC_CSR_PORRSTF) {
        r = DRV_RESET_POWER_ON;      /* or a brown-out - see the header */
    } else if (csr & RCC_CSR_PINRSTF) {
        r = DRV_RESET_PIN;
    } else {
        r = DRV_RESET_UNKNOWN;
    }

    /* Clear, so the next reset reports its own cause and not this one too. */
    RCC->CSR |= RCC_CSR_RMVF;

    return r;
}

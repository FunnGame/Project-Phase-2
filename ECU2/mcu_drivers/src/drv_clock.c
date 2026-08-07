#include "drv_clock.h"

static const uint16_t s_ahb_div[16] = {
    1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 8, 16, 64, 128, 256, 512
};

static const uint8_t s_apb_div[8] = { 1, 1, 1, 1, 2, 4, 8, 16 };

#define CLOCK_LOCK_TIMEOUT  0x00080000UL

DRV_Status DRV_Clock_Init72MHz(uint32_t hse_hz)
{
    if (hse_hz == 0u) {

        SystemCoreClockUpdate();
        return DRV_OK;
    }

    if ((72000000u % hse_hz) != 0u) {
        return DRV_UNSUPPORTED;
    }

    const uint32_t mul = 72000000u / hse_hz;

    if (mul < 2u || mul > 16u) {
        return DRV_UNSUPPORTED;
    }

    DRV_SET_BITS(RCC->CR, RCC_CR_HSEON);

    if (DRV_WaitFlag(&RCC->CR,
                     RCC_CR_HSERDY,
                     RCC_CR_HSERDY,
                     CLOCK_LOCK_TIMEOUT) != DRV_OK) {
        return DRV_TIMEOUT;
    }

    DRV_MODIFY(FLASH->ACR,
               FLASH_ACR_LATENCY,
               FLASH_ACR_LATENCY_2 | FLASH_ACR_PRFTBE);

    DRV_MODIFY(RCC->CFGR,
               RCC_CFGR_HPRE |
               RCC_CFGR_PPRE1 |
               RCC_CFGR_PPRE2,
               RCC_CFGR_HPRE_DIV1 |
               RCC_CFGR_PPRE1_DIV2 |
               RCC_CFGR_PPRE2_DIV1);

    DRV_MODIFY(RCC->CFGR,
               RCC_CFGR_PLLSRC |
               RCC_CFGR_PLLXTPRE |
               RCC_CFGR_PLLMULL,
               RCC_CFGR_PLLSRC |
               ((mul - 2u) << 18));          // S?A

    DRV_SET_BITS(RCC->CR, RCC_CR_PLLON);

    if (DRV_WaitFlag(&RCC->CR,
                     RCC_CR_PLLRDY,
                     RCC_CR_PLLRDY,
                     CLOCK_LOCK_TIMEOUT) != DRV_OK) {
        return DRV_TIMEOUT;
    }

    DRV_MODIFY(RCC->CFGR,
               RCC_CFGR_SW,
               RCC_CFGR_SW_PLL);

    if (DRV_WaitFlag(&RCC->CFGR,
                     RCC_CFGR_SWS,
                     RCC_CFGR_SWS_PLL,
                     CLOCK_LOCK_TIMEOUT) != DRV_OK) {
        return DRV_TIMEOUT;
    }

    SystemCoreClockUpdate();

    return DRV_OK;
}

uint32_t DRV_Clock_GetSysClk(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

uint32_t DRV_Clock_GetHCLK(void)
{
    const uint32_t hpre =
        (RCC->CFGR & RCC_CFGR_HPRE) >> 4;    // S?A

    return DRV_Clock_GetSysClk() /
           s_ahb_div[hpre & 0xFu];
}

uint32_t DRV_Clock_GetPCLK1(void)
{
    const uint32_t ppre1 =
        (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;   // S?A

    return DRV_Clock_GetHCLK() /
           s_apb_div[ppre1 & 0x7u];
}

uint32_t DRV_Clock_GetPCLK2(void)
{
    const uint32_t ppre2 =
        (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;  // S?A

    return DRV_Clock_GetHCLK() /
           s_apb_div[ppre2 & 0x7u];
}

uint32_t DRV_Clock_GetTimerClock(TIM_TypeDef *tim)
{
    if (tim == TIM1)
    {
        const uint32_t ppre2 =
            (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;  // S?A

        const uint32_t pclk2 = DRV_Clock_GetPCLK2();

        return (s_apb_div[ppre2 & 0x7u] == 1u)
                ? pclk2
                : (pclk2 * 2u);
    }

    const uint32_t ppre1 =
        (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;   // S?A

    const uint32_t pclk1 = DRV_Clock_GetPCLK1();

    return (s_apb_div[ppre1 & 0x7u] == 1u)
            ? pclk1
            : (pclk1 * 2u);
}
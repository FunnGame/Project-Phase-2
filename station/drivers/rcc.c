/**
 ******************************************************************************
 * @file    rcc.c
 * @brief   Reset & Clock Control driver implementation (STM32F446RE).
 ******************************************************************************
 */
#include "rcc.h"

/* AHB prescaler encodings for RCC_CFGR[HPRE] and their divide values. */
static const uint16_t s_ahb_div_table[16] = {
    1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 8, 16, 64, 128, 256, 512
};
/* APB prescaler encodings for RCC_CFGR[PPREx] and their divide values. */
static const uint8_t s_apb_div_table[8] = { 1, 1, 1, 1, 2, 4, 8, 16 };

/* -------------------------------------------------------------------------- */
/*  Peripheral clock gating                                                   */
/* -------------------------------------------------------------------------- */

void rcc_gpio_clock_enable(GPIO_TypeDef *port)
{
    if      (port == GPIOA) rcc_ahb1_enable(RCC_AHB1ENR_GPIOAEN);
    else if (port == GPIOB) rcc_ahb1_enable(RCC_AHB1ENR_GPIOBEN);
    else if (port == GPIOC) rcc_ahb1_enable(RCC_AHB1ENR_GPIOCEN);
    else if (port == GPIOD) rcc_ahb1_enable(RCC_AHB1ENR_GPIODEN);
    else if (port == GPIOE) rcc_ahb1_enable(RCC_AHB1ENR_GPIOEEN);
    else if (port == GPIOF) rcc_ahb1_enable(RCC_AHB1ENR_GPIOFEN);
    else if (port == GPIOG) rcc_ahb1_enable(RCC_AHB1ENR_GPIOGEN);
    else if (port == GPIOH) rcc_ahb1_enable(RCC_AHB1ENR_GPIOHEN);
}

/* -------------------------------------------------------------------------- */
/*  System clock configuration                                                */
/* -------------------------------------------------------------------------- */

/* Conservative iteration budget for oscillator/PLL lock polling. */
#define RCC_LOCK_TIMEOUT   0x00080000UL

static bool rcc_cfg_is_valid(const rcc_sysclk_config_t *cfg)
{
    if (cfg == NULL) return false;
    if (cfg->pll_m < 2U || cfg->pll_m > 63U) return false;
    if (cfg->pll_n < 50U || cfg->pll_n > 432U) return false;
    if (cfg->pll_p != 2U && cfg->pll_p != 4U &&
        cfg->pll_p != 6U && cfg->pll_p != 8U) return false;
    if (cfg->src == RCC_SYSCLK_SRC_HSE && cfg->hse_hz == 0U) return false;
    return true;
}

drv_status_t rcc_sysclk_config(const rcc_sysclk_config_t *cfg)
{
    if (!rcc_cfg_is_valid(cfg)) {
        return DRV_INVALID_PARAM;
    }

    /* 1. Enable the selected PLL source and wait for it to stabilise. */
    if (cfg->src == RCC_SYSCLK_SRC_HSE) {
        DRV_SET_BITS(RCC->CR, RCC_CR_HSEON);
        if (drv_wait_flag(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY,
                          RCC_LOCK_TIMEOUT) != DRV_OK) {
            return DRV_TIMEOUT;
        }
    } else {
        DRV_SET_BITS(RCC->CR, RCC_CR_HSION);
        if (drv_wait_flag(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY,
                          RCC_LOCK_TIMEOUT) != DRV_OK) {
            return DRV_TIMEOUT;
        }
    }

    /* 2. Configure flash latency BEFORE raising the clock. */
    DRV_MODIFY(FLASH->ACR, FLASH_ACR_LATENCY,
               (cfg->flash_ws & FLASH_ACR_LATENCY) |
               FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN);

    /* 3. Program bus prescalers (HCLK, PCLK1, PCLK2). */
    uint32_t hpre = 0U; /* default /1 */
    for (uint32_t i = 8U; i < 16U; ++i) {
        if (s_ahb_div_table[i] == cfg->ahb_div) { hpre = i; break; }
    }
    DRV_MODIFY(RCC->CFGR,
               RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2,
               (hpre << RCC_CFGR_HPRE_Pos) |
               ((uint32_t)cfg->apb1_div << (RCC_CFGR_PPRE1_Pos + 0U)) |
               ((uint32_t)cfg->apb2_div << (RCC_CFGR_PPRE2_Pos + 0U)));

    /* 4. Configure the main PLL (M, N, P and the input source). */
    uint32_t pll_p_bits = (cfg->pll_p / 2U) - 1U; /* 2->0, 4->1, 6->2, 8->3 */
    uint32_t pll_src = (cfg->src == RCC_SYSCLK_SRC_HSE) ? RCC_PLLCFGR_PLLSRC_HSE
                                                        : 0U;
    RCC->PLLCFGR =
        (cfg->pll_m << RCC_PLLCFGR_PLLM_Pos) |
        (cfg->pll_n << RCC_PLLCFGR_PLLN_Pos) |
        (pll_p_bits << RCC_PLLCFGR_PLLP_Pos) |
        pll_src |
        (RCC->PLLCFGR & RCC_PLLCFGR_PLLQ); /* keep existing Q (USB/SDIO) */

    /* 5. Enable the PLL and wait for lock. */
    DRV_SET_BITS(RCC->CR, RCC_CR_PLLON);
    if (drv_wait_flag(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY,
                      RCC_LOCK_TIMEOUT) != DRV_OK) {
        return DRV_TIMEOUT;
    }

    /* 6. Switch SYSCLK to the PLL and confirm the switch took effect. */
    DRV_MODIFY(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
    if (drv_wait_flag(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL,
                      RCC_LOCK_TIMEOUT) != DRV_OK) {
        return DRV_TIMEOUT;
    }

    (void)rcc_get_sysclk_hz(); /* refresh SystemCoreClock */
    return DRV_OK;
}

drv_status_t rcc_sysclk_config_default(uint32_t hse_hz)
{
    rcc_sysclk_config_t cfg = {
        .src      = (hse_hz != 0U) ? RCC_SYSCLK_SRC_HSE : RCC_SYSCLK_SRC_HSI,
        .hse_hz   = hse_hz,
        /* Normalise VCO input to 1 MHz so N maps directly to VCO out in MHz. */
        .pll_m    = (hse_hz != 0U) ? (hse_hz / 1000000U) : 16U,
        .pll_n    = 360U,   /* VCO = 360 MHz */
        .pll_p    = 2U,     /* SYSCLK = 180 MHz */
        .ahb_div  = 1U,     /* HCLK  = 180 MHz */
        .apb1_div = RCC_APB_DIV4, /* PCLK1 = 45 MHz (max) */
        .apb2_div = RCC_APB_DIV2, /* PCLK2 = 90 MHz (max) */
        .flash_ws = 5U,     /* 180 MHz @ 2.7-3.6V => 5 wait states */
    };
    return rcc_sysclk_config(&cfg);
}

/* -------------------------------------------------------------------------- */
/*  Clock introspection                                                       */
/* -------------------------------------------------------------------------- */

uint32_t rcc_get_sysclk_hz(void)
{
    SystemCoreClockUpdate();  /* provided by system_stm32f4xx.c */
    return SystemCoreClock;
}

uint32_t rcc_get_hclk_hz(void)
{
    uint32_t hpre = (RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;
    return rcc_get_sysclk_hz() / s_ahb_div_table[hpre & 0xFU];
}

uint32_t rcc_get_pclk1_hz(void)
{
    uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    return rcc_get_hclk_hz() / s_apb_div_table[ppre1 & 0x7U];
}

uint32_t rcc_get_pclk2_hz(void)
{
    uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    return rcc_get_hclk_hz() / s_apb_div_table[ppre2 & 0x7U];
}

uint32_t rcc_get_timer_pclk1_hz(void)
{
    uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    /* APB timer clock doubles whenever the APB prescaler is > 1. */
    return (s_apb_div_table[ppre1 & 0x7U] == 1U) ? rcc_get_pclk1_hz()
                                                 : rcc_get_pclk1_hz() * 2U;
}

uint32_t rcc_get_timer_pclk2_hz(void)
{
    uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    return (s_apb_div_table[ppre2 & 0x7U] == 1U) ? rcc_get_pclk2_hz()
                                                 : rcc_get_pclk2_hz() * 2U;
}

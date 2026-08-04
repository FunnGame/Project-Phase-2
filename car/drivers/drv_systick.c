/**
 ******************************************************************************
 * @file    drv_systick.c
 * @brief   SysTick millisecond time base implementation (STM32F103).
 ******************************************************************************
 */
#include "drv_systick.h"
#include "drv_clock.h"

static volatile uint32_t s_tick_ms = 0u;
static DRV_SysTick_Callback s_cb = NULL;
static void *s_cb_ctx = NULL;

DRV_Status DRV_SysTick_Init(uint32_t priority)
{
    /* Ask the clock driver rather than assuming 72 MHz: without an explicit
     * PLL bring-up the F1 runs at 8 MHz HSI, and a hardcoded reload would make
     * every timeout in the firmware 9x too long. */
    const uint32_t hclk = DRV_Clock_GetHCLK();
    const uint32_t reload = (hclk / 1000u) - 1u;

    if (reload > SysTick_LOAD_RELOAD_Msk) {
        return DRV_UNSUPPORTED;
    }

    s_tick_ms = 0u;

    SysTick->LOAD = reload;
    SysTick->VAL  = 0u;                       /* clear the current count */
    NVIC_SetPriority(SysTick_IRQn, priority);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |   /* processor clock (HCLK) */
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;

    return DRV_OK;
}

uint32_t DRV_SysTick_GetTick(void)
{
    return s_tick_ms;
}

bool DRV_SysTick_Elapsed(uint32_t start_ms, uint32_t timeout_ms)
{
    return (uint32_t)(s_tick_ms - start_ms) >= timeout_ms;
}

void DRV_SysTick_DelayMs(uint32_t ms)
{
    const uint32_t start = s_tick_ms;
    /* +1 because the first tick may land immediately after entry, which would
     * otherwise cut the delay short by up to a millisecond. */
    while ((uint32_t)(s_tick_ms - start) < (ms + 1u)) {
        __NOP();
    }
}

void DRV_SysTick_SetCallback(DRV_SysTick_Callback cb, void *ctx)
{
    s_cb     = cb;
    s_cb_ctx = ctx;
}

void DRV_SysTick_OnTick(void)
{
    s_tick_ms++;
    if (s_cb != NULL) {
        s_cb(s_tick_ms, s_cb_ctx);
    }
}

#if DRV_SYSTICK_PROVIDE_IRQ_HANDLER
void SysTick_Handler(void)
{
    DRV_SysTick_OnTick();
}
#endif

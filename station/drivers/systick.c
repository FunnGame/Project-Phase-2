/**
 ******************************************************************************
 * @file    systick.c
 * @brief   SysTick time base + delay implementation (STM32F446RE).
 ******************************************************************************
 */
#include "systick.h"
#include "rcc.h"

static volatile uint32_t s_ms_ticks = 0U;
static systick_callback_t s_cb = NULL;
static void *s_cb_ctx = NULL;
static uint32_t s_core_hz = 0U;

/* Enable the DWT cycle counter (used for microsecond delays). */
static void systick_dwt_enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

drv_status_t systick_init(uint32_t core_clk_hz)
{
    s_core_hz = (core_clk_hz != 0U) ? core_clk_hz : rcc_get_hclk_hz();

    /* 1 ms reload; SysTick runs from the processor clock (HCLK). */
    const uint32_t reload = (s_core_hz / 1000U) - 1U;
    if (reload > SysTick_LOAD_RELOAD_Msk) {
        return DRV_UNSUPPORTED;
    }

    s_ms_ticks    = 0U;
    SysTick->LOAD = reload;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;

    systick_dwt_enable();
    return DRV_OK;
}

uint32_t systick_get_ms(void)
{
    return s_ms_ticks;
}

void systick_delay_ms(uint32_t ms)
{
    const uint32_t start = s_ms_ticks;
    /* +1 so we always wait at least the requested whole milliseconds. */
    while ((s_ms_ticks - start) < (ms + 1U)) {
        __NOP();
    }
}

void systick_delay_us(uint32_t us)
{
    const uint32_t start  = DWT->CYCCNT;
    const uint32_t cycles = (s_core_hz / 1000000U) * us;
    while ((DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}

bool systick_elapsed(uint32_t start_ms, uint32_t timeout_ms)
{
    return (uint32_t)(s_ms_ticks - start_ms) >= timeout_ms;
}

void systick_set_callback(systick_callback_t cb, void *ctx)
{
    s_cb     = cb;
    s_cb_ctx = ctx;
}

void systick_on_tick(void)
{
    s_ms_ticks++;
    if (s_cb != NULL) {
        s_cb(s_ms_ticks, s_cb_ctx);
    }
}

#if SYSTICK_PROVIDE_IRQ_HANDLER
void SysTick_Handler(void)
{
    systick_on_tick();
}
#endif

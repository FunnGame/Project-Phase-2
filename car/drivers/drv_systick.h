/**
 ******************************************************************************
 * @file    drv_systick.h
 * @brief   Millisecond time base on the Cortex-M3 SysTick (STM32F103).
 *
 * The system tick belongs here rather than on a general-purpose timer: the
 * F103C8 has only four timers (TIM1..TIM4) and the car needs all of them —
 * two for the wheel encoders, one for the motor PWM, one for the encoder
 * sampler. SysTick is a core peripheral and costs none of that budget.
 *
 * Microsecond delays are NOT provided here; they live in drv_timer.h
 * (DRV_Delay_Us), which uses the DWT cycle counter and is independent of this
 * tick.
 *
 * ISR wiring: this driver defines SysTick_Handler by default. Compile with
 * -DDRV_SYSTICK_PROVIDE_IRQ_HANDLER=0 to supply your own and call
 * DRV_SysTick_OnTick() from it.
 ******************************************************************************
 */
#ifndef DRV_SYSTICK_H_
#define DRV_SYSTICK_H_

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DRV_SYSTICK_PROVIDE_IRQ_HANDLER
#define DRV_SYSTICK_PROVIDE_IRQ_HANDLER 1
#endif

/** @brief Periodic hook, invoked from the SysTick ISR every millisecond. */
typedef void (*DRV_SysTick_Callback)(uint32_t tick_ms, void *ctx);

/**
 * @brief Start the 1 kHz (1 ms) time base.
 * @param priority NVIC priority for the SysTick exception (0 = highest).
 * @return DRV_OK, or DRV_UNSUPPORTED if the reload value does not fit the
 *         24-bit SysTick counter (HCLK above ~16.7 MHz per millisecond).
 *
 * Reads the real HCLK from drv_clock, so the tick is 1 ms whether the part is
 * running at 8 MHz HSI or 72 MHz PLL. Call after the clock is configured.
 */
DRV_Status DRV_SysTick_Init(uint32_t priority);

/** @brief Milliseconds since DRV_SysTick_Init(). Wraps after ~49 days. */
uint32_t DRV_SysTick_GetTick(void);

/** @brief True once @p timeout_ms has elapsed since @p start_ms (wrap-safe). */
bool DRV_SysTick_Elapsed(uint32_t start_ms, uint32_t timeout_ms);

/** @brief Block for @p ms milliseconds. Requires DRV_SysTick_Init(). */
void DRV_SysTick_DelayMs(uint32_t ms);

/**
 * @brief Register a hook called from every tick.
 * @param cb  Callback, or NULL to remove.
 * @param ctx Opaque pointer handed back to @p cb.
 *
 * Runs in interrupt context at 1 kHz — keep it short.
 */
void DRV_SysTick_SetCallback(DRV_SysTick_Callback cb, void *ctx);

/** @brief Advance the tick and run the hook; call from a custom ISR. */
void DRV_SysTick_OnTick(void);

#ifdef __cplusplus
}
#endif

#endif /* DRV_SYSTICK_H_ */

/**
 ******************************************************************************
 * @file    systick.h
 * @brief   SysTick millisecond time base + delay helpers (STM32F446RE).
 *
 * Provides a monotonic millisecond counter, blocking millisecond/microsecond
 * delays and an optional periodic user hook (handy as a lightweight scheduler
 * tick). Microsecond delays use the Cortex-M4 DWT cycle counter for accuracy
 * independent of the tick rate.
 *
 * ISR wiring: this driver defines SysTick_Handler by default. Compile with
 * -DSYSTICK_PROVIDE_IRQ_HANDLER=0 to supply your own and call systick_on_tick()
 * from it.
 ******************************************************************************
 */
#ifndef STATION_DRIVERS_SYSTICK_H
#define STATION_DRIVERS_SYSTICK_H

#include "drivers_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SYSTICK_PROVIDE_IRQ_HANDLER
#define SYSTICK_PROVIDE_IRQ_HANDLER 1
#endif

/** @brief Periodic tick callback, invoked from the SysTick ISR every 1 ms. */
typedef void (*systick_callback_t)(uint32_t tick_ms, void *ctx);

/**
 * @brief Start the 1 kHz (1 ms) SysTick time base.
 * @param core_clk_hz Core clock in Hz. Pass 0 to auto-detect via the rcc
 *                    driver / SystemCoreClock.
 * @return DRV_OK, or DRV_UNSUPPORTED if the reload value does not fit 24 bits.
 *
 * Also enables the DWT cycle counter used by systick_delay_us().
 */
drv_status_t systick_init(uint32_t core_clk_hz);

/** @brief Milliseconds elapsed since systick_init(). Wraps after ~49 days. */
uint32_t systick_get_ms(void);

/** @brief Block for @p ms milliseconds. */
void systick_delay_ms(uint32_t ms);

/** @brief Block for @p us microseconds (DWT-based, +/- a few cycles). */
void systick_delay_us(uint32_t us);

/**
 * @brief True once @p timeout_ms has elapsed since @p start_ms.
 * @param start_ms   A value previously returned by systick_get_ms().
 * @param timeout_ms Timeout window in milliseconds.
 *
 * Wrap-around safe. Use for non-blocking timeouts:
 *   uint32_t t0 = systick_get_ms();
 *   while (!systick_elapsed(t0, 100)) { ... }
 */
bool systick_elapsed(uint32_t start_ms, uint32_t timeout_ms);

/**
 * @brief Register a callback invoked from every SysTick interrupt.
 * @param cb  Callback, or NULL to remove.
 * @param ctx Opaque pointer forwarded to @p cb.
 */
void systick_set_callback(systick_callback_t cb, void *ctx);

/** @brief Advance the tick and run the callback; call from a custom ISR. */
void systick_on_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* STATION_DRIVERS_SYSTICK_H */

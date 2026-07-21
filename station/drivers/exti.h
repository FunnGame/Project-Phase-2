/**
 ******************************************************************************
 * @file    exti.h
 * @brief   External interrupt / event (EXTI + NVIC) driver for the F446RE.
 *
 * Wraps the SYSCFG line multiplexer, the EXTI edge-detection block and the
 * NVIC so higher layers can attach a C callback to any GPIO edge with one
 * call. Also exposes thin NVIC priority/enable helpers for use by other
 * peripherals.
 *
 * On the control station this is what services, e.g., the nRF24 IRQ line and
 * user push-buttons.
 *
 * ISR wiring
 * ----------
 * By default (EXTI_PROVIDE_IRQ_HANDLERS == 1) this driver defines the six EXTI
 * vector handlers (EXTI0..4, EXTI9_5, EXTI15_10). If your project already
 * defines them elsewhere, compile this file with -DEXTI_PROVIDE_IRQ_HANDLERS=0
 * and call exti_irq_dispatch() from your own handlers.
 ******************************************************************************
 */
#ifndef STATION_DRIVERS_EXTI_H
#define STATION_DRIVERS_EXTI_H

#include "drivers_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef EXTI_PROVIDE_IRQ_HANDLERS
#define EXTI_PROVIDE_IRQ_HANDLERS 1
#endif

/** @brief Edge(s) on which the interrupt/event is generated. */
typedef enum {
    EXTI_TRIGGER_RISING = 0,
    EXTI_TRIGGER_FALLING,
    EXTI_TRIGGER_BOTH,
} exti_trigger_t;

/**
 * @brief Callback invoked from interrupt context when a line fires.
 * @param line The EXTI line number (0..15) that triggered.
 * @param ctx  Opaque user pointer supplied at registration time.
 */
typedef void (*exti_callback_t)(uint8_t line, void *ctx);

/**
 * @brief Per-line configuration.
 *
 * @p port selects which GPIO bank drives the line via SYSCFG->EXTICR
 * (line N is always pin N of the chosen port). @p priority is the NVIC
 * preemption priority (lower value = higher urgency).
 */
typedef struct {
    GPIO_TypeDef   *port;      /**< Source GPIO bank (GPIOA..GPIOH).        */
    exti_trigger_t  trigger;   /**< Active edge(s).                         */
    exti_callback_t callback;  /**< Handler, or NULL for event-only.        */
    void           *ctx;       /**< User context passed to @p callback.     */
    uint32_t        priority;  /**< NVIC priority (0 = highest).            */
    bool            enable_nvic;/**< Enable the NVIC IRQ immediately.       */
} exti_line_config_t;

/* -------------------------------------------------------------------------- */
/*  Line configuration                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configure GPIO EXTI @p line (0..15) from @p cfg.
 * @param line EXTI line number, equal to the GPIO pin number.
 * @param cfg  Configuration (must not be NULL).
 * @return DRV_OK or DRV_INVALID_PARAM.
 *
 * The corresponding GPIO pin must already be configured as an input (e.g. via
 * gpio_init_input()). SYSCFG's clock is enabled automatically.
 */
drv_status_t exti_config_line(uint8_t line, const exti_line_config_t *cfg);

/** @brief Unmask (enable) the interrupt request for @p line. */
void exti_enable_line(uint8_t line);
/** @brief Mask (disable) the interrupt request for @p line. */
void exti_disable_line(uint8_t line);
/** @brief Return true if @p line has a pending interrupt flag set. */
bool exti_is_pending(uint8_t line);
/** @brief Clear the pending flag for @p line (write-1-to-clear). */
void exti_clear_pending(uint8_t line);
/** @brief Raise @p line by software (for self-test / task signalling). */
void exti_software_trigger(uint8_t line);

/**
 * @brief Dispatch handler for one or more EXTI lines.
 * @param line_first First line in the vector's range.
 * @param line_last  Last line in the vector's range (inclusive).
 *
 * Clears the pending flag and invokes the registered callback for every
 * asserted line in [line_first, line_last]. Call this from a custom ISR when
 * EXTI_PROVIDE_IRQ_HANDLERS is 0.
 */
void exti_irq_dispatch(uint8_t line_first, uint8_t line_last);

/* Generic NVIC helpers (nvic_enable / nvic_disable) live in drivers_common.h. */

#ifdef __cplusplus
}
#endif

#endif /* STATION_DRIVERS_EXTI_H */

/**
 ******************************************************************************
 * @file    gpio.h
 * @brief   General-purpose I/O driver for the STM32F446RE.
 *
 * A register-level, HAL-free GPIO abstraction covering every pin attribute the
 * higher layers need: mode, output type, speed, pull resistors and alternate
 * function selection. Pins are addressed by (port, pin) or by a 16-bit mask so
 * several pins sharing one configuration can be set up in a single call.
 *
 * Typical use on the control station:
 *   - ILI9341 control lines (RESET / DC / CS)  -> push-pull outputs
 *   - SPI1 SCK/MISO/MOSI                        -> alternate function
 *   - User button / interrupt sources           -> inputs with pull-up
 *
 * Enabling the port's bus clock is handled for you by gpio_init().
 ******************************************************************************
 */
#ifndef STATION_DRIVERS_GPIO_H
#define STATION_DRIVERS_GPIO_H

#include "drivers_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/*  Configuration enumerations                                                */
/* -------------------------------------------------------------------------- */

/** @brief Pin direction / function (GPIOx_MODER, 2 bits per pin). */
typedef enum {
    GPIO_MODE_INPUT  = 0x0,  /**< Digital input (default after reset).      */
    GPIO_MODE_OUTPUT = 0x1,  /**< General-purpose output.                   */
    GPIO_MODE_AF     = 0x2,  /**< Alternate function (see gpio_af_t).       */
    GPIO_MODE_ANALOG = 0x3,  /**< Analog (ADC/DAC, lowest leakage).         */
} gpio_mode_t;

/** @brief Output driver stage type (GPIOx_OTYPER, 1 bit per pin). */
typedef enum {
    GPIO_OTYPE_PP = 0x0,     /**< Push-pull.                                */
    GPIO_OTYPE_OD = 0x1,     /**< Open-drain (needs external/internal PU).  */
} gpio_otype_t;

/** @brief Output slew-rate / speed (GPIOx_OSPEEDR, 2 bits per pin). */
typedef enum {
    GPIO_SPEED_LOW       = 0x0,
    GPIO_SPEED_MEDIUM    = 0x1,
    GPIO_SPEED_HIGH      = 0x2,
    GPIO_SPEED_VERY_HIGH = 0x3,
} gpio_speed_t;

/** @brief Internal pull resistor selection (GPIOx_PUPDR, 2 bits per pin). */
typedef enum {
    GPIO_PULL_NONE = 0x0,
    GPIO_PULL_UP   = 0x1,
    GPIO_PULL_DOWN = 0x2,
} gpio_pull_t;

/** @brief Alternate-function index (GPIOx_AFRL/AFRH, 4 bits per pin). */
typedef enum {
    GPIO_AF0 = 0,  GPIO_AF1,  GPIO_AF2,  GPIO_AF3,
    GPIO_AF4,      GPIO_AF5,  GPIO_AF6,  GPIO_AF7,
    GPIO_AF8,      GPIO_AF9,  GPIO_AF10, GPIO_AF11,
    GPIO_AF12,     GPIO_AF13, GPIO_AF14, GPIO_AF15,
} gpio_af_t;

/** @brief Logical pin level. */
typedef enum {
    GPIO_LOW  = 0,
    GPIO_HIGH = 1,
} gpio_state_t;

/* -------------------------------------------------------------------------- */
/*  Pin-mask helpers                                                          */
/* -------------------------------------------------------------------------- */

/** @brief Build the 16-bit mask for a single pin number (0..15). */
#define GPIO_PIN(n)   ((uint16_t)(1U << (n)))
#define GPIO_PIN_ALL  ((uint16_t)0xFFFFU)

/* -------------------------------------------------------------------------- */
/*  Configuration structure                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Full configuration for one or more pins of a single port.
 *
 * @p pin_mask may combine several pins (e.g. GPIO_PIN(5) | GPIO_PIN(6)); they
 * all receive the same attributes. @p af is only consulted when @p mode is
 * GPIO_MODE_AF; @p otype/@p speed only matter for outputs and AF.
 */
typedef struct {
    uint16_t     pin_mask;   /**< OR of GPIO_PIN(n) bits to configure.      */
    gpio_mode_t  mode;       /**< Direction / function.                     */
    gpio_otype_t otype;      /**< Output stage type.                        */
    gpio_speed_t speed;      /**< Output speed.                             */
    gpio_pull_t  pull;       /**< Pull resistor.                            */
    gpio_af_t    af;         /**< Alternate function (mode == AF only).     */
} gpio_config_t;

/* -------------------------------------------------------------------------- */
/*  Configuration API                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Enable @p port's clock and apply @p cfg to the selected pins.
 * @param port One of GPIOA..GPIOH.
 * @param cfg  Configuration to apply (must not be NULL).
 * @return DRV_OK or DRV_INVALID_PARAM.
 */
drv_status_t gpio_init(GPIO_TypeDef *port, const gpio_config_t *cfg);

/**
 * @brief Convenience wrapper to configure a single pin as a plain output.
 * @param port    GPIO port.
 * @param pin     Pin number 0..15.
 * @param otype   Push-pull or open-drain.
 * @param initial Initial output level.
 */
drv_status_t gpio_init_output(GPIO_TypeDef *port, uint8_t pin,
                              gpio_otype_t otype, gpio_state_t initial);

/**
 * @brief Convenience wrapper to configure a single pin as a digital input.
 * @param port GPIO port.
 * @param pin  Pin number 0..15.
 * @param pull Pull resistor selection.
 */
drv_status_t gpio_init_input(GPIO_TypeDef *port, uint8_t pin, gpio_pull_t pull);

/* -------------------------------------------------------------------------- */
/*  Fast access (inline, no bounds checking)                                  */
/* -------------------------------------------------------------------------- */

/** @brief Drive @p pin high using the atomic set/reset register. */
static inline void gpio_set(GPIO_TypeDef *port, uint8_t pin)
{
    port->BSRR = (uint32_t)GPIO_PIN(pin);
}

/** @brief Drive @p pin low using the atomic set/reset register. */
static inline void gpio_clear(GPIO_TypeDef *port, uint8_t pin)
{
    port->BSRR = (uint32_t)GPIO_PIN(pin) << 16U;
}

/** @brief Drive @p pin to @p state. */
static inline void gpio_write(GPIO_TypeDef *port, uint8_t pin, gpio_state_t state)
{
    if (state != GPIO_LOW) gpio_set(port, pin);
    else                   gpio_clear(port, pin);
}

/** @brief Toggle @p pin. */
static inline void gpio_toggle(GPIO_TypeDef *port, uint8_t pin)
{
    port->ODR ^= (uint32_t)GPIO_PIN(pin);
}

/** @brief Read the current input level of @p pin (GPIO_LOW/GPIO_HIGH). */
static inline gpio_state_t gpio_read(GPIO_TypeDef *port, uint8_t pin)
{
    return (port->IDR & GPIO_PIN(pin)) ? GPIO_HIGH : GPIO_LOW;
}

/** @brief Write a masked group of output bits at once (atomic set + reset). */
static inline void gpio_write_masked(GPIO_TypeDef *port, uint16_t mask,
                                     uint16_t value)
{
    port->BSRR = ((uint32_t)(~value & mask) << 16U) | (uint32_t)(value & mask);
}

/** @brief Read the whole input port register. */
static inline uint16_t gpio_read_port(GPIO_TypeDef *port)
{
    return (uint16_t)port->IDR;
}

#ifdef __cplusplus
}
#endif

#endif /* STATION_DRIVERS_GPIO_H */

/**
 ******************************************************************************
 * @file    drv_gpio.h
 * @brief   General-purpose I/O driver for the STM32F103.
 *
 * Pins are addressed by (GPIO_TypeDef*, pin) so any port the part offers works
 * — no enum to extend. Configuration goes through explicit mode/cnf enums that
 * mirror the F1 reference manual's CRL/CRH encoding.
 *
 * Fast access (write/read/toggle) is inline and uses BSRR/BRR, so a pin write
 * is a single atomic store that cannot be corrupted by an interrupt touching
 * another pin on the same port.
 ******************************************************************************
 */
#ifndef DRV_GPIO_H_
#define DRV_GPIO_H_

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Pin direction / output speed (CRL/CRH MODE field). */
typedef enum {
    GPIO_MODE_INPUT   = 0x0,  /**< Input.                                  */
    GPIO_MODE_OUT_10M = 0x1,  /**< Output, 10 MHz.                         */
    GPIO_MODE_OUT_2M  = 0x2,  /**< Output, 2 MHz.                          */
    GPIO_MODE_OUT_50M = 0x3,  /**< Output, 50 MHz.                         */
} GPIO_Mode;

/**
 * @brief Pin function (CRL/CRH CNF field).
 *
 * The meaning depends on MODE: the GPIO_CNF_IN_* values apply when the pin is
 * an input, the GPIO_CNF_OUT_* values when it is an output.
 */
typedef enum {
    GPIO_CNF_IN_ANALOG  = 0x0,  /**< Analog input.                         */
    GPIO_CNF_IN_FLOAT   = 0x1,  /**< Floating input (reset state).         */
    GPIO_CNF_IN_PULL    = 0x2,  /**< Input with pull-up or pull-down.      */

    GPIO_CNF_OUT_PP     = 0x0,  /**< General-purpose push-pull.            */
    GPIO_CNF_OUT_OD     = 0x1,  /**< General-purpose open-drain.           */
    GPIO_CNF_OUT_AF_PP  = 0x2,  /**< Alternate function push-pull.         */
    GPIO_CNF_OUT_AF_OD  = 0x3,  /**< Alternate function open-drain.        */
} GPIO_Cnf;

/** @brief Pull direction, used when cnf is GPIO_CNF_IN_PULL. */
typedef enum {
    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
} GPIO_Pull;

/** @brief Logical pin level. */
typedef enum {
    GPIO_LOW  = 0,
    GPIO_HIGH = 1,
} GPIO_State;

/* -------------------------------------------------------------------------- */
/*  Configuration                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configure one pin. Enables the port clock automatically.
 * @param port GPIOA..GPIOE.
 * @param pin  Pin number 0..15.
 * @param mode Direction / speed.
 * @param cnf  Function (interpretation depends on @p mode).
 * @param pull Pull direction; only meaningful for GPIO_CNF_IN_PULL.
 * @return DRV_OK or DRV_INVALID_PARAM.
 */
DRV_Status DRV_GPIO_Init(GPIO_TypeDef *port, uint8_t pin, GPIO_Mode mode,
                         GPIO_Cnf cnf, GPIO_Pull pull);

/** @brief Shorthand: push-pull output at 50 MHz, driven to @p initial. */
DRV_Status DRV_GPIO_InitOutput(GPIO_TypeDef *port, uint8_t pin,
                               GPIO_State initial);

/** @brief Shorthand: digital input with the given pull direction. */
DRV_Status DRV_GPIO_InitInput(GPIO_TypeDef *port, uint8_t pin, GPIO_Pull pull);

/** @brief Shorthand: alternate-function push-pull output at 50 MHz. */
DRV_Status DRV_GPIO_InitAF(GPIO_TypeDef *port, uint8_t pin);

/* -------------------------------------------------------------------------- */
/*  Fast access (inline, atomic, no bounds checking)                          */
/* -------------------------------------------------------------------------- */

/** @brief Drive @p pin high (atomic). */
static inline void DRV_GPIO_Set(GPIO_TypeDef *port, uint8_t pin)
{
    port->BSRR = DRV_BIT(pin);
}

/** @brief Drive @p pin low (atomic). */
static inline void DRV_GPIO_Clear(GPIO_TypeDef *port, uint8_t pin)
{
    port->BRR = DRV_BIT(pin);
}

/** @brief Drive @p pin to @p state (atomic). */
static inline void DRV_GPIO_Write(GPIO_TypeDef *port, uint8_t pin,
                                  GPIO_State state)
{
    /* BSRR: low half sets, high half resets — one store either way. */
    port->BSRR = (state != GPIO_LOW) ? DRV_BIT(pin) : (DRV_BIT(pin) << 16);
}

/** @brief Toggle @p pin. */
static inline void DRV_GPIO_Toggle(GPIO_TypeDef *port, uint8_t pin)
{
    port->ODR ^= DRV_BIT(pin);
}

/** @brief Read the input level of @p pin. */
static inline GPIO_State DRV_GPIO_Read(GPIO_TypeDef *port, uint8_t pin)
{
    return (port->IDR & DRV_BIT(pin)) ? GPIO_HIGH : GPIO_LOW;
}

/** @brief Read the whole 16-bit input port. */
static inline uint16_t DRV_GPIO_ReadPort(GPIO_TypeDef *port)
{
    return (uint16_t)port->IDR;
}

#ifdef __cplusplus
}
#endif

#endif /* DRV_GPIO_H_ */

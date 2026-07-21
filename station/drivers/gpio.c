/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   General-purpose I/O driver implementation (STM32F446RE).
 ******************************************************************************
 */
#include "gpio.h"
#include "rcc.h"

drv_status_t gpio_init(GPIO_TypeDef *port, const gpio_config_t *cfg)
{
    if (port == NULL || cfg == NULL || cfg->pin_mask == 0U) {
        return DRV_INVALID_PARAM;
    }

    rcc_gpio_clock_enable(port);

    for (uint8_t pin = 0U; pin < 16U; ++pin) {
        if ((cfg->pin_mask & GPIO_PIN(pin)) == 0U) {
            continue;
        }

        const uint32_t pos2 = (uint32_t)pin * 2U;  /* 2-bit field offset */

        /* MODER: direction / function. */
        DRV_MODIFY(port->MODER, (0x3UL << pos2),
                   ((uint32_t)cfg->mode << pos2));

        /* PUPDR: pull resistor. */
        DRV_MODIFY(port->PUPDR, (0x3UL << pos2),
                   ((uint32_t)cfg->pull << pos2));

        /* OTYPER: output stage type (1 bit per pin). */
        DRV_MODIFY(port->OTYPER, (0x1UL << pin),
                   ((uint32_t)cfg->otype << pin));

        /* OSPEEDR: output speed. */
        DRV_MODIFY(port->OSPEEDR, (0x3UL << pos2),
                   ((uint32_t)cfg->speed << pos2));

        /* AFRL/AFRH: alternate function (4 bits per pin) when in AF mode. */
        if (cfg->mode == GPIO_MODE_AF) {
            const uint32_t idx  = pin >> 3U;        /* 0 = AFR[0], 1 = AFR[1] */
            const uint32_t pos4 = ((uint32_t)pin & 0x7U) * 4U;
            DRV_MODIFY(port->AFR[idx], (0xFUL << pos4),
                       ((uint32_t)cfg->af << pos4));
        }
    }

    return DRV_OK;
}

drv_status_t gpio_init_output(GPIO_TypeDef *port, uint8_t pin,
                              gpio_otype_t otype, gpio_state_t initial)
{
    if (pin > 15U) {
        return DRV_INVALID_PARAM;
    }

    /* Set the initial level first so the pin never glitches when it switches
     * from input to output. */
    gpio_write(port, pin, initial);

    const gpio_config_t cfg = {
        .pin_mask = GPIO_PIN(pin),
        .mode     = GPIO_MODE_OUTPUT,
        .otype    = otype,
        .speed    = GPIO_SPEED_HIGH,
        .pull     = GPIO_PULL_NONE,
        .af       = GPIO_AF0,
    };
    return gpio_init(port, &cfg);
}

drv_status_t gpio_init_input(GPIO_TypeDef *port, uint8_t pin, gpio_pull_t pull)
{
    if (pin > 15U) {
        return DRV_INVALID_PARAM;
    }

    const gpio_config_t cfg = {
        .pin_mask = GPIO_PIN(pin),
        .mode     = GPIO_MODE_INPUT,
        .otype    = GPIO_OTYPE_PP,
        .speed    = GPIO_SPEED_LOW,
        .pull     = pull,
        .af       = GPIO_AF0,
    };
    return gpio_init(port, &cfg);
}

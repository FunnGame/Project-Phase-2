#include "drv_gpio.h"

DRV_Status DRV_GPIO_Init(GPIO_TypeDef *port, uint8_t pin, GPIO_Mode mode,
                         GPIO_Cnf cnf, GPIO_Pull pull)
{
    if (port == NULL || pin > 15u) {
        return DRV_INVALID_PARAM;
    }

    DRV_GPIO_ClockEnable(port);

    /* CRL/CRH nibble is [CNF(2) | MODE(2)]. */
    const uint8_t nibble = (uint8_t)((((uint8_t)cnf & 0x3u) << 2) |
                                     ((uint8_t)mode & 0x3u));
    DRV_GPIO_WriteCfgNibble(port, pin, nibble);

    /* For a pulled input the direction lives in ODR: 1 = up, 0 = down. */
    if (mode == GPIO_MODE_INPUT && cnf == GPIO_CNF_IN_PULL) {
        if (pull == GPIO_PULL_UP) {
            port->BSRR = DRV_BIT(pin);
        } else {
            port->BRR = DRV_BIT(pin);   /* pull-down (and the NONE fallback) */
        }
    }

    return DRV_OK;
}

DRV_Status DRV_GPIO_InitOutput(GPIO_TypeDef *port, uint8_t pin,
                               GPIO_State initial)
{
    /* Drive the level before switching direction so the pin never glitches. */
    if (port != NULL && pin <= 15u) {
        DRV_GPIO_Write(port, pin, initial);
    }
    return DRV_GPIO_Init(port, pin, GPIO_MODE_OUT_50M, GPIO_CNF_OUT_PP,
                         GPIO_PULL_NONE);
}

DRV_Status DRV_GPIO_InitInput(GPIO_TypeDef *port, uint8_t pin, GPIO_Pull pull)
{
    const GPIO_Cnf cnf = (pull == GPIO_PULL_NONE) ? GPIO_CNF_IN_FLOAT
                                                  : GPIO_CNF_IN_PULL;
    return DRV_GPIO_Init(port, pin, GPIO_MODE_INPUT, cnf, pull);
}

DRV_Status DRV_GPIO_InitAF(GPIO_TypeDef *port, uint8_t pin)
{
    return DRV_GPIO_Init(port, pin, GPIO_MODE_OUT_50M, GPIO_CNF_OUT_AF_PP,
                         GPIO_PULL_NONE);
}
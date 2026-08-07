#ifndef DRV_GPIO_H_
#define DRV_GPIO_H_

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    GPIO_MODE_INPUT   = 0x0,  /**< Input.                                  */
    GPIO_MODE_OUT_10M = 0x1,  /**< Output, 10 MHz.                         */
    GPIO_MODE_OUT_2M  = 0x2,  /**< Output, 2 MHz.                          */
    GPIO_MODE_OUT_50M = 0x3,  /**< Output, 50 MHz.                         */
} GPIO_Mode;


typedef enum {
    GPIO_CNF_IN_ANALOG  = 0x0,  /**< Analog input.                         */
    GPIO_CNF_IN_FLOAT   = 0x1,  /**< Floating input (reset state).         */
    GPIO_CNF_IN_PULL    = 0x2,  /**< Input with pull-up or pull-down.      */

    GPIO_CNF_OUT_PP     = 0x0,  /**< General-purpose push-pull.            */
    GPIO_CNF_OUT_OD     = 0x1,  /**< General-purpose open-drain.           */
    GPIO_CNF_OUT_AF_PP  = 0x2,  /**< Alternate function push-pull.         */
    GPIO_CNF_OUT_AF_OD  = 0x3,  /**< Alternate function open-drain.        */
} GPIO_Cnf;

typedef enum {
    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
} GPIO_Pull;


typedef enum {
    GPIO_LOW  = 0,
    GPIO_HIGH = 1,
} GPIO_State;


DRV_Status DRV_GPIO_Init(GPIO_TypeDef *port, uint8_t pin, GPIO_Mode mode,
                         GPIO_Cnf cnf, GPIO_Pull pull);


DRV_Status DRV_GPIO_InitOutput(GPIO_TypeDef *port, uint8_t pin,
                               GPIO_State initial);


DRV_Status DRV_GPIO_InitInput(GPIO_TypeDef *port, uint8_t pin, GPIO_Pull pull);

DRV_Status DRV_GPIO_InitAF(GPIO_TypeDef *port, uint8_t pin);


static inline void DRV_GPIO_Set(GPIO_TypeDef *port, uint8_t pin)
{
    port->BSRR = DRV_BIT(pin);
}

static inline void DRV_GPIO_Clear(GPIO_TypeDef *port, uint8_t pin)
{
    port->BRR = DRV_BIT(pin);
}


static inline void DRV_GPIO_Write(GPIO_TypeDef *port, uint8_t pin,
                                  GPIO_State state)
{
   
    port->BSRR = (state != GPIO_LOW) ? DRV_BIT(pin) : (DRV_BIT(pin) << 16);
}

static inline void DRV_GPIO_Toggle(GPIO_TypeDef *port, uint8_t pin)
{
    port->ODR ^= DRV_BIT(pin);
}

static inline GPIO_State DRV_GPIO_Read(GPIO_TypeDef *port, uint8_t pin)
{
    return (port->IDR & DRV_BIT(pin)) ? GPIO_HIGH : GPIO_LOW;
}

static inline uint16_t DRV_GPIO_ReadPort(GPIO_TypeDef *port)
{
    return (uint16_t)port->IDR;
}

#ifdef __cplusplus
}
#endif

#endif 
#include "../inc/drv_gpio.h"

static GPIO_TypeDef* GPIO_GetPort(GPIO_Port_t port)
{
    switch(port)
    {
        case GPIO_PORT_A: return GPIOA;
        case GPIO_PORT_B: return GPIOB;
        case GPIO_PORT_C: return GPIOC;
        default:          return NULL;
    }
}

static void GPIO_EnableClock(GPIO_Port_t port)
{
    switch(port)
    {
        case GPIO_PORT_A:
            RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
            break;

        case GPIO_PORT_B:
            RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
            break;

        case GPIO_PORT_C:
            RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
            break;

        default:
            break;
    }
}


Status_t DRV_GPIO_Init(GPIO_Port_t port,
                       GPIO_Pin_t pin,
                       GPIO_Mode_t mode,
                       uint8 config)
{
    GPIO_TypeDef *gpio = GPIO_GetPort(port);

    if(gpio == NULL)
    {
        return STATUS_INVALID_PARAM;
    }

    GPIO_EnableClock(port);

    uint32 pinConfig = ((config & 0x03U) << 2U) | (mode & 0x03U);

    if(pin < GPIO_PIN_8)
    {
        gpio->CRL &= ~(0x0FU << (pin * 4U));
        gpio->CRL |=  (pinConfig << (pin * 4U));
    }
    else
    {
        uint8 tempPin = pin - 8U;

        gpio->CRH &= ~(0x0FU << (tempPin * 4U));
        gpio->CRH |=  (pinConfig << (tempPin * 4U));
    }

    /* Pull-up input */
    if((mode == GPIO_MODE_INPUT) &&
       (config == GPIO_INPUT_PULL))
    {
        gpio->ODR |= (1U << pin);
    }

    return STATUS_OK;
}

Status_t DRV_GPIO_WritePin(GPIO_Port_t port,
                           GPIO_Pin_t pin,
                           GPIO_PinState_t state)
{
    GPIO_TypeDef *gpio = GPIO_GetPort(port);

    if(gpio == NULL)
    {
        return STATUS_INVALID_PARAM;
    }

    if(state == GPIO_PIN_SET)
    {
        gpio->BSRR = (1U << pin);
    }
    else
    {
        gpio->BRR = (1U << pin);
    }

    return STATUS_OK;
}

GPIO_PinState_t DRV_GPIO_ReadPin(GPIO_Port_t port,
                                 GPIO_Pin_t pin)
{
    GPIO_TypeDef *gpio = GPIO_GetPort(port);

    if(gpio == NULL)
    {
        return GPIO_PIN_RESET;
    }

    return ((gpio->IDR & (1U << pin)) != 0U) ?
            GPIO_PIN_SET :
            GPIO_PIN_RESET;
}

Status_t DRV_GPIO_TogglePin(GPIO_Port_t port,
                            GPIO_Pin_t pin)
{
    GPIO_TypeDef *gpio = GPIO_GetPort(port);

    if(gpio == NULL)
    {
        return STATUS_INVALID_PARAM;
    }

    gpio->ODR ^= (1U << pin);

    return STATUS_OK;
}
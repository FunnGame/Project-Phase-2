#ifndef DRV_GPIO_H
#define DRV_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"
#include "types.h"

typedef enum
{
    GPIO_PORT_A = 0U,
    GPIO_PORT_B,
    GPIO_PORT_C

} GPIO_Port_t;

typedef enum
{
    GPIO_PIN_0 = 0U,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7,
    GPIO_PIN_8,
    GPIO_PIN_9,
    GPIO_PIN_10,
    GPIO_PIN_11,
    GPIO_PIN_12,
    GPIO_PIN_13,
    GPIO_PIN_14,
    GPIO_PIN_15

} GPIO_Pin_t;

typedef enum
{
    GPIO_MODE_INPUT = 0U,
    GPIO_MODE_OUTPUT_10MHz,
    GPIO_MODE_OUTPUT_2MHz,
    GPIO_MODE_OUTPUT_50MHz

} GPIO_Mode_t;

typedef enum
{
    GPIO_INPUT_ANALOG = 0U,
    GPIO_INPUT_FLOATING,
    GPIO_INPUT_PULL

} GPIO_InputType_t;

typedef enum
{
    GPIO_OUTPUT_PP = 0U,
    GPIO_OUTPUT_OD,
    GPIO_AF_PP,
    GPIO_AF_OD

} GPIO_OutputType_t;

/* Pin State */
typedef enum
{
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET

} GPIO_PinState_t;

Status_t DRV_GPIO_Init(
    GPIO_Port_t port,
    GPIO_Pin_t pin,
    GPIO_Mode_t mode,
    uint8 config);

Status_t DRV_GPIO_WritePin(
    GPIO_Port_t port,
    GPIO_Pin_t pin,
    GPIO_PinState_t state);

GPIO_PinState_t DRV_GPIO_ReadPin(
    GPIO_Port_t port,
    GPIO_Pin_t pin);

Status_t DRV_GPIO_TogglePin(
    GPIO_Port_t port,
    GPIO_Pin_t pin);

#ifdef __cplusplus
}
#endif

#endif /* DRV_GPIO_H */
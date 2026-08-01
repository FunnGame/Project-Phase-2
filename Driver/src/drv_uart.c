#ifndef DRV_UART_H
#define DRV_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"
#include "types.h"

typedef enum
{
    UART_1 = 0U,
    UART_2,
    UART_3

} UART_t;

/* Initialization */
Status_t DRV_UART_Init(UART_t uart, uint32 baudrate);

/* Transmit */
Status_t DRV_UART_SendByte(UART_t uart, uint8 data);

Status_t DRV_UART_SendBytes(UART_t uart,
                            const uint8 *data,
                            uint16 length);

/* Receive */
Status_t DRV_UART_ReadByte(UART_t uart,
                           uint8 *data);

#ifdef __cplusplus
}
#endif

#endif
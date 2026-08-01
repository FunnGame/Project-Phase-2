
#ifndef ERROR_H
#define ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"


typedef enum
{
    /* General */
    ERROR_NONE = 0,

    ERROR_UNKNOWN,
    ERROR_BUSY,
    ERROR_TIMEOUT,
    ERROR_INVALID_PARAM,
    ERROR_NULL_POINTER,

    /* GPIO */
    ERROR_GPIO_INIT,

    /* TIMER */
    ERROR_TIMER_INIT,

    /* I2C */
    ERROR_I2C_INIT,
    ERROR_I2C_TIMEOUT,
    ERROR_I2C_NACK,
    ERROR_I2C_BUS,

    /* CAN */
    ERROR_CAN_INIT,
    ERROR_CAN_TX,
    ERROR_CAN_RX,
    ERROR_CAN_BUSOFF,

    /* UART */
    ERROR_UART_INIT,
    ERROR_UART_TX,
    ERROR_UART_RX,

    /* VL53 */
    ERROR_VL53_INIT,
    ERROR_VL53_TIMEOUT,
    ERROR_VL53_OUT_OF_RANGE,

    /* MPU */
    ERROR_MPU_INIT,
    ERROR_MPU_CALIBRATION,
    ERROR_MPU_DATA_INVALID,

    /* Application */
    ERROR_SENSOR_NOT_READY,
    ERROR_SENSOR_DATA_INVALID,
    ERROR_CAN_MESSAGE_LOST

} ErrorCode_t;

#ifdef __cplusplus
}
#endif

#endif /* ERROR_H */
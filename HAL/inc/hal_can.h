#ifndef HAL_CAN_H
#define HAL_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"
#include "error.h"
#include "drv_can.h"

/* Initialize CAN */
Status_t HAL_CAN_Init(CAN_Baudrate_t baudrate);

/* Send CAN message */
Status_t HAL_CAN_Send(const CAN_Message_t *message);

/* Receive CAN message */
Status_t HAL_CAN_Receive(CAN_Message_t *message);

/* Configure CAN filter */
Status_t HAL_CAN_SetFilter(uint16 filterId,
                           uint16 filterMask);

/* Reset CAN */
Status_t HAL_CAN_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_CAN_H */
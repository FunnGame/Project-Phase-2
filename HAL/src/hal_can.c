#include "../inc/hal_can.h"

Status_t HAL_CAN_Init(CAN_Baudrate_t baudrate)
{
    return DRV_CAN_Init(CAN_1, baudrate);
}


Status_t HAL_CAN_Send(const CAN_Message_t *message)
{
    if(message == NULL)
    {
        return STATUS_INVALID_PARAM;
    }

    return DRV_CAN_Send(CAN_1, message);
}


Status_t HAL_CAN_Receive(CAN_Message_t *message)
{
    if(message == NULL)
    {
        return STATUS_INVALID_PARAM;
    }

    return DRV_CAN_Receive(CAN_1, message);
}


Status_t HAL_CAN_SetFilter(uint16 filterId,
                           uint16 filterMask)
{
    return DRV_CAN_SetFilter(CAN_1,
                             filterId,
                             filterMask);
}


Status_t HAL_CAN_Reset(void)
{
    return DRV_CAN_Reset(CAN_1);
}
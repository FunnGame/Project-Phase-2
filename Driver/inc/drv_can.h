#ifndef DRV_CAN_H
#define DRV_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"
#include "types.h"
#include "error.h"

/* CAN Peripheral */

typedef enum
{
    CAN_1 = 0U

} CAN_t;


/* CAN Bit Rate */

typedef enum
{
    CAN_BAUDRATE_125K = 125000U,
    CAN_BAUDRATE_250K = 250000U,
    CAN_BAUDRATE_500K = 500000U,
    CAN_BAUDRATE_1M   = 1000000U

} CAN_Baudrate_t;


/* CAN Frame Type */

typedef enum
{
    CAN_FRAME_DATA = 0U,
    CAN_FRAME_REMOTE

} CAN_FrameType_t;


/* CAN ID Type */

typedef enum
{
    CAN_ID_STANDARD = 0U,
    CAN_ID_EXTENDED

} CAN_IdType_t;


/* CAN Message */

typedef struct
{
    uint32 id;

    CAN_IdType_t idType;

    CAN_FrameType_t frameType;

    uint8 dlc;

    uint8 data[8];

} CAN_Message_t;


/*  CAN Initialization */

Status_t DRV_CAN_Init(
    CAN_t can,
    CAN_Baudrate_t baudrate);


/* CAN Transmit */

Status_t DRV_CAN_Send(
    CAN_t can,
    const CAN_Message_t *message);


/* CAN Receive */

Status_t DRV_CAN_Receive(
    CAN_t can,
    CAN_Message_t *message);


/*  CAN Filter */

Status_t DRV_CAN_SetFilter(
    CAN_t can,
    uint16 filterId,
    uint16 filterMask);


/*  CAN Reset */

Status_t DRV_CAN_Reset(
    CAN_t can);


#ifdef __cplusplus
}
#endif

#endif /* DRV_CAN_H */
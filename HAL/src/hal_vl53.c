#include "hal_vl53.h"
#include "VL53L1X_api.h"

#define VL53_DEV_ADDR      (0x29)

static Status_t VL53_Status(VL53L1X_ERROR err)
{
    if(err == VL53L1X_ERROR_NONE)
    {
        return STATUS_OK;
    }

    return STATUS_ERROR;
}

Status_t HAL_VL53_Init(void)
{
    uint8 state = 0;

    while(state == 0)
    {
        if(VL53_Status(
            VL53L1X_BootState(
                VL53_DEV_ADDR,
                &state)) != STATUS_OK)
        {
            return STATUS_ERROR;
        }
    }

    if(VL53_Status(
        VL53L1X_SensorInit(
            VL53_DEV_ADDR)) != STATUS_OK)
    {
        return STATUS_ERROR;
    }

    VL53L1X_SetDistanceMode(
        VL53_DEV_ADDR,
        2);

    VL53L1X_SetTimingBudgetInMs(
        VL53_DEV_ADDR,
        50);

    VL53L1X_SetInterMeasurementInMs(
        VL53_DEV_ADDR,
        50);

    return STATUS_OK;
}

Status_t HAL_VL53_Start(void)
{
    return VL53_Status(
        VL53L1X_StartRanging(
            VL53_DEV_ADDR));
}

Status_t HAL_VL53_Stop(void)
{
    return VL53_Status(
        VL53L1X_StopRanging(
            VL53_DEV_ADDR));
}

Status_t HAL_VL53_IsDataReady(uint8 *ready)
{
    return VL53_Status(
        VL53L1X_CheckForDataReady(
            VL53_DEV_ADDR,
            ready));
}

Status_t HAL_VL53_ReadDistance(uint16 *distance)
{
    return VL53_Status(
        VL53L1X_GetDistance(
            VL53_DEV_ADDR,
            distance));
}

Status_t HAL_VL53_ClearInterrupt(void)
{
    return VL53_Status(
        VL53L1X_ClearInterrupt(
            VL53_DEV_ADDR));
}
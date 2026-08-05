#include "drv_mpu.h"


/* ============================================================
 * Configuration
 * ============================================================ */

#define MPU_I2C_INSTANCE       I2C_1
#define MPU_DEFAULT_ADDRESS    MPU_I2C_ADDR_AD0_LOW

#define MPU_DEVICE_COUNT       (1U)


/* ============================================================
 * Private Data
 * ============================================================ */

static uint8_t mpu_i2c_address[MPU_DEVICE_COUNT] =
{
    MPU_DEFAULT_ADDRESS
};


/* Private Functions */

static Status_t DRV_MPU_WriteByte(
    MPU_t mpu,
    uint8_t reg,
    uint8_t data)
{
    if((uint32_t)mpu >= MPU_DEVICE_COUNT)
    {
        return STATUS_ERROR;
    }

    return DRV_I2C_Write(
        MPU_I2C_INSTANCE,
        mpu_i2c_address[mpu],
        (uint16_t)reg,
        &data,
        1U);
}


static Status_t DRV_MPU_ReadByte(
    MPU_t mpu,
    uint8_t reg,
    uint8_t *data)
{
    if((uint32_t)mpu >= MPU_DEVICE_COUNT)
    {
        return STATUS_ERROR;
    }

    if(data == (uint8_t *)0)
    {
        return STATUS_ERROR;
    }

    return DRV_I2C_Read(
        MPU_I2C_INSTANCE,
        mpu_i2c_address[mpu],
        (uint16_t)reg,
        data,
        1U);
}


static Status_t DRV_MPU_ReadBytes(
    MPU_t mpu,
    uint8_t reg,
    uint8_t *data,
    uint16_t length)
{
    if((uint32_t)mpu >= MPU_DEVICE_COUNT)
    {
        return STATUS_ERROR;
    }

    if((data == (uint8_t *)0) || (length == 0U))
    {
        return STATUS_ERROR;
    }

    return DRV_I2C_Read(
        MPU_I2C_INSTANCE,
        mpu_i2c_address[mpu],
        (uint16_t)reg,
        data,
        length);
}


/*  Initialization */

Status_t DRV_MPU_Init(
    MPU_t mpu,
    uint8_t i2c_address)
{
    Status_t status;
    uint8_t who_am_i;

    if((uint32_t)mpu >= MPU_DEVICE_COUNT)
    {
        return STATUS_ERROR;
    }

    if((i2c_address != MPU_I2C_ADDR_AD0_LOW) &&
       (i2c_address != MPU_I2C_ADDR_AD0_HIGH))
    {
        return STATUS_ERROR;
    }

    mpu_i2c_address[mpu] = i2c_address;

    /* Reset MPU */
    status = DRV_MPU_WriteByte(
        mpu,
        MPU_REG_PWR_MGMT_1,
        0x80U);

    if(status != STATUS_OK)
    {
        return status;
    }

    /*  Wake MPU and select internal oscillator */
    status = DRV_MPU_WriteByte(
        mpu,
        MPU_REG_PWR_MGMT_1,
        0x00U);

    if(status != STATUS_OK)
    {
        return status;
    }

    /* Check device identity */
    status = DRV_MPU_ReadWhoAmI(
        mpu,
        &who_am_i);

    if(status != STATUS_OK)
    {
        return status;
    }

    /*
     * MPU6050/MPU6500 do not necessarily share
     * the same WHO_AM_I value, so identity validation
     * is intentionally left to the upper layer.
     */

    return STATUS_OK;
}


/*  WHO AM I */

Status_t DRV_MPU_ReadWhoAmI(
    MPU_t mpu,
    uint8_t *id)
{
    return DRV_MPU_ReadByte(
        mpu,
        MPU_REG_WHO_AM_I,
        id);
}


/*  Accelerometer Range */

Status_t DRV_MPU_SetAccelRange(
    MPU_t mpu,
    MPU_AccelRange_t range)
{
    uint8_t value;

    if((uint32_t)range > (uint32_t)MPU_ACCEL_FS_16G)
    {
        return STATUS_ERROR;
    }

    value = ((uint8_t)range << 3U);

    return DRV_MPU_WriteByte(
        mpu,
        MPU_REG_ACCEL_CONFIG,
        value);
}


/*  Gyroscope Range */

Status_t DRV_MPU_SetGyroRange(
    MPU_t mpu,
    MPU_GyroRange_t range)
{
    uint8_t value;

    if((uint32_t)range > (uint32_t)MPU_GYRO_FS_2000DPS)
    {
        return STATUS_ERROR;
    }

    value = ((uint8_t)range << 3U);

    return DRV_MPU_WriteByte(
        mpu,
        MPU_REG_GYRO_CONFIG,
        value);
}


/*  Accelerometer */

Status_t DRV_MPU_ReadAccel(
    MPU_t mpu,
    int16_t *accel_x,
    int16_t *accel_y,
    int16_t *accel_z)
{
    uint8_t buffer[6U];
    Status_t status;

    if((accel_x == (int16_t *)0) ||
       (accel_y == (int16_t *)0) ||
       (accel_z == (int16_t *)0))
    {
        return STATUS_ERROR;
    }

    status = DRV_MPU_ReadBytes(
        mpu,
        MPU_REG_ACCEL_XOUT_H,
        buffer,
        6U);

    if(status != STATUS_OK)
    {
        return status;
    }

    *accel_x = (int16_t)(((uint16_t)buffer[0U] << 8U) |
                          (uint16_t)buffer[1U]);

    *accel_y = (int16_t)(((uint16_t)buffer[2U] << 8U) |
                          (uint16_t)buffer[3U]);

    *accel_z = (int16_t)(((uint16_t)buffer[4U] << 8U) |
                          (uint16_t)buffer[5U]);

    return STATUS_OK;
}


/*  Gyroscope */

Status_t DRV_MPU_ReadGyro(
    MPU_t mpu,
    int16_t *gyro_x,
    int16_t *gyro_y,
    int16_t *gyro_z)
{
    uint8_t buffer[6U];
    Status_t status;

    if((gyro_x == (int16_t *)0) ||
       (gyro_y == (int16_t *)0) ||
       (gyro_z == (int16_t *)0))
    {
        return STATUS_ERROR;
    }

    status = DRV_MPU_ReadBytes(
        mpu,
        MPU_REG_GYRO_XOUT_H,
        buffer,
        6U);

    if(status != STATUS_OK)
    {
        return status;
    }

    *gyro_x = (int16_t)(((uint16_t)buffer[0U] << 8U) |
                         (uint16_t)buffer[1U]);

    *gyro_y = (int16_t)(((uint16_t)buffer[2U] << 8U) |
                         (uint16_t)buffer[3U]);

    *gyro_z = (int16_t)(((uint16_t)buffer[4U] << 8U) |
                         (uint16_t)buffer[5U]);

    return STATUS_OK;
}


/*  Temperature */

Status_t DRV_MPU_ReadTemperature(
    MPU_t mpu,
    int16_t *temperature)
{
    uint8_t buffer[2U];
    Status_t status;

    if(temperature == (int16_t *)0)
    {
        return STATUS_ERROR;
    }

    status = DRV_MPU_ReadBytes(
        mpu,
        MPU_REG_TEMP_OUT_H,
        buffer,
        2U);

    if(status != STATUS_OK)
    {
        return status;
    }

    *temperature = (int16_t)(((uint16_t)buffer[0U] << 8U) |
                              (uint16_t)buffer[1U]);

    return STATUS_OK;
}


/*  Read All Raw Data */

Status_t DRV_MPU_ReadRawData(
    MPU_t mpu,
    MPU_RawData_t *data)
{
    uint8_t buffer[14U];
    Status_t status;

    if(data == (MPU_RawData_t *)0)
    {
        return STATUS_ERROR;
    }

    status = DRV_MPU_ReadBytes(
        mpu,
        MPU_REG_ACCEL_XOUT_H,
        buffer,
        14U);

    if(status != STATUS_OK)
    {
        return status;
    }

    data->accel_x = (int16_t)(((uint16_t)buffer[0U] << 8U) |
                               (uint16_t)buffer[1U]);

    data->accel_y = (int16_t)(((uint16_t)buffer[2U] << 8U) |
                               (uint16_t)buffer[3U]);

    data->accel_z = (int16_t)(((uint16_t)buffer[4U] << 8U) |
                               (uint16_t)buffer[5U]);

    data->temperature = (int16_t)(((uint16_t)buffer[6U] << 8U) |
                                    (uint16_t)buffer[7U]);

    data->gyro_x = (int16_t)(((uint16_t)buffer[8U] << 8U) |
                              (uint16_t)buffer[9U]);

    data->gyro_y = (int16_t)(((uint16_t)buffer[10U] << 8U) |
                              (uint16_t)buffer[11U]);

    data->gyro_z = (int16_t)(((uint16_t)buffer[12U] << 8U) |
                              (uint16_t)buffer[13U]);

    return STATUS_OK;
}

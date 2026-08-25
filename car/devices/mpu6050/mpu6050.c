/**
 ******************************************************************************
 * @file    mpu6050.c
 * @brief   MPU6050 six-axis IMU driver implementation.
 ******************************************************************************
 */
#include "mpu6050.h"

#include "drv_i2c.h"
#include "drv_timer.h"
#include "vehicle_config.h"

/* Resolved once from the node config, so nothing below names a bus or an
 * address directly. */
#define MPU_I2C     CAR_MPU_I2C
#define MPU_ADDR    CAR_MPU_ADDR

/* The configured ranges are kept because a caller cannot scale raw counts
 * without knowing them, and the device offers no way to ask. */
static MPU6050_AccelRange_t s_accel_range = MPU6050_ACCEL_FS_2G;
static MPU6050_GyroRange_t  s_gyro_range  = MPU6050_GYRO_FS_250DPS;

/* -------------------------------------------------------------------------- */
/*  Private register access                                                   */
/* -------------------------------------------------------------------------- */

static DRV_Status mpu_write_byte(uint8_t reg, uint8_t value)
{
    return DRV_I2C_MemWrite(MPU_I2C, MPU_ADDR, reg, &value, 1U);
}

static DRV_Status mpu_read_bytes(uint8_t reg, uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return DRV_INVALID_PARAM;
    }
    return DRV_I2C_MemRead(MPU_I2C, MPU_ADDR, reg, data, len);
}

/** Rebuild one big-endian signed 16-bit sample from a register pair. */
static inline int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* -------------------------------------------------------------------------- */
/*  Initialisation                                                            */
/* -------------------------------------------------------------------------- */

DRV_Status MPU6050_Init(void)
{
    DRV_Status st;
    uint8_t    who = 0U;

    /* 1. Device reset. Everything before this is undefined after power-on. */
    st = mpu_write_byte(MPU6050_REG_PWR_MGMT_1, 0x80U);
    if (st != DRV_OK) {
        return st;
    }
    DRV_Delay_Ms(100U);       /* mandatory: the reset is not instantaneous */

    /* 2. Wake up and run off the internal oscillator. The device boots into
     *    sleep, so without this every subsequent read returns zeros. */
    st = mpu_write_byte(MPU6050_REG_PWR_MGMT_1, 0x00U);
    if (st != DRV_OK) {
        return st;
    }
    DRV_Delay_Ms(10U);

    /* 3. Identity check. An I2C read succeeds against a floating bus as
     *    readily as against a real device, so the ACK alone proves nothing —
     *    this is what actually distinguishes "present" from "miswired". */
    st = MPU6050_ReadWhoAmI(&who);
    if (st != DRV_OK) {
        return st;
    }
    if (who != MPU6050_WHO_AM_I_VALUE) {
        return DRV_ERROR;
    }

    /* 4. Anti-alias filter and sample rate, BEFORE the ranges.
     *
     * With DLPF enabled the internal rate is 1 kHz, so SMPLRT_DIV sets the
     * output rate to 1000/(1+div). Matching it to how often the node actually
     * reads means every sample is fresh rather than a repeat, which matters
     * for a first-difference or an integral. */
    st = mpu_write_byte(MPU6050_REG_CONFIG, (uint8_t)CAR_MPU_DLPF);
    if (st != DRV_OK) {
        return st;
    }
    st = mpu_write_byte(MPU6050_REG_SMPLRT_DIV, (uint8_t)CAR_MPU_SMPLRT_DIV);
    if (st != DRV_OK) {
        return st;
    }

    /* 5. Full-scale ranges, from the node config. */
    st = MPU6050_SetAccelRange(CAR_MPU_ACCEL_RANGE);
    if (st != DRV_OK) {
        return st;
    }
    return MPU6050_SetGyroRange(CAR_MPU_GYRO_RANGE);
}

/* -------------------------------------------------------------------------- */
/*  Configuration                                                             */
/* -------------------------------------------------------------------------- */

DRV_Status MPU6050_Resume(void)
{
    uint8_t who = 0U;

    if (MPU6050_ReadWhoAmI(&who) != DRV_OK) {
        return DRV_TIMEOUT;
    }
    if (who != MPU6050_WHO_AM_I_VALUE) {
        return DRV_ERROR;
    }

    /* Harmless if it never slept; essential if it browned out and came back. */
    const DRV_Status st = mpu_write_byte(MPU6050_REG_PWR_MGMT_1, 0x00U);
    if (st != DRV_OK) {
        return st;
    }

    /* A device that reset is back on its DEFAULTS: no filter, and the wrong
     * ranges. Re-assert everything, or a recovered IMU quietly starts feeding
     * the integrator unfiltered noise again. */
    if (mpu_write_byte(MPU6050_REG_CONFIG, (uint8_t)CAR_MPU_DLPF) != DRV_OK) {
        return DRV_ERROR;
    }
    if (mpu_write_byte(MPU6050_REG_SMPLRT_DIV,
                       (uint8_t)CAR_MPU_SMPLRT_DIV) != DRV_OK) {
        return DRV_ERROR;
    }

    if (MPU6050_SetAccelRange(s_accel_range) != DRV_OK) {
        return DRV_ERROR;
    }
    return MPU6050_SetGyroRange(s_gyro_range);
}

DRV_Status MPU6050_ReadWhoAmI(uint8_t *id)
{
    return mpu_read_bytes(MPU6050_REG_WHO_AM_I, id, 1U);
}

DRV_Status MPU6050_SetAccelRange(MPU6050_AccelRange_t range)
{
    if ((uint32_t)range > (uint32_t)MPU6050_ACCEL_FS_16G) {
        return DRV_INVALID_PARAM;
    }

    const DRV_Status st = mpu_write_byte(MPU6050_REG_ACCEL_CONFIG,
                                         (uint8_t)((uint8_t)range << 3U));
    if (st == DRV_OK) {
        s_accel_range = range;
    }
    return st;
}

DRV_Status MPU6050_SetGyroRange(MPU6050_GyroRange_t range)
{
    if ((uint32_t)range > (uint32_t)MPU6050_GYRO_FS_2000DPS) {
        return DRV_INVALID_PARAM;
    }

    const DRV_Status st = mpu_write_byte(MPU6050_REG_GYRO_CONFIG,
                                         (uint8_t)((uint8_t)range << 3U));
    if (st == DRV_OK) {
        s_gyro_range = range;
    }
    return st;
}

MPU6050_AccelRange_t MPU6050_GetAccelRange(void) { return s_accel_range; }
MPU6050_GyroRange_t  MPU6050_GetGyroRange(void)  { return s_gyro_range;  }

/* -------------------------------------------------------------------------- */
/*  Measurement                                                               */
/* -------------------------------------------------------------------------- */

DRV_Status MPU6050_ReadAccel(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];

    if (x == NULL || y == NULL || z == NULL) {
        return DRV_INVALID_PARAM;
    }

    const DRV_Status st = mpu_read_bytes(MPU6050_REG_ACCEL_XOUT_H, buf,
                                         sizeof buf);
    if (st != DRV_OK) {
        return st;
    }

    *x = be16(&buf[0]);
    *y = be16(&buf[2]);
    *z = be16(&buf[4]);
    return DRV_OK;
}

DRV_Status MPU6050_ReadGyro(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];

    if (x == NULL || y == NULL || z == NULL) {
        return DRV_INVALID_PARAM;
    }

    const DRV_Status st = mpu_read_bytes(MPU6050_REG_GYRO_XOUT_H, buf,
                                         sizeof buf);
    if (st != DRV_OK) {
        return st;
    }

    *x = be16(&buf[0]);
    *y = be16(&buf[2]);
    *z = be16(&buf[4]);
    return DRV_OK;
}

DRV_Status MPU6050_ReadTemperature(int16_t *temperature)
{
    uint8_t buf[2];

    if (temperature == NULL) {
        return DRV_INVALID_PARAM;
    }

    const DRV_Status st = mpu_read_bytes(MPU6050_REG_TEMP_OUT_H, buf,
                                         sizeof buf);
    if (st != DRV_OK) {
        return st;
    }

    *temperature = be16(&buf[0]);
    return DRV_OK;
}

DRV_Status MPU6050_ReadRaw(MPU6050_Raw_t *out)
{
    uint8_t buf[14];

    if (out == NULL) {
        return DRV_INVALID_PARAM;
    }

    /* One burst across all fourteen output registers: the pointer
     * auto-increments, so accel, temperature and gyro all come from the same
     * sample instant. Three separate reads would not. */
    const DRV_Status st = mpu_read_bytes(MPU6050_REG_ACCEL_XOUT_H, buf,
                                         sizeof buf);
    if (st != DRV_OK) {
        return st;
    }

    out->accel_x     = be16(&buf[0]);
    out->accel_y     = be16(&buf[2]);
    out->accel_z     = be16(&buf[4]);
    out->temperature = be16(&buf[6]);
    out->gyro_x      = be16(&buf[8]);
    out->gyro_y      = be16(&buf[10]);
    out->gyro_z      = be16(&buf[12]);
    return DRV_OK;
}

#ifndef DRV_MPU_H
#define DRV_MPU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"
#include "drv_i2c.h"
#include <stdint.h>

/*  MPU Device */

typedef enum
{
    MPU_1 = 0U

} MPU_t;


/* MPU I2C Address */

#define MPU_I2C_ADDR_AD0_LOW      (0xD0U)
#define MPU_I2C_ADDR_AD0_HIGH     (0xD2U)


/*  MPU Register Map */

#define MPU_REG_SMPLRT_DIV        (0x19U)
#define MPU_REG_CONFIG            (0x1AU)
#define MPU_REG_GYRO_CONFIG       (0x1BU)
#define MPU_REG_ACCEL_CONFIG      (0x1CU)

#define MPU_REG_ACCEL_XOUT_H      (0x3BU)
#define MPU_REG_ACCEL_XOUT_L      (0x3CU)
#define MPU_REG_ACCEL_YOUT_H      (0x3DU)
#define MPU_REG_ACCEL_YOUT_L      (0x3EU)
#define MPU_REG_ACCEL_ZOUT_H      (0x3FU)
#define MPU_REG_ACCEL_ZOUT_L      (0x40U)

#define MPU_REG_TEMP_OUT_H        (0x41U)
#define MPU_REG_TEMP_OUT_L        (0x42U)

#define MPU_REG_GYRO_XOUT_H       (0x43U)
#define MPU_REG_GYRO_XOUT_L       (0x44U)
#define MPU_REG_GYRO_YOUT_H       (0x45U)
#define MPU_REG_GYRO_YOUT_L       (0x46U)
#define MPU_REG_GYRO_ZOUT_H       (0x47U)
#define MPU_REG_GYRO_ZOUT_L       (0x48U)

#define MPU_REG_PWR_MGMT_1        (0x6BU)
#define MPU_REG_PWR_MGMT_2        (0x6CU)

#define MPU_REG_WHO_AM_I          (0x75U)


/* Configuration */

typedef enum
{
    MPU_ACCEL_FS_2G = 0U,
    MPU_ACCEL_FS_4G,
    MPU_ACCEL_FS_8G,
    MPU_ACCEL_FS_16G

} MPU_AccelRange_t;


typedef enum
{
    MPU_GYRO_FS_250DPS = 0U,
    MPU_GYRO_FS_500DPS,
    MPU_GYRO_FS_1000DPS,
    MPU_GYRO_FS_2000DPS

} MPU_GyroRange_t;


/* ============================================================
 * Raw Sensor Data
 * ============================================================ */

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t temperature;

} MPU_RawData_t;


/*  API */

Status_t DRV_MPU_Init(
    MPU_t mpu,
    uint8_t i2c_address);

Status_t DRV_MPU_ReadWhoAmI(
    MPU_t mpu,
    uint8_t *id);

Status_t DRV_MPU_SetAccelRange(
    MPU_t mpu,
    MPU_AccelRange_t range);

Status_t DRV_MPU_SetGyroRange(
    MPU_t mpu,
    MPU_GyroRange_t range);

Status_t DRV_MPU_ReadAccel(
    MPU_t mpu,
    int16_t *accel_x,
    int16_t *accel_y,
    int16_t *accel_z);

Status_t DRV_MPU_ReadGyro(
    MPU_t mpu,
    int16_t *gyro_x,
    int16_t *gyro_y,
    int16_t *gyro_z);

Status_t DRV_MPU_ReadTemperature(
    MPU_t mpu,
    int16_t *temperature);

Status_t DRV_MPU_ReadRawData(
    MPU_t mpu,
    MPU_RawData_t *data);

#ifdef __cplusplus
}
#endif

#endif

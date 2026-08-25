/**
 ******************************************************************************
 * @file    mpu6050.h
 * @brief   MPU6050 six-axis IMU (accelerometer + gyroscope) over I2C.
 *
 * The vehicle-control node's yaw rate is currently kinematic — derived from the
 * wheel-speed difference — so it reads zero for a skidding wheel. This sensor is
 * what closes that gap.
 *
 * The bus instance, slave address and full-scale ranges all come from
 * vehicle_config.h (CAR_MPU_*); nothing here is hard-coded. Like the TB6612
 * modules, this only ever builds into the vehicle-control node.
 *
 * Raw int16 counts out, deliberately. Turning gyro counts into the mrad/s that
 * VC_Motion carries needs a bias estimate captured at standstill, and that is
 * filtering rather than device access — it belongs in car/lib beside the mixer.
 *
 * REQUIRES, before MPU6050_Init():
 *   - DRV_I2C_Init() on the configured bus,
 *   - DRV_Delay_Init(), for the reset settling delays.
 ******************************************************************************
 */
#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/*  Slave address                                                             */
/* -------------------------------------------------------------------------- */
/*
 * SEVEN-bit addresses. The datasheet and most example code quote 0xD0/0xD2,
 * which is the same address already shifted left with the R/W bit in place;
 * DRV_I2C_MemRead/MemWrite do that shift themselves, so passing the 8-bit form
 * addresses nothing that exists.
 */
#define MPU6050_ADDR_AD0_LOW      (0x68U)   /**< AD0 tied low  (8-bit: 0xD0) */
#define MPU6050_ADDR_AD0_HIGH     (0x69U)   /**< AD0 tied high (8-bit: 0xD2) */

/** Value WHO_AM_I returns on a genuine MPU6050, whatever AD0 is strapped to. */
#define MPU6050_WHO_AM_I_VALUE    (0x68U)

/* -------------------------------------------------------------------------- */
/*  Register map                                                              */
/* -------------------------------------------------------------------------- */

#define MPU6050_REG_SMPLRT_DIV    (0x19U)
#define MPU6050_REG_CONFIG        (0x1AU)
#define MPU6050_REG_GYRO_CONFIG   (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG  (0x1CU)

#define MPU6050_REG_ACCEL_XOUT_H  (0x3BU)
#define MPU6050_REG_TEMP_OUT_H    (0x41U)
#define MPU6050_REG_GYRO_XOUT_H   (0x43U)

#define MPU6050_REG_PWR_MGMT_1    (0x6BU)
#define MPU6050_REG_PWR_MGMT_2    (0x6CU)
#define MPU6050_REG_WHO_AM_I      (0x75U)

/* -------------------------------------------------------------------------- */
/*  Configuration                                                             */
/* -------------------------------------------------------------------------- */

/** @brief Accelerometer full scale. Wider range, coarser resolution. */
typedef enum {
    MPU6050_ACCEL_FS_2G = 0U,
    MPU6050_ACCEL_FS_4G,
    MPU6050_ACCEL_FS_8G,
    MPU6050_ACCEL_FS_16G,
} MPU6050_AccelRange_t;

/**
 * @brief Digital low-pass filter bandwidth (CONFIG register, DLPF_CFG).
 *
 * NOT optional for anything that integrates. At DLPF_OFF the accelerometer has
 * 260 Hz of bandwidth; sampling that at 100 Hz aliases every bit of wideband
 * noise straight into the integrator, and an integrator has no way to shed it -
 * the velocity estimate performs a random walk away from truth within seconds.
 *
 * Pick a bandwidth below half the sample rate. At 100 Hz that means DLPF_44HZ.
 */
typedef enum {
    MPU6050_DLPF_OFF   = 0U,  /**< accel 260 Hz, gyro 256 Hz, Fs 8 kHz      */
    MPU6050_DLPF_184HZ = 1U,
    MPU6050_DLPF_94HZ  = 2U,
    MPU6050_DLPF_44HZ  = 3U,  /**< the sane default for a 100 Hz loop       */
    MPU6050_DLPF_21HZ  = 4U,
    MPU6050_DLPF_10HZ  = 5U,
    MPU6050_DLPF_5HZ   = 6U,
} MPU6050_Dlpf_t;

/** @brief Gyroscope full scale, in degrees per second. */
typedef enum {
    MPU6050_GYRO_FS_250DPS = 0U,
    MPU6050_GYRO_FS_500DPS,
    MPU6050_GYRO_FS_1000DPS,
    MPU6050_GYRO_FS_2000DPS,
} MPU6050_GyroRange_t;

/* -------------------------------------------------------------------------- */
/*  Sample                                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief One complete sample, in raw sensor counts.
 *
 * Field order matches the register order so the whole struct comes off the bus
 * in a single burst — accel, then temperature, then gyro.
 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050_Raw_t;

/* -------------------------------------------------------------------------- */
/*  API                                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Reset the device, wake it, verify its identity and apply the ranges
 *        configured in vehicle_config.h.
 * @return DRV_OK, DRV_ERROR if WHO_AM_I does not match, or an I2C error.
 *
 * Blocks for ~110 ms: the power-on state is undefined and the internal reset
 * needs its settling time before any further register access is meaningful.
 *
 * A DRV_ERROR here is worth acting on — it distinguishes "sensor present but
 * misbehaving" from "SDA floating", which a bare read cannot.
 */
DRV_Status MPU6050_Init(void);

/**
 * @brief Re-establish contact with a device that stopped answering, WITHOUT
 *        the full reset sequence.
 * @return DRV_OK if it responds and has been reconfigured.
 *
 * MPU6050_Init() blocks for over 100 ms because a device reset genuinely needs
 * that long to settle. That is fine once at boot and completely unacceptable
 * from a running control loop, so this does the cheap part only: verify the
 * part is answering, make sure it is awake, and re-apply the full-scale
 * ranges. A few hundred microseconds.
 *
 * Covers both realistic causes of a dropout - a wedged bus (the caller
 * re-inits I2C first, which clocks it free) and a device that browned out and
 * came back in sleep mode with default ranges.
 *
 * Does NOT re-measure the bias: the caller owns that, and by the time a
 * dropout happens the car is usually moving, which is the worst possible
 * moment to recalibrate a gyro.
 */
DRV_Status MPU6050_Resume(void);

/** @brief Read WHO_AM_I. Expect MPU6050_WHO_AM_I_VALUE. */
DRV_Status MPU6050_ReadWhoAmI(uint8_t *id);

/** @brief Set the accelerometer full scale and remember it. */
DRV_Status MPU6050_SetAccelRange(MPU6050_AccelRange_t range);

/** @brief Set the gyroscope full scale and remember it. */
DRV_Status MPU6050_SetGyroRange(MPU6050_GyroRange_t range);

/** @brief The range currently configured — needed to scale raw counts. */
MPU6050_AccelRange_t MPU6050_GetAccelRange(void);
/** @brief The range currently configured — needed to scale raw counts. */
MPU6050_GyroRange_t  MPU6050_GetGyroRange(void);

/** @brief Read the three accelerometer axes. */
DRV_Status MPU6050_ReadAccel(int16_t *x, int16_t *y, int16_t *z);

/** @brief Read the three gyroscope axes. */
DRV_Status MPU6050_ReadGyro(int16_t *x, int16_t *y, int16_t *z);

/** @brief Read the die temperature, in raw counts. */
DRV_Status MPU6050_ReadTemperature(int16_t *temperature);

/**
 * @brief Read accel, temperature and gyro in ONE bus transaction.
 *
 * Prefer this over the individual reads whenever more than one axis is wanted:
 * the fourteen output registers are contiguous and auto-increment, so a single
 * burst is both faster and — more importantly — coherent, with every axis
 * sampled at the same instant.
 */
DRV_Status MPU6050_ReadRaw(MPU6050_Raw_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H_ */

#ifndef DRV_I2C_H
#define DRV_I2C_H

#include "stm32f10x.h"
#include "types.h"
#include "error.h"

/* I2C Peripheral */
typedef enum
{
    I2C_1 = 0,
    I2C_2
} I2C_t;

/* I2C Speed */
typedef enum
{
    I2C_SPEED_100KHZ = 100000U,
    I2C_SPEED_400KHZ = 400000U
} I2C_Speed_t;

/* I2C Address Direction */
typedef enum
{
    I2C_WRITE = 0,
    I2C_READ
} I2C_Direction_t;

/* I2C Initialization */
Status_t DRV_I2C_Init(
    I2C_t i2c,
    I2C_Speed_t speed);

/* I2C Register Write */
Status_t DRV_I2C_Write(
    I2C_t i2c,
    uint8_t address,
    uint16_t reg,
    const uint8_t *data,
    uint16_t length);

/* I2C Register Read */
Status_t DRV_I2C_Read(
    I2C_t i2c,
    uint8_t address,
    uint16_t reg,
    uint8_t *data,
    uint16_t length);

/* Reset I2C peripheral */
Status_t DRV_I2C_Reset(I2C_t i2c);

#endif /* DRV_I2C_H */
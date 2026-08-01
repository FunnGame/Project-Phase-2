#include "vl53l1_platform.h"

#include "drv_i2c.h"
#include "drv_timer.h"

#include <stdint.h>

/* ============================================================
 * Configuration
 * ============================================================ */

#define VL53L1_I2C_INSTANCE       I2C_1
#define VL53L1_I2C_SPEED          I2C_SPEED_400KHZ

/*
 * VL53L1X default 7-bit I2C address = 0x29.
 *
 * DRV_I2C expects the 7-bit address and performs
 * the R/W bit handling internally.
 */
#define VL53L1_DEFAULT_I2C_ADDR   (0x29U)


/* ============================================================
 * Private Functions
 * ============================================================ */

static int8_t VL53L1_StatusToPlatform(Status_t status)
{
    if(status == STATUS_OK)
    {
        return 0;
    }

    return -1;
}


/* ============================================================
 * Write Multiple Bytes
 * ============================================================ */

int8_t VL53L1_WriteMulti(
    uint16_t dev,
    uint16_t index,
    uint8_t *pdata,
    uint32_t count)
{
    Status_t status;

    if((pdata == (uint8_t *)0) && (count > 0U))
    {
        return -1;
    }

    if(count > 65535U)
    {
        return -1;
    }

    status = DRV_I2C_Write(
        VL53L1_I2C_INSTANCE,
        (uint8_t)dev,
        index,
        pdata,
        (uint16_t)count);

    return VL53L1_StatusToPlatform(status);
}


/* ============================================================
 * Read Multiple Bytes
 * ============================================================ */

int8_t VL53L1_ReadMulti(
    uint16_t dev,
    uint16_t index,
    uint8_t *pdata,
    uint32_t count)
{
    Status_t status;

    if((pdata == (uint8_t *)0) || (count == 0U))
    {
        return -1;
    }

    if(count > 65535U)
    {
        return -1;
    }

    status = DRV_I2C_Read(
        VL53L1_I2C_INSTANCE,
        (uint8_t)dev,
        index,
        pdata,
        (uint16_t)count);

    return VL53L1_StatusToPlatform(status);
}


/* ============================================================
 * Write Byte
 * ============================================================ */

int8_t VL53L1_WrByte(
    uint16_t dev,
    uint16_t index,
    uint8_t data)
{
    return VL53L1_WriteMulti(
        dev,
        index,
        &data,
        1U);
}


/* ============================================================
 * Write Word
 * ============================================================ */

int8_t VL53L1_WrWord(
    uint16_t dev,
    uint16_t index,
    uint16_t data)
{
    uint8_t buffer[2];

    buffer[0] = (uint8_t)(data >> 8U);
    buffer[1] = (uint8_t)(data & 0xFFU);

    return VL53L1_WriteMulti(
        dev,
        index,
        buffer,
        2U);
}


/* ============================================================
 * Write Double Word
 * ============================================================ */

int8_t VL53L1_WrDWord(
    uint16_t dev,
    uint16_t index,
    uint32_t data)
{
    uint8_t buffer[4];

    buffer[0] = (uint8_t)(data >> 24U);
    buffer[1] = (uint8_t)(data >> 16U);
    buffer[2] = (uint8_t)(data >> 8U);
    buffer[3] = (uint8_t)(data & 0xFFU);

    return VL53L1_WriteMulti(
        dev,
        index,
        buffer,
        4U);
}


/* ============================================================
 * Read Byte
 * ============================================================ */

int8_t VL53L1_RdByte(
    uint16_t dev,
    uint16_t index,
    uint8_t *pdata)
{
    if(pdata == (uint8_t *)0)
    {
        return -1;
    }

    return VL53L1_ReadMulti(
        dev,
        index,
        pdata,
        1U);
}


/* ============================================================
 * Read Word
 * ============================================================ */

int8_t VL53L1_RdWord(
    uint16_t dev,
    uint16_t index,
    uint16_t *pdata)
{
    uint8_t buffer[2];
    int8_t status;

    if(pdata == (uint16_t *)0)
    {
        return -1;
    }

    status = VL53L1_ReadMulti(
        dev,
        index,
        buffer,
        2U);

    if(status != 0)
    {
        return status;
    }

    *pdata = ((uint16_t)buffer[0] << 8U) |
             ((uint16_t)buffer[1]);

    return 0;
}


/* ============================================================
 * Read Double Word
 * ============================================================ */

int8_t VL53L1_RdDWord(
    uint16_t dev,
    uint16_t index,
    uint32_t *pdata)
{
    uint8_t buffer[4];
    int8_t status;

    if(pdata == (uint32_t *)0)
    {
        return -1;
    }

    status = VL53L1_ReadMulti(
        dev,
        index,
        buffer,
        4U);

    if(status != 0)
    {
        return status;
    }

    *pdata = ((uint32_t)buffer[0] << 24U) |
             ((uint32_t)buffer[1] << 16U) |
             ((uint32_t)buffer[2] << 8U)  |
             ((uint32_t)buffer[3]);

    return 0;
}


/* ============================================================
 * Delay
 * ============================================================ */

int8_t VL53L1_WaitMs(
    uint16_t dev,
    int32_t wait_ms)
{
    (void)dev;

    if(wait_ms < 0)
    {
        return -1;
    }

    DRV_DelayMs((uint32)wait_ms);

    return 0;
}
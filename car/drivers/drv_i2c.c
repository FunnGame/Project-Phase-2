/**
 ******************************************************************************
 * @file    drv_i2c.c
 * @brief   I2C master driver implementation (STM32F103).
 ******************************************************************************
 */
#include "drv_i2c.h"
#include "drv_clock.h"
#include "drv_gpio.h"

/* Busy-wait budget for a single flag (iterations, not wall-clock). Generous:
 * a byte at 100 kHz is ~90 us, and the CPU may be at 8 MHz HSI. */
#define I2C_WAIT_LOOPS   (200000u)

/* -------------------------------------------------------------------------- */
/*  Low-level helpers                                                          */
/* -------------------------------------------------------------------------- */

/* Wait until (SR1 & mask) == mask, or time out. Also bails on an ACK failure
 * (AF), which is how a missing/held device shows up. */
static DRV_Status i2c_wait_sr1(I2C_TypeDef *i2c, uint32_t mask)
{
    for (uint32_t i = 0; i < I2C_WAIT_LOOPS; ++i) {
        const uint32_t sr1 = i2c->SR1;
        if (sr1 & I2C_SR1_AF) {
            i2c->SR1 = (uint16_t)~I2C_SR1_AF;   /* clear and report */
            return DRV_ERROR;
        }
        if ((sr1 & mask) == mask) {
            return DRV_OK;
        }
    }
    return DRV_TIMEOUT;
}

static DRV_Status i2c_start(I2C_TypeDef *i2c)
{
    DRV_SET_BITS(i2c->CR1, I2C_CR1_START);
    return i2c_wait_sr1(i2c, I2C_SR1_SB);
}

static void i2c_stop(I2C_TypeDef *i2c)
{
    DRV_SET_BITS(i2c->CR1, I2C_CR1_STOP);
}

/* Send the addressing byte and clear ADDR (read SR1 then SR2). */
static DRV_Status i2c_send_addr(I2C_TypeDef *i2c, uint8_t addr7, bool read)
{
    i2c->DR = (uint8_t)((addr7 << 1) | (read ? 1u : 0u));

    const DRV_Status st = i2c_wait_sr1(i2c, I2C_SR1_ADDR);
    if (st != DRV_OK) {
        return st;
    }
    (void)i2c->SR1;
    (void)i2c->SR2;                          /* ADDR cleared by SR1+SR2 read */
    return DRV_OK;
}

static DRV_Status i2c_write_byte(I2C_TypeDef *i2c, uint8_t b)
{
    const DRV_Status st = i2c_wait_sr1(i2c, I2C_SR1_TXE);
    if (st != DRV_OK) {
        return st;
    }
    i2c->DR = b;
    return DRV_OK;
}

/* Wait for the bus to be free (BUSY clear) before starting a transaction. */
static DRV_Status i2c_wait_idle(I2C_TypeDef *i2c)
{
    for (uint32_t i = 0; i < I2C_WAIT_LOOPS; ++i) {
        if ((i2c->SR2 & I2C_SR2_BUSY) == 0u) {
            return DRV_OK;
        }
    }
    return DRV_TIMEOUT;
}

/* -------------------------------------------------------------------------- */
/*  Init                                                                       */
/* -------------------------------------------------------------------------- */

DRV_Status DRV_I2C_Init(const I2C_Config *cfg)
{
    if (cfg == NULL || cfg->i2c == NULL || cfg->speed_hz == 0u) {
        return DRV_INVALID_PARAM;
    }

    uint8_t scl_pin;
    uint8_t sda_pin;

    if (cfg->i2c == I2C1) {
        DRV_SET_BITS(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
        if (cfg->remap) {
            DRV_SET_BITS(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
            DRV_SET_BITS(AFIO->MAPR, AFIO_MAPR_I2C1_REMAP);
            scl_pin = 8u; sda_pin = 9u;      /* PB8 / PB9 */
        } else {
            scl_pin = 6u; sda_pin = 7u;      /* PB6 / PB7 */
        }
    } else if (cfg->i2c == I2C2) {
        DRV_SET_BITS(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);
        scl_pin = 10u; sda_pin = 11u;        /* PB10 / PB11 */
    } else {
        return DRV_INVALID_PARAM;
    }

    /* SCL/SDA: alternate-function open-drain (an I2C bus is wire-AND). */
    DRV_GPIO_Init(GPIOB, scl_pin, GPIO_MODE_OUT_50M, GPIO_CNF_OUT_AF_OD,
                  GPIO_PULL_NONE);
    DRV_GPIO_Init(GPIOB, sda_pin, GPIO_MODE_OUT_50M, GPIO_CNF_OUT_AF_OD,
                  GPIO_PULL_NONE);

    I2C_TypeDef *i2c = cfg->i2c;

    /* Timing from the real APB1 clock, not a hardcoded 36 MHz. */
    const uint32_t pclk1 = DRV_Clock_GetPCLK1();
    const uint32_t freq_mhz = pclk1 / 1000000u;
    if (freq_mhz < 2u || freq_mhz > 50u) {
        return DRV_UNSUPPORTED;
    }

    DRV_CLEAR_BITS(i2c->CR1, I2C_CR1_PE);    /* configure with PE off */

    DRV_MODIFY(i2c->CR2, I2C_CR2_FREQ, freq_mhz);

    if (cfg->speed_hz <= 100000u) {
        /* Standard mode: CCR = PCLK1 / (2 * SCL). */
        const uint32_t ccr = pclk1 / (2u * cfg->speed_hz);
        i2c->CCR   = (uint16_t)(ccr & I2C_CCR_CCR);
        i2c->TRISE = (uint16_t)(freq_mhz + 1u);
    } else {
        /* Fast mode, Tlow/Thigh = 2: CCR = PCLK1 / (3 * SCL). */
        const uint32_t ccr = pclk1 / (3u * cfg->speed_hz);
        i2c->CCR   = (uint16_t)(I2C_CCR_FS | (ccr & I2C_CCR_CCR));
        i2c->TRISE = (uint16_t)(((freq_mhz * 300u) / 1000u) + 1u);
    }

    DRV_SET_BITS(i2c->CR1, I2C_CR1_ACK);
    DRV_SET_BITS(i2c->CR1, I2C_CR1_PE);
    return DRV_OK;
}

/* -------------------------------------------------------------------------- */
/*  Register write / read                                                      */
/* -------------------------------------------------------------------------- */

DRV_Status DRV_I2C_MemWrite(I2C_TypeDef *i2c, uint8_t addr7, uint8_t reg,
                            const uint8_t *data, uint16_t len)
{
    if (i2c == NULL || (data == NULL && len > 0u)) {
        return DRV_INVALID_PARAM;
    }

    DRV_Status st = i2c_wait_idle(i2c);
    if (st != DRV_OK) { return st; }

    st = i2c_start(i2c);
    if (st != DRV_OK) { i2c_stop(i2c); return st; }

    st = i2c_send_addr(i2c, addr7, false);
    if (st != DRV_OK) { i2c_stop(i2c); return st; }

    st = i2c_write_byte(i2c, reg);
    if (st != DRV_OK) { i2c_stop(i2c); return st; }

    for (uint16_t i = 0; i < len; ++i) {
        st = i2c_write_byte(i2c, data[i]);
        if (st != DRV_OK) { i2c_stop(i2c); return st; }
    }

    /* Let the last byte finish shifting out (BTF) before STOP. */
    st = i2c_wait_sr1(i2c, I2C_SR1_BTF);
    i2c_stop(i2c);
    return st;
}

DRV_Status DRV_I2C_MemRead(I2C_TypeDef *i2c, uint8_t addr7, uint8_t reg,
                           uint8_t *data, uint16_t len)
{
    if (i2c == NULL || data == NULL || len == 0u) {
        return DRV_INVALID_PARAM;
    }

    DRV_Status st = i2c_wait_idle(i2c);
    if (st != DRV_OK) { return st; }

    /* Phase 1: write the register index. */
    st = i2c_start(i2c);
    if (st != DRV_OK) { i2c_stop(i2c); return st; }

    st = i2c_send_addr(i2c, addr7, false);
    if (st != DRV_OK) { i2c_stop(i2c); return st; }

    st = i2c_write_byte(i2c, reg);
    if (st != DRV_OK) { i2c_stop(i2c); return st; }

    st = i2c_wait_sr1(i2c, I2C_SR1_BTF);
    if (st != DRV_OK) { i2c_stop(i2c); return st; }

    /* Phase 2: repeated START, then read. The address byte is sent here rather
     * than via i2c_send_addr() because the single-byte case needs ACK cleared
     * *before* ADDR is cleared (RM0008 master-receiver EV6_1). */
    DRV_SET_BITS(i2c->CR1, I2C_CR1_ACK);

    st = i2c_start(i2c);
    if (st != DRV_OK) { i2c_stop(i2c); return st; }

    i2c->DR = (uint8_t)((addr7 << 1) | 1u);
    st = i2c_wait_sr1(i2c, I2C_SR1_ADDR);
    if (st != DRV_OK) { i2c_stop(i2c); return st; }

    if (len == 1u) {
        DRV_CLEAR_BITS(i2c->CR1, I2C_CR1_ACK);   /* NACK before ADDR clear */
        (void)i2c->SR1;
        (void)i2c->SR2;                          /* clear ADDR */
        i2c_stop(i2c);

        st = i2c_wait_sr1(i2c, I2C_SR1_RXNE);
        if (st != DRV_OK) { return st; }
        data[0] = (uint8_t)i2c->DR;
    } else {
        (void)i2c->SR1;
        (void)i2c->SR2;                          /* clear ADDR */
        for (uint16_t i = 0; i < len; ++i) {
            if (i == (uint16_t)(len - 1u)) {
                DRV_CLEAR_BITS(i2c->CR1, I2C_CR1_ACK);   /* NACK the last */
                i2c_stop(i2c);
            }
            st = i2c_wait_sr1(i2c, I2C_SR1_RXNE);
            if (st != DRV_OK) { i2c_stop(i2c); return st; }
            data[i] = (uint8_t)i2c->DR;
        }
    }

    /* Leave ACK enabled for the next transaction. */
    DRV_SET_BITS(i2c->CR1, I2C_CR1_ACK);
    return DRV_OK;
}

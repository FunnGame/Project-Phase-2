/**
 ******************************************************************************
 * @file    spi.c
 * @brief   SPI master driver implementation (STM32F446RE).
 ******************************************************************************
 */
#include "spi.h"
#include "rcc.h"

/* Busy-wait budget for a single frame's TXE/RXNE flag. */
#define SPI_FLAG_TIMEOUT   0x00100000UL

static bool spi_clock_enable(SPI_TypeDef *spi)
{
    if      (spi == SPI1) rcc_apb2_enable(RCC_APB2ENR_SPI1EN);
    else if (spi == SPI2) rcc_apb1_enable(RCC_APB1ENR_SPI2EN);
    else if (spi == SPI3) rcc_apb1_enable(RCC_APB1ENR_SPI3EN);
    else if (spi == SPI4) rcc_apb2_enable(RCC_APB2ENR_SPI4EN);
    else return false;
    return true;
}

drv_status_t spi_init(SPI_TypeDef *spi, const spi_config_t *cfg)
{
    if (spi == NULL || cfg == NULL) {
        return DRV_INVALID_PARAM;
    }
    if (!spi_clock_enable(spi)) {
        return DRV_INVALID_PARAM;
    }

    spi_disable(spi);

    uint32_t cr1 = 0U;

    /* Master with software slave management: SSM + SSI keep NSS high so the
     * peripheral does not drop out of master mode. Caller drives CS via GPIO. */
    cr1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;

    if (cfg->mode == SPI_MODE1 || cfg->mode == SPI_MODE3) cr1 |= SPI_CR1_CPHA;
    if (cfg->mode == SPI_MODE2 || cfg->mode == SPI_MODE3) cr1 |= SPI_CR1_CPOL;

    cr1 |= ((uint32_t)cfg->baud << SPI_CR1_BR_Pos) & SPI_CR1_BR;

    if (cfg->bit_order == SPI_BIT_LSB_FIRST) cr1 |= SPI_CR1_LSBFIRST;
    if (cfg->datasize == SPI_DATASIZE_16BIT) cr1 |= SPI_CR1_DFF;

    spi->CR1 = cr1;
    spi->CR2 = 0U;

    spi_enable(spi);
    return DRV_OK;
}

void spi_enable(SPI_TypeDef *spi)  { DRV_SET_BITS(spi->CR1, SPI_CR1_SPE); }

void spi_disable(SPI_TypeDef *spi)
{
    /* Wait for any in-flight frame to complete before disabling. */
    (void)drv_wait_flag(&spi->SR, SPI_SR_BSY, 0U, SPI_FLAG_TIMEOUT);
    DRV_CLEAR_BITS(spi->CR1, SPI_CR1_SPE);
}

uint8_t spi_transfer_byte(SPI_TypeDef *spi, uint8_t tx)
{
    (void)drv_wait_flag(&spi->SR, SPI_SR_TXE, SPI_SR_TXE, SPI_FLAG_TIMEOUT);
    *(volatile uint8_t *)&spi->DR = tx;

    (void)drv_wait_flag(&spi->SR, SPI_SR_RXNE, SPI_SR_RXNE, SPI_FLAG_TIMEOUT);
    return *(volatile uint8_t *)&spi->DR;
}

drv_status_t spi_transfer(SPI_TypeDef *spi, const uint8_t *tx, uint8_t *rx,
                          size_t len)
{
    if (spi == NULL || len == 0U) {
        return DRV_INVALID_PARAM;
    }

    for (size_t i = 0; i < len; ++i) {
        const uint8_t out = (tx != NULL) ? tx[i] : 0x00U;

        if (drv_wait_flag(&spi->SR, SPI_SR_TXE, SPI_SR_TXE,
                          SPI_FLAG_TIMEOUT) != DRV_OK) {
            return DRV_TIMEOUT;
        }
        *(volatile uint8_t *)&spi->DR = out;

        if (drv_wait_flag(&spi->SR, SPI_SR_RXNE, SPI_SR_RXNE,
                          SPI_FLAG_TIMEOUT) != DRV_OK) {
            return DRV_TIMEOUT;
        }
        const uint8_t in = *(volatile uint8_t *)&spi->DR;
        if (rx != NULL) {
            rx[i] = in;
        }
    }

    /* Ensure the bus is idle before returning so the caller may raise CS. */
    if (drv_wait_flag(&spi->SR, SPI_SR_BSY, 0U, SPI_FLAG_TIMEOUT) != DRV_OK) {
        return DRV_TIMEOUT;
    }
    return DRV_OK;
}

drv_status_t spi_write(SPI_TypeDef *spi, const uint8_t *tx, size_t len)
{
    return spi_transfer(spi, tx, NULL, len);
}

drv_status_t spi_read(SPI_TypeDef *spi, uint8_t *rx, size_t len)
{
    return spi_transfer(spi, NULL, rx, len);
}

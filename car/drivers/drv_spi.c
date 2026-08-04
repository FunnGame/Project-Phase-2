/**
 ******************************************************************************
 * @file    drv_spi.c
 * @brief   SPI master driver implementation (STM32F103).
 ******************************************************************************
 */
#include "drv_spi.h"

/* Busy-wait budget for a single frame's status flag. */
#define SPI_FLAG_TIMEOUT   0x00100000UL

static bool spi_clock_enable(SPI_TypeDef *spi)
{
    if      (spi == SPI1) DRV_SET_BITS(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
    else if (spi == SPI2) DRV_SET_BITS(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);
    else return false;
    return true;
}

DRV_Status DRV_SPI_Init(const SPI_Config *cfg)
{
    if (cfg == NULL || cfg->spi == NULL || cfg->port == NULL ||
        cfg->sck_pin > 15u || cfg->miso_pin > 15u || cfg->mosi_pin > 15u) {
        return DRV_INVALID_PARAM;
    }
    if (!spi_clock_enable(cfg->spi)) {
        return DRV_INVALID_PARAM;
    }

    /* Configure exactly the three bus pins — neighbouring pins are untouched.
     * SCK and MOSI are driven by the peripheral (AF push-pull); MISO is an
     * input. */
    DRV_GPIO_InitAF(cfg->port, cfg->sck_pin);
    DRV_GPIO_InitAF(cfg->port, cfg->mosi_pin);
    DRV_GPIO_InitInput(cfg->port, cfg->miso_pin, GPIO_PULL_NONE);

    SPI_TypeDef *spi = cfg->spi;

    DRV_CLEAR_BITS(spi->CR1, SPI_CR1_SPE);   /* disable while configuring */

    uint32_t cr1 = 0u;

    /* Master with software slave management: SSM + SSI hold NSS high
     * internally so the peripheral cannot drop out of master mode. */
    cr1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;

    if (cfg->mode == SPI_MODE1 || cfg->mode == SPI_MODE3) cr1 |= SPI_CR1_CPHA;
    if (cfg->mode == SPI_MODE2 || cfg->mode == SPI_MODE3) cr1 |= SPI_CR1_CPOL;

    cr1 |= ((uint32_t)cfg->baud << SPI_CR1_BR_Pos) & SPI_CR1_BR;

    if (cfg->bit_order == SPI_LSB_FIRST) cr1 |= SPI_CR1_LSBFIRST;

    spi->CR1 = cr1;
    spi->CR2 = 0u;
    DRV_SET_BITS(spi->CR1, SPI_CR1_SPE);

    return DRV_OK;
}

uint8_t DRV_SPI_Transfer(SPI_TypeDef *spi, uint8_t data)
{
    (void)DRV_WaitFlag(&spi->SR, SPI_SR_TXE, SPI_SR_TXE, SPI_FLAG_TIMEOUT);
    *(volatile uint8_t *)&spi->DR = data;

    (void)DRV_WaitFlag(&spi->SR, SPI_SR_RXNE, SPI_SR_RXNE, SPI_FLAG_TIMEOUT);
    return *(volatile uint8_t *)&spi->DR;
}

DRV_Status DRV_SPI_TransferBuffer(SPI_TypeDef *spi, const uint8_t *tx,
                                  uint8_t *rx, size_t len)
{
    if (spi == NULL || len == 0u) {
        return DRV_INVALID_PARAM;
    }

    for (size_t i = 0; i < len; ++i) {
        const uint8_t out = (tx != NULL) ? tx[i] : 0x00u;

        if (DRV_WaitFlag(&spi->SR, SPI_SR_TXE, SPI_SR_TXE,
                         SPI_FLAG_TIMEOUT) != DRV_OK) {
            return DRV_TIMEOUT;
        }
        *(volatile uint8_t *)&spi->DR = out;

        if (DRV_WaitFlag(&spi->SR, SPI_SR_RXNE, SPI_SR_RXNE,
                         SPI_FLAG_TIMEOUT) != DRV_OK) {
            return DRV_TIMEOUT;
        }
        const uint8_t in = *(volatile uint8_t *)&spi->DR;
        if (rx != NULL) {
            rx[i] = in;
        }
    }

    /* Let the shift register drain so the caller may safely raise CS. */
    if (DRV_WaitFlag(&spi->SR, SPI_SR_BSY, 0u, SPI_FLAG_TIMEOUT) != DRV_OK) {
        return DRV_TIMEOUT;
    }
    return DRV_OK;
}

DRV_Status DRV_SPI_InitCS(GPIO_TypeDef *port, uint8_t pin)
{
    /* Idle high = deselected. */
    return DRV_GPIO_InitOutput(port, pin, GPIO_HIGH);
}

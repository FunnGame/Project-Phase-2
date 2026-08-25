/**
 ******************************************************************************
 * @file    nrf24.c
 * @brief   Platform-agnostic nRF24L01+ driver implementation.
 ******************************************************************************
 */
#include "nrf24.h"

#include <string.h>

/* ---- SPI commands -------------------------------------------------------- */
#define CMD_R_REGISTER      0x00u  /* | reg */
#define CMD_W_REGISTER      0x20u  /* | reg */
#define CMD_R_RX_PAYLOAD    0x61u
#define CMD_W_TX_PAYLOAD    0xA0u
#define CMD_FLUSH_TX        0xE1u
#define CMD_FLUSH_RX        0xE2u
#define CMD_R_RX_PL_WID     0x60u
#define CMD_W_ACK_PAYLOAD   0xA8u  /* | pipe */
#define CMD_NOP             0xFFu

/* ---- Registers ----------------------------------------------------------- */
#define REG_CONFIG          0x00u
#define REG_EN_AA           0x01u
#define REG_EN_RXADDR       0x02u
#define REG_SETUP_AW        0x03u
#define REG_SETUP_RETR      0x04u
#define REG_RF_CH           0x05u
#define REG_RF_SETUP        0x06u
#define REG_STATUS          0x07u
#define REG_RX_ADDR_P0      0x0Au
#define REG_TX_ADDR         0x10u
#define REG_RX_PW_P0        0x11u
#define REG_FIFO_STATUS     0x17u
#define REG_DYNPD           0x1Cu
#define REG_FEATURE         0x1Du

/* ---- FEATURE bits -------------------------------------------------------- */
#define FEATURE_EN_DPL      0x04u
#define FEATURE_EN_ACK_PAY  0x02u

/* ---- CONFIG bits --------------------------------------------------------- */
#define CONFIG_EN_CRC       0x08u
#define CONFIG_CRCO         0x04u  /* 1 = 2-byte CRC */
#define CONFIG_PWR_UP       0x02u
#define CONFIG_PRIM_RX      0x01u

/* ---- STATUS bits --------------------------------------------------------- */
#define STATUS_RX_DR        0x40u
#define STATUS_TX_DS        0x20u
#define STATUS_MAX_RT       0x10u

/* ---- RF_SETUP bits ------------------------------------------------------- */
#define RF_SETUP_RF_DR_LOW  0x20u
#define RF_SETUP_RF_DR_HIGH 0x08u

/* Wait budget for a transmit result: ~50 us * 400 = 20 ms (covers 15 retries
 * at ARD 500 us). */
#define TX_WAIT_STEPS       400u
#define TX_WAIT_US          50u

/* -------------------------------------------------------------------------- */
/*  Low-level register access                                                 */
/* -------------------------------------------------------------------------- */

static uint8_t nrf24_status(const nrf24_t *dev)
{
    uint8_t tx = CMD_NOP, rx = 0;
    dev->hal->csn_write(false);
    dev->hal->spi_transfer(&tx, &rx, 1);
    dev->hal->csn_write(true);
    return rx;
}

static void nrf24_write_reg(const nrf24_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(CMD_W_REGISTER | reg), val };
    dev->hal->csn_write(false);
    dev->hal->spi_transfer(tx, NULL, 2);
    dev->hal->csn_write(true);
}

static uint8_t nrf24_read_reg(const nrf24_t *dev, uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(CMD_R_REGISTER | reg), CMD_NOP };
    uint8_t rx[2] = { 0, 0 };
    dev->hal->csn_write(false);
    dev->hal->spi_transfer(tx, rx, 2);
    dev->hal->csn_write(true);
    return rx[1];
}

static void nrf24_write_reg_buf(const nrf24_t *dev, uint8_t reg,
                                const uint8_t *buf, uint8_t len)
{
    uint8_t tx[1 + NRF24_ADDR_WIDTH];
    tx[0] = (uint8_t)(CMD_W_REGISTER | reg);
    memcpy(&tx[1], buf, len);
    dev->hal->csn_write(false);
    dev->hal->spi_transfer(tx, NULL, (size_t)len + 1u);
    dev->hal->csn_write(true);
}

static void nrf24_command(const nrf24_t *dev, uint8_t cmd)
{
    dev->hal->csn_write(false);
    dev->hal->spi_transfer(&cmd, NULL, 1);
    dev->hal->csn_write(true);
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

bool nrf24_init(nrf24_t *dev, const nrf24_hal_t *hal, const nrf24_config_t *cfg)
{
    if (dev == NULL || hal == NULL || cfg == NULL ||
        cfg->payload_width == 0u || cfg->payload_width > NRF24_MAX_PAYLOAD) {
        return false;
    }

    dev->hal = hal;
    dev->payload_width = cfg->payload_width;

    hal->ce_write(false);
    hal->csn_write(true);

    /* Power-on reset (Tpor) is 100 ms from VDD rising. An MCU running at tens
     * or hundreds of MHz reaches this function within a few ms of power-up, so
     * without this wait the first register writes land while the radio is still
     * in reset and are silently ignored — which then shows up as a failed
     * presence check even though the wiring is perfect. */
    hal->delay_us(100000u);

    /* Power up, 2-byte CRC, PTX to start. */
    nrf24_write_reg(dev, REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP);
    hal->delay_us(5000u);  /* power down -> standby (Tpd2stby, 1.5 ms max) */

    /* RF channel. */
    nrf24_write_reg(dev, REG_RF_CH, (uint8_t)(cfg->channel & 0x7Fu));

    /* Data rate + power in RF_SETUP. */
    uint8_t rf = (uint8_t)((cfg->power & 0x3u) << 1);
    switch (cfg->data_rate) {
        case NRF24_DR_2MBPS:   rf |= RF_SETUP_RF_DR_HIGH; break;
        case NRF24_DR_250KBPS: rf |= RF_SETUP_RF_DR_LOW;  break;
        case NRF24_DR_1MBPS:   default: break;
    }
    nrf24_write_reg(dev, REG_RF_SETUP, rf);

    /* 5-byte address on TX and RX pipe 0 (pipe 0 also receives the auto-ACK). */
    nrf24_write_reg(dev, REG_SETUP_AW, 0x03u);
    nrf24_write_reg_buf(dev, REG_TX_ADDR, cfg->address, NRF24_ADDR_WIDTH);
    nrf24_write_reg_buf(dev, REG_RX_ADDR_P0, cfg->address, NRF24_ADDR_WIDTH);

    /* Auto-ack + retransmit, or plain packets. */
    if (cfg->auto_ack) {
        nrf24_write_reg(dev, REG_EN_AA, 0x01u);        /* pipe 0 */
        nrf24_write_reg(dev, REG_SETUP_RETR, 0x1Fu);   /* 500 us, 15 retries */
    } else {
        nrf24_write_reg(dev, REG_EN_AA, 0x00u);
        nrf24_write_reg(dev, REG_SETUP_RETR, 0x00u);
    }

    /* Fixed payload width on pipe 0; no dynamic payloads. */
    nrf24_write_reg(dev, REG_EN_RXADDR, 0x01u);
    nrf24_write_reg(dev, REG_RX_PW_P0, dev->payload_width);
    /* Dynamic payload length + ACK payloads, or neither.
     *
     * The nRF24 has no fixed-width ACK payload mode: EN_ACK_PAY requires EN_DPL
     * and DYNPD for the pipe. Both ends must agree - a receiver attaching
     * payloads to a transmitter that has not enabled them leaves data stuck in
     * the transmitter's RX FIFO until it blocks. */
    dev->ack_payload = (cfg->ack_payload && cfg->auto_ack);
    if (dev->ack_payload) {
        nrf24_write_reg(dev, REG_FEATURE, FEATURE_EN_DPL | FEATURE_EN_ACK_PAY);
        nrf24_write_reg(dev, REG_DYNPD, 0x01u);        /* pipe 0 */
    } else {
        nrf24_write_reg(dev, REG_DYNPD, 0x00u);
        nrf24_write_reg(dev, REG_FEATURE, 0x00u);
    }

    /* Clear FIFOs and any latched IRQ flags. */
    nrf24_command(dev, CMD_FLUSH_TX);
    nrf24_command(dev, CMD_FLUSH_RX);
    nrf24_write_reg(dev, REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);

    /* Presence check: RF_CH must read back what we wrote.
     *
     * Retried rather than sampled once — if the radio was still settling when
     * the first writes went out, re-writing after a short pause recovers
     * instead of failing the whole init. A module that is genuinely absent or
     * miswired still fails every attempt, so this does not mask real faults. */
    const uint8_t expect = (uint8_t)(cfg->channel & 0x7Fu);
    for (uint8_t attempt = 0u; attempt < 5u; ++attempt) {
        if (nrf24_read_reg(dev, REG_RF_CH) == expect) {
            return true;
        }
        hal->delay_us(20000u);
        nrf24_write_reg(dev, REG_RF_CH, expect);
    }
    return false;
}

/* -------------------------------------------------------------------------- */
/*  ACK payloads                                                              */
/* -------------------------------------------------------------------------- */

bool nrf24_write_ack_payload(nrf24_t *dev, uint8_t pipe,
                             const uint8_t *data, uint8_t len)
{
    uint8_t tx[1u + NRF24_MAX_PAYLOAD];

    if ((dev == NULL) || (data == NULL) || !dev->ack_payload ||
        (pipe > 5u) || (len == 0u) || (len > NRF24_MAX_PAYLOAD)) {
        return false;
    }

    tx[0] = (uint8_t)(CMD_W_ACK_PAYLOAD | pipe);
    memcpy(&tx[1], data, len);

    dev->hal->csn_write(false);
    dev->hal->spi_transfer(tx, NULL, (size_t)len + 1u);
    dev->hal->csn_write(true);
    return true;
}

bool nrf24_read_ack_payload(nrf24_t *dev, uint8_t *buf, uint8_t max_len,
                            uint8_t *out_len)
{
    uint8_t width = 0u;

    if ((dev == NULL) || (buf == NULL) || !dev->ack_payload) {
        return false;
    }
    if ((nrf24_status(dev) & STATUS_RX_DR) == 0u) {
        return false;                      /* no payload came back - normal */
    }

    /* Length lives with the payload, not in a register, so it must be read
     * before the payload itself. */
    {
        uint8_t tx[2] = { CMD_R_RX_PL_WID, CMD_NOP };
        uint8_t rx[2] = { 0u, 0u };
        dev->hal->csn_write(false);
        dev->hal->spi_transfer(tx, rx, 2);
        dev->hal->csn_write(true);
        width = rx[1];
    }

    /* "> 32 means corrupt, flush it" is the datasheet's own instruction. A
     * payload too big for the caller is discarded rather than truncated: half a
     * frame that still passes a length check is worse than none. */
    if ((width == 0u) || (width > NRF24_MAX_PAYLOAD) || (width > max_len)) {
        nrf24_command(dev, CMD_FLUSH_RX);
        nrf24_write_reg(dev, REG_STATUS, STATUS_RX_DR);
        return false;
    }

    {
        uint8_t tx[1u + NRF24_MAX_PAYLOAD];
        uint8_t rx[1u + NRF24_MAX_PAYLOAD];
        memset(tx, CMD_NOP, sizeof(tx));
        tx[0] = CMD_R_RX_PAYLOAD;
        dev->hal->csn_write(false);
        dev->hal->spi_transfer(tx, rx, (size_t)width + 1u);
        dev->hal->csn_write(true);
        memcpy(buf, &rx[1], width);
    }

    nrf24_write_reg(dev, REG_STATUS, STATUS_RX_DR);
    if (out_len != NULL) {
        *out_len = width;
    }
    return true;
}

uint8_t nrf24_read_register(const nrf24_t *dev, uint8_t reg)
{
    /* Thin public wrapper over the private accessor, for SWD diagnostics. */
    return nrf24_read_reg(dev, reg);
}

void nrf24_set_tx_mode(nrf24_t *dev)
{
    dev->hal->ce_write(false);
    uint8_t cfg = nrf24_read_reg(dev, REG_CONFIG);
    cfg &= (uint8_t)~CONFIG_PRIM_RX;
    cfg |= CONFIG_PWR_UP;
    nrf24_write_reg(dev, REG_CONFIG, cfg);
    dev->hal->delay_us(150);
}

void nrf24_set_rx_mode(nrf24_t *dev)
{
    uint8_t cfg = nrf24_read_reg(dev, REG_CONFIG);
    cfg |= CONFIG_PWR_UP | CONFIG_PRIM_RX;
    nrf24_write_reg(dev, REG_CONFIG, cfg);
    nrf24_write_reg(dev, REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
    nrf24_command(dev, CMD_FLUSH_RX);
    dev->hal->ce_write(true);   /* start listening */
    dev->hal->delay_us(150);
}

nrf24_result_t nrf24_send(nrf24_t *dev, const uint8_t *data, uint8_t len)
{
    if (dev == NULL || data == NULL || len != dev->payload_width) {
        return NRF24_INVALID;
    }

    dev->hal->ce_write(false);

    /* Load the TX FIFO. */
    uint8_t tx[1 + NRF24_MAX_PAYLOAD];
    tx[0] = CMD_W_TX_PAYLOAD;
    memcpy(&tx[1], data, len);
    dev->hal->csn_write(false);
    dev->hal->spi_transfer(tx, NULL, (size_t)len + 1u);
    dev->hal->csn_write(true);

    /* Strobe CE >= 10 us to transmit. */
    dev->hal->ce_write(true);
    dev->hal->delay_us(15);
    dev->hal->ce_write(false);

    /* Wait for the transmit result. */
    for (uint32_t i = 0; i < TX_WAIT_STEPS; ++i) {
        uint8_t st = nrf24_status(dev);
        if (st & STATUS_TX_DS) {
            nrf24_write_reg(dev, REG_STATUS, STATUS_TX_DS);
            return NRF24_OK;
        }
        if (st & STATUS_MAX_RT) {
            nrf24_write_reg(dev, REG_STATUS, STATUS_MAX_RT);
            nrf24_command(dev, CMD_FLUSH_TX);
            return NRF24_MAX_RT;
        }
        dev->hal->delay_us(TX_WAIT_US);
    }

    /* Timed out waiting for either result flag. The payload is still sitting in
     * the TX FIFO and whatever status bits are set are still set, so leaving
     * now would stack the next payload behind this one. The FIFO holds three:
     * after three timeouts the radio stops transmitting ENTIRELY and never
     * recovers, which presents as a link that works and then simply stops with
     * no corrupt frames to show for it.
     *
     * The MAX_RT path below already cleans up; this one did not. */
    nrf24_write_reg(dev, REG_STATUS, STATUS_TX_DS | STATUS_MAX_RT);
    nrf24_command(dev, CMD_FLUSH_TX);
    return NRF24_TIMEOUT;
}

bool nrf24_data_available(nrf24_t *dev)
{
    /* RX FIFO empty flag is bit 0 of FIFO_STATUS. */
    return (nrf24_read_reg(dev, REG_FIFO_STATUS) & 0x01u) == 0u;
}

bool nrf24_read(nrf24_t *dev, uint8_t *buf, uint8_t len)
{
    if (dev == NULL || buf == NULL || len != dev->payload_width) {
        return false;
    }
    if (!nrf24_data_available(dev)) {
        return false;
    }

    uint8_t tx[1 + NRF24_MAX_PAYLOAD];
    uint8_t rx[1 + NRF24_MAX_PAYLOAD];
    tx[0] = CMD_R_RX_PAYLOAD;
    memset(&tx[1], CMD_NOP, len);

    dev->hal->csn_write(false);
    dev->hal->spi_transfer(tx, rx, (size_t)len + 1u);
    dev->hal->csn_write(true);

    memcpy(buf, &rx[1], len);
    nrf24_write_reg(dev, REG_STATUS, STATUS_RX_DR);
    return true;
}

void nrf24_power_down(nrf24_t *dev)
{
    dev->hal->ce_write(false);
    uint8_t cfg = nrf24_read_reg(dev, REG_CONFIG);
    cfg &= (uint8_t)~CONFIG_PWR_UP;
    nrf24_write_reg(dev, REG_CONFIG, cfg);
}

/**
 ******************************************************************************
 * @file    nrf24.h
 * @brief   Platform-agnostic nRF24L01+ driver (shared by station and car).
 *
 * Wiring per node (provided by the caller's HAL):
 *   SPI  SCK/MOSI/MISO   - full-duplex, mode 0, MSB-first, <= 10 MHz
 *   CSN  (chip select)   - GPIO output, active low
 *   CE   (chip enable)   - GPIO output, TX strobe / RX listen
 *   IRQ  (optional)      - GPIO input, not required for the polled API here
 ******************************************************************************
 */
#ifndef SHARED_NRF24_H
#define SHARED_NRF24_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Largest nRF24 payload. */
#define NRF24_MAX_PAYLOAD   32u
/** @brief Fixed address width used by this driver (bytes). */
#define NRF24_ADDR_WIDTH    5u

/** @brief Result of a transmit attempt. */
typedef enum {
    NRF24_OK = 0,       /**< Auto-acknowledged by the receiver.            */
    NRF24_MAX_RT,       /**< Max retransmits reached (no ACK).             */
    NRF24_TIMEOUT,      /**< No TX_DS/MAX_RT within the wait budget.       */
    NRF24_INVALID,      /**< Bad argument.                                 */
} nrf24_result_t;

/** @brief On-air data rate. */
typedef enum {
    NRF24_DR_1MBPS = 0,
    NRF24_DR_2MBPS,
    NRF24_DR_250KBPS,
} nrf24_datarate_t;

/** @brief Output power. */
typedef enum {
    NRF24_PWR_M18DBM = 0, /**< -18 dBm */
    NRF24_PWR_M12DBM,     /**< -12 dBm */
    NRF24_PWR_M6DBM,      /**<  -6 dBm */
    NRF24_PWR_0DBM,       /**<   0 dBm */
} nrf24_power_t;

/**
 * @brief Hardware access hooks the caller must implement (one radio per HAL).
 *
 * @p spi_transfer does a full-duplex exchange of @p len bytes; @p rx may be
 * NULL to discard input, @p tx may be NULL to clock out zeros. CSN/CE take the
 * desired pin level (true = high). @p delay_us busy-waits microseconds.
 */
typedef struct {
    void (*spi_transfer)(const uint8_t *tx, uint8_t *rx, size_t len);
    void (*csn_write)(bool high);
    void (*ce_write)(bool high);
    void (*delay_us)(uint32_t us);
} nrf24_hal_t;

/** @brief Link configuration. */
typedef struct {
    uint8_t          channel;                 /**< RF channel 0..125.        */
    nrf24_datarate_t data_rate;
    nrf24_power_t    power;
    uint8_t          payload_width;           /**< Fixed payload 1..32.      */
    uint8_t          address[NRF24_ADDR_WIDTH]; /**< TX / RX pipe-0 address.  */
    bool             auto_ack;                /**< Enable Enhanced ShockBurst.*/
    /**
     * @brief Let the receiver attach a payload to its acknowledgement.
     *
     * The ACK is already being sent, so the return data costs NO extra airtime
     * and cannot collide with the forward link. This is how the car returns
     * telemetry without ever becoming a transmitter.
     *
     * Requires auto_ack, and enables dynamic payload length on pipe 0 - the
     * nRF24 has no fixed-width ACK payload mode. MUST match on both ends: a
     * receiver attaching payloads to a transmitter that is not expecting them
     * leaves data stuck in the transmitter's RX FIFO until it blocks.
     */
    bool             ack_payload;
} nrf24_config_t;

/** @brief Driver instance. Treat as opaque; initialise with nrf24_init(). */
typedef struct {
    const nrf24_hal_t *hal;
    uint8_t payload_width;
    bool    ack_payload;   /**< dynamic payload length is on */
} nrf24_t;

/**
 * @brief Configure the radio from @p cfg using @p hal for I/O.
 * @return true if the chip responds (a register read-back matches), else false
 *         — a quick way to catch wiring/power problems.
 */
bool nrf24_init(nrf24_t *dev, const nrf24_hal_t *hal, const nrf24_config_t *cfg);

/** @brief Switch to primary transmitter (PTX) mode. */
void nrf24_set_tx_mode(nrf24_t *dev);

/** @brief Switch to primary receiver (PRX) mode and start listening (CE high). */
void nrf24_set_rx_mode(nrf24_t *dev);

/**
 * @brief Transmit one payload and block until ACK, max-retransmit, or timeout.
 * @param data Payload bytes.
 * @param len  Payload length (must equal the configured payload width).
 */
nrf24_result_t nrf24_send(nrf24_t *dev, const uint8_t *data, uint8_t len);

/** @brief True if a payload is waiting in the RX FIFO (PRX mode). */
bool nrf24_data_available(nrf24_t *dev);

/**
 * @brief Read one payload from the RX FIFO.
 * @param buf Destination buffer.
 * @param len Bytes to read (the configured payload width).
 * @return true if a payload was read.
 */
bool nrf24_read(nrf24_t *dev, uint8_t *buf, uint8_t len);

/** @brief Power the radio down (lowest current; must re-enter TX/RX to use). */
void nrf24_power_down(nrf24_t *dev);

/* -------------------------------------------------------------------------- */
/*  ACK payloads (requires cfg.ack_payload on BOTH ends)                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief RECEIVER side: queue bytes to ride out on the next acknowledgement.
 * @param dev  Radio in PRX mode.
 * @param pipe Pipe the payload answers, 0..5. Use 0 for a single-pipe link.
 * @param data Payload, 1..32 bytes.
 * @param len  Payload length.
 * @return true if it was queued.
 *
 * The payload is consumed by the NEXT packet received on that pipe, so call
 * this after handling each packet to keep one always ready. The chip holds up
 * to three; queueing faster than packets arrive fills the FIFO and the oldest
 * data goes out first, which for telemetry means STALE data. Queue exactly one
 * per received packet.
 */
bool nrf24_write_ack_payload(nrf24_t *dev, uint8_t pipe,
                             const uint8_t *data, uint8_t len);

/**
 * @brief TRANSMITTER side: collect the payload that came back with the ACK.
 * @param dev     Radio in PTX mode, immediately after nrf24_send() returned OK.
 * @param buf     Destination.
 * @param max_len Size of @p buf; a longer payload is discarded, not truncated.
 * @param out_len Receives the actual length.
 * @return true if a payload was waiting.
 *
 * Not every ACK carries one - the receiver may not have queued anything yet -
 * so a false return is normal, not an error.
 */
bool nrf24_read_ack_payload(nrf24_t *dev, uint8_t *buf, uint8_t max_len,
                            uint8_t *out_len);

/* -------------------------------------------------------------------------- */
/*  Diagnostics                                                               */
/* -------------------------------------------------------------------------- */

/* Configuration-register addresses, for read-back via nrf24_read_register().
 * A responding module returns the values written by nrf24_init(); all 0x00
 * means MISO is unwired or the module is unpowered, all 0xFF means MISO is
 * floating. */
#define NRF24_REG_CONFIG        0x00u
#define NRF24_REG_EN_AA         0x01u
#define NRF24_REG_SETUP_RETR    0x04u
#define NRF24_REG_RF_CH         0x05u
#define NRF24_REG_RF_SETUP      0x06u
#define NRF24_REG_STATUS        0x07u
#define NRF24_REG_RX_PW_P0      0x11u

/**
 * @brief Read one configuration register over SPI.
 * @param dev A device whose HAL is set (nrf24_init() sets it before it can
 *            fail, so this works even after a failed presence check).
 * @param reg Register address, e.g. NRF24_REG_CONFIG.
 * @return The register byte. Diagnostic use — a failing bus reads all 0x00
 *         or all 0xFF.
 */
uint8_t nrf24_read_register(const nrf24_t *dev, uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif /* SHARED_NRF24_H */

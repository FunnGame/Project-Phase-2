/**
 ******************************************************************************
 * @file    e2e.h
 * @brief   AUTOSAR-style end-to-end protection for CAN and RF frames.
 *
 * Two mechanisms, both required on every safety-relevant frame:
 *
 *   CRC-8         catches corrupted bytes, and - because the message's Data ID
 *                 is folded into the calculation - catches a frame that arrives
 *                 under the WRONG identifier with intact bytes (masquerading).
 *
 *   Alive counter catches what a CRC structurally cannot: a repeated value
 *                 means a stale frame was replayed, a skipped value means one
 *                 was dropped. In both cases the bytes were never damaged, so
 *                 no checksum would object.
 *
 * Frame layout, identical for every protected message:
 *
 *      byte 0        byte 1                      bytes 2..n
 *   +---------+-------------------+   +-------------------------------+
 *   |  CRC-8  | counter | msg use |   |            payload            |
 *   |  8 bits | 4 bits  | 4 bits  |   |                               |
 *   +---------+-------------------+   +-------------------------------+
 ******************************************************************************
 */
#ifndef SHARED_E2E_H
#define SHARED_E2E_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Byte holding the CRC. */
#define E2E_CRC_BYTE        0u
/** Byte holding the alive counter, in its low nibble. */
#define E2E_COUNTER_BYTE    1u
/** Alive counter wraps at 16 - 320 ms at a 20 ms cycle, far longer than any
 *  timeout in this system, so a wrap can never be mistaken for a stall. */
#define E2E_COUNTER_MODULO  16u

/** @brief Result of checking a received frame. */
typedef enum {
    E2E_OK = 0,        /**< CRC valid and the counter advanced by exactly one. */
    E2E_CRC_ERROR,     /**< Checksum mismatch - corrupted, or wrong Data ID.   */
    E2E_REPEATED,      /**< Counter unchanged: a stale frame was replayed.     */
    E2E_LOST,          /**< Counter jumped: one or more frames were dropped.   */
    E2E_INVALID_PARAM, /**< NULL pointer or a frame shorter than 2 bytes.      */
} E2E_Status;

/** @brief Receiver state for one message. Zero-initialise before first use. */
typedef struct {
    uint8_t last_counter;  /**< Counter from the last accepted frame.         */
    bool    synced;        /**< False until the first valid frame is seen.    */
    uint8_t lost_frames;   /**< Frames skipped by the most recent E2E_LOST.   */
} E2E_Receiver;

/**
 * @brief AUTOSAR CRC-8 (SAE J1850): poly 0x1D, init 0xFF, final XOR 0xFF.
 * @param data_id Message Data ID - use the CAN identifier.
 * @param data    Frame buffer. Byte 0 (the CRC itself) is NOT included.
 * @param len     Frame length in bytes, at least 2.
 * @return The computed CRC.
 *
 * Covers the Data ID low byte, then its high byte, then @p data[1..len-1].
 */
uint8_t E2E_Crc8(uint16_t data_id, const uint8_t *data, size_t len);

/**
 * @brief Stamp a frame before transmission: write the counter, then the CRC.
 * @param data_id Message Data ID - use the CAN identifier.
 * @param data    Frame buffer, modified in place. Payload must already be set.
 * @param len     Frame length in bytes, at least 2.
 * @param counter Sender's counter for THIS message. Incremented on return, so
 *                each protected message needs its own.
 *
 * Only the low nibble of byte 1 is touched; the high nibble is left to the
 * message, which is why several messages carry a 4-bit field there.
 */
void E2E_Protect(uint16_t data_id, uint8_t *data, size_t len, uint8_t *counter);

/**
 * @brief Validate a received frame.
 * @param data_id Message Data ID - use the CAN identifier.
 * @param data    Received frame.
 * @param len     Frame length in bytes, at least 2.
 * @param rx      Per-message receiver state.
 * @return E2E_OK only if the frame should be acted upon.
 *
 * The first frame after @p rx is zeroed always returns E2E_OK and adopts its
 * counter - a receiver that starts mid-stream has no basis to reject it.
 *
 * E2E_LOST reports the gap in @p rx->lost_frames and RESYNCS, so a single
 * dropped frame costs one frame rather than deadlocking the link. Whether a
 * gap is tolerable is the caller's decision, not this layer's.
 */
E2E_Status E2E_Check(uint16_t data_id, const uint8_t *data, size_t len,
                     E2E_Receiver *rx);

#ifdef __cplusplus
}
#endif

#endif /* SHARED_E2E_H */

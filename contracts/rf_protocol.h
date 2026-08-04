/**
 ******************************************************************************
 * @file    rf_protocol.h
 * @brief   Shared control-frame protocol (PC -> station -> car).
 *
 * The wire format produced by the PC control app
 * (station/control-app, ControlFrameSerializer) and decoded by both the
 * F446RE station and the F103 car. Keep this file MCU-independent (only
 * stdint/stdbool/stddef) so every node shares one definition.
 *
 * Frame layout (7 bytes, one byte per field so byte order is irrelevant):
 *   [0] magic     0xA5           frame start / sync
 *   [1] seq       0..255         increments per frame (drop/duplicate detect)
 *   [2] steering  int8  -100..+100  (left..right)
 *   [3] throttle  uint8 0..100      (drive magnitude)
 *   [4] brake     uint8 0..100
 *   [5] buttons   uint8 bit0=reverse, bit1=armed; bits 2..7 reserved
 *   [6] crc       uint8 CRC-8 (poly 0x07, init 0x00) over bytes [0..5]
 ******************************************************************************
 */
#ifndef CONTRACTS_RF_PROTOCOL_H
#define CONTRACTS_RF_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RF_CONTROL_MAGIC        0xA5u
#define RF_CONTROL_FRAME_SIZE   7u

/* buttons byte bit masks */
#define RF_BTN_REVERSE          0x01u
#define RF_BTN_ARMED            0x02u

/** @brief On-the-wire control frame. Packed to exactly 7 bytes. */
typedef struct __attribute__((packed)) {
    uint8_t magic;      /**< always RF_CONTROL_MAGIC                         */
    uint8_t seq;        /**< sequence counter                               */
    int8_t  steering;   /**< -100..+100                                     */
    uint8_t throttle;   /**< 0..100                                         */
    uint8_t brake;      /**< 0..100                                         */
    uint8_t buttons;    /**< RF_BTN_* bit field                             */
    uint8_t crc;        /**< CRC-8 over the first 6 bytes                   */
} rf_control_frame_t;

/** @brief True if the armed bit is set. */
static inline bool rf_control_is_armed(const rf_control_frame_t *f)
{
    return (f->buttons & RF_BTN_ARMED) != 0u;
}

/** @brief True if the reverse bit is set. */
static inline bool rf_control_is_reverse(const rf_control_frame_t *f)
{
    return (f->buttons & RF_BTN_REVERSE) != 0u;
}

/**
 * @brief CRC-8 (polynomial 0x07, init 0x00, no reflection).
 * @param data Bytes to checksum.
 * @param len  Number of bytes.
 * @return The 8-bit CRC. Must match the PC-side implementation.
 */
uint8_t rf_control_crc8(const uint8_t *data, size_t len);

/**
 * @brief Validate a complete 7-byte frame (magic + CRC).
 * @param frame Pointer to a candidate frame.
 * @return true if magic and CRC are correct.
 */
bool rf_control_frame_valid(const rf_control_frame_t *frame);

/**
 * @brief Serialise @p frame's fields and (re)compute its CRC in place.
 *        Handy for the station when re-emitting frames over the radio.
 */
void rf_control_frame_finalize(rf_control_frame_t *frame);

/* -------------------------------------------------------------------------- */
/*  Streaming parser                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Incremental parser state. Feed it received bytes one at a time; it
 *        syncs on the magic byte, collects a frame and checks the CRC.
 */
typedef struct {
    uint8_t buf[RF_CONTROL_FRAME_SIZE];
    uint8_t idx;
} rf_control_parser_t;

/** @brief Reset a parser to the "hunting for magic" state. */
void rf_control_parser_reset(rf_control_parser_t *p);

/**
 * @brief Feed one received byte to the parser.
 * @param p    Parser state.
 * @param byte The received byte.
 * @param out  Filled with the decoded frame when the return value is true.
 * @return true exactly when a complete, CRC-valid frame has been decoded.
 *
 * On a CRC failure the parser resynchronises by scanning the buffered bytes
 * for the next magic, so a single dropped byte costs at most one frame.
 */
bool rf_control_parser_feed(rf_control_parser_t *p, uint8_t byte,
                            rf_control_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CONTRACTS_RF_PROTOCOL_H */

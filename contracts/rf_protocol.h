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
/*  Telemetry: car -> station (the UPLINK)                                    */
/* -------------------------------------------------------------------------- */
/*
 * Rides back on the nRF24 auto-ACK of each control frame, so it costs no extra
 * airtime. The gateway fills it from the CAN messages it already receives
 * (0x300 VC_Status, 0x310 VC_Motion, and the heartbeats); the station firmware
 * decodes it for its own display.
 *
 * It stops at the station - it is NOT forwarded to the laptop - so unlike the
 * control frame this layout has no C# counterpart to keep in step.
 *
 *   [ 0] magic     0x5A            distinct from the downlink's 0xA5
 *   [ 1] seq_echo  the control frame this acknowledges -> round-trip latency
 *   [ 2] state     bits 0-2 vehicle state, 3-5 AEB state, 6 degraded
 *   [ 3] faults    VC_Faults verbatim
 *   [ 4] speed_lo  int16 mm/s, little-endian
 *   [ 5] speed_hi
 *   [ 6] range_lo  uint16 mm, 0xFFFF = no target / no sensor node
 *   [ 7] range_hi
 *   [ 8] throttle  APPLIED 0..100 - differs from intent once AEB can override
 *   [ 9] brake     APPLIED 0..100
 *   [10] health    bits 0-3 link quality, 4-6 nodes present, 7 E2E error seen
 *   [11] slow_id   selector for the rotating diagnostic below
 *   [12] slow_val  one diagnostic byte per frame; all of them within ~1 s
 *   [13] crc       CRC-8 over bytes [0..12]
 *
 * Both ends must agree on the size or the station rejects every frame on
 * length - there is no negotiation.
 */

#define RF_TELEM_MAGIC          0x5Au
#define RF_TELEM_FRAME_SIZE     14u

/** No target, or no sensor node on the bus. */
#define RF_TELEM_RANGE_NONE     0xFFFFu

/* health byte: which nodes have been heard from recently */
#define RF_TELEM_NODE_GW        0x10u
#define RF_TELEM_NODE_VC        0x20u
#define RF_TELEM_NODE_SF        0x40u
#define RF_TELEM_E2E_ERROR      0x80u
#define RF_TELEM_LINKQ_MASK     0x0Fu

/** @brief Rotating diagnostic selector for byte 12. */
typedef enum {
    RF_SLOW_VC_UPTIME_LO = 0,
    RF_SLOW_VC_UPTIME_HI,
    RF_SLOW_VC_CAN_TEC,
    RF_SLOW_VC_CAN_REC,
    RF_SLOW_VC_E2E_ERRORS,
    RF_SLOW_GW_CAN_TEC,
    RF_SLOW_GW_CAN_REC,
    RF_SLOW_GW_RF_DROPPED,
    RF_SLOW_COUNT,
} rf_slow_id_t;

/** @brief On-the-wire telemetry frame. Packed to exactly 14 bytes. */
typedef struct __attribute__((packed)) {
    uint8_t  magic;     /**< always RF_TELEM_MAGIC                          */
    uint8_t  seq_echo;  /**< seq of the control frame being acknowledged    */
    uint8_t  state;     /**< packed vehicle/AEB state - use the accessors   */
    uint8_t  faults;    /**< VC_Faults bitfield, verbatim                   */
    int16_t  speed;     /**< mm/s, positive forward                         */
    uint16_t range;     /**< mm, RF_TELEM_RANGE_NONE if unavailable         */
    uint8_t  throttle;  /**< applied, 0..100                                */
    uint8_t  brake;     /**< applied, 0..100                                */
    uint8_t  health;    /**< RF_TELEM_* bits                                */
    uint8_t  slow_id;   /**< rf_slow_id_t                                   */
    uint8_t  slow_val;  /**< the selected diagnostic                        */
    uint8_t  crc;       /**< CRC-8 over the first 13 bytes                  */
} rf_telemetry_frame_t;

/*
 * The two ends are different cores (Cortex-M3 gateway, Cortex-M4 station) with
 * different compilers' idea of alignment. "packed" should make that moot, but
 * a silent disagreement here produces frames that are the right LENGTH and the
 * wrong SHAPE - which fails the CRC and looks exactly like radio interference.
 * Fail the build instead.
 */
#if !defined(__cplusplus)
_Static_assert(sizeof(rf_telemetry_frame_t) == RF_TELEM_FRAME_SIZE,
               "rf_telemetry_frame_t does not match RF_TELEM_FRAME_SIZE");
_Static_assert(sizeof(rf_control_frame_t) == RF_CONTROL_FRAME_SIZE,
               "rf_control_frame_t does not match RF_CONTROL_FRAME_SIZE");
#endif

/** @brief Vehicle state (0..7), matching VC_VehicleState in adas.dbc. */
static inline uint8_t rf_telem_vehicle_state(const rf_telemetry_frame_t *f)
{
    return (uint8_t)(f->state & 0x07u);
}

/** @brief AEB state (0..7), matching VC_AebState in adas.dbc. */
static inline uint8_t rf_telem_aeb_state(const rf_telemetry_frame_t *f)
{
    return (uint8_t)((f->state >> 3) & 0x07u);
}

/** @brief True when the vehicle node reports itself degraded. */
static inline bool rf_telem_degraded(const rf_telemetry_frame_t *f)
{
    return (f->state & 0x40u) != 0u;
}

/** @brief Pack the two state enums and the degraded flag into byte 2. */
static inline uint8_t rf_telem_pack_state(uint8_t vehicle, uint8_t aeb,
                                          bool degraded)
{
    return (uint8_t)((vehicle & 0x07u) |
                     (uint8_t)((aeb & 0x07u) << 3) |
                     (degraded ? 0x40u : 0x00u));
}

/**
 * @brief Validate a telemetry frame (magic + CRC).
 * @return true if it should be believed.
 *
 * The nRF24 already CRCs the air interface, so this is belt-and-braces - but a
 * corrupted `state` byte would show DISARMED for an armed car, which is the one
 * display error that gets somebody hurt.
 */
bool rf_telemetry_frame_valid(const rf_telemetry_frame_t *frame);

/** @brief Fill in magic and recompute the CRC in place, ready to transmit. */
void rf_telemetry_frame_finalize(rf_telemetry_frame_t *frame);

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

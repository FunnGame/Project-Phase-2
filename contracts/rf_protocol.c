/**
 ******************************************************************************
 * @file    rf_protocol.c
 * @brief   Shared control-frame protocol implementation (MCU-independent).
 ******************************************************************************
 */
#include "rf_protocol.h"

#include <string.h>

uint8_t rf_control_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00u;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8u; ++bit) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u)
                                : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

bool rf_control_frame_valid(const rf_control_frame_t *frame)
{
    const uint8_t *bytes = (const uint8_t *)frame;
    return (frame->magic == RF_CONTROL_MAGIC) &&
           (frame->crc == rf_control_crc8(bytes, RF_CONTROL_FRAME_SIZE - 1u));
}

void rf_control_frame_finalize(rf_control_frame_t *frame)
{
    frame->magic = RF_CONTROL_MAGIC;
    const uint8_t *bytes = (const uint8_t *)frame;
    frame->crc = rf_control_crc8(bytes, RF_CONTROL_FRAME_SIZE - 1u);
}

/* -------------------------------------------------------------------------- */
/*  Telemetry (car -> station)                                                */
/* -------------------------------------------------------------------------- */

bool rf_telemetry_frame_valid(const rf_telemetry_frame_t *frame)
{
    if (frame->magic != RF_TELEM_MAGIC) {
        return false;
    }
    const uint8_t *bytes = (const uint8_t *)frame;
    return frame->crc == rf_control_crc8(bytes, RF_TELEM_FRAME_SIZE - 1u);
}

void rf_telemetry_frame_finalize(rf_telemetry_frame_t *frame)
{
    frame->magic = RF_TELEM_MAGIC;
    const uint8_t *bytes = (const uint8_t *)frame;
    frame->crc = rf_control_crc8(bytes, RF_TELEM_FRAME_SIZE - 1u);
}

void rf_control_parser_reset(rf_control_parser_t *p)
{
    p->idx = 0u;
}

bool rf_control_parser_feed(rf_control_parser_t *p, uint8_t byte,
                            rf_control_frame_t *out)
{
    /* While hunting, ignore everything until the magic start byte. */
    if (p->idx == 0u && byte != RF_CONTROL_MAGIC) {
        return false;
    }

    p->buf[p->idx++] = byte;

    if (p->idx < RF_CONTROL_FRAME_SIZE) {
        return false;               /* still collecting */
    }

    /* A full frame is buffered. */
    if (rf_control_crc8(p->buf, RF_CONTROL_FRAME_SIZE - 1u) ==
        p->buf[RF_CONTROL_FRAME_SIZE - 1u]) {
        memcpy(out, p->buf, RF_CONTROL_FRAME_SIZE);
        p->idx = 0u;
        return true;
    }

    /* CRC failed: resync by dropping the first byte and scanning the rest for
     * the next magic, keeping any partial frame that follows it. */
    uint8_t i = 1u;
    while (i < RF_CONTROL_FRAME_SIZE && p->buf[i] != RF_CONTROL_MAGIC) {
        ++i;
    }
    if (i < RF_CONTROL_FRAME_SIZE) {
        uint8_t remaining = (uint8_t)(RF_CONTROL_FRAME_SIZE - i);
        memmove(p->buf, &p->buf[i], remaining);
        p->idx = remaining;
    } else {
        p->idx = 0u;
    }
    return false;
}

/**
 ******************************************************************************
 * @file    e2e.c
 * @brief   AUTOSAR-style end-to-end protection. See e2e.h for the contract.
 ******************************************************************************
 */
#include "e2e.h"

/* SAE J1850 / AUTOSAR Crc_CalculateCRC8 parameters. */
#define E2E_CRC_POLY    0x1Du
#define E2E_CRC_INIT    0xFFu
#define E2E_CRC_XOROUT  0xFFu

#define E2E_COUNTER_MASK  0x0Fu
#define E2E_MIN_LEN       2u

/* Bitwise rather than table-driven: 256 bytes of Flash is a poor trade on a
 * 64 KB part for eight iterations over seven bytes, twenty times a second. */
static uint8_t crc8_update(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (uint8_t bit = 0u; bit < 8u; bit++) {
        crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ E2E_CRC_POLY)
                            : (uint8_t)(crc << 1);
    }
    return crc;
}

uint8_t E2E_Crc8(uint16_t data_id, const uint8_t *data, size_t len)
{
    uint8_t crc = E2E_CRC_INIT;

    if ((data == NULL) || (len < E2E_MIN_LEN)) {
        return 0u;
    }

    /* Data ID first, low byte then high. Including it is what makes a frame
     * received under the wrong identifier fail even with intact bytes. */
    crc = crc8_update(crc, (uint8_t)(data_id & 0xFFu));
    crc = crc8_update(crc, (uint8_t)((data_id >> 8) & 0xFFu));

    /* Byte 0 holds the CRC and is excluded. */
    for (size_t i = 1u; i < len; i++) {
        crc = crc8_update(crc, data[i]);
    }

    return (uint8_t)(crc ^ E2E_CRC_XOROUT);
}

void E2E_Protect(uint16_t data_id, uint8_t *data, size_t len, uint8_t *counter)
{
    if ((data == NULL) || (counter == NULL) || (len < E2E_MIN_LEN)) {
        return;
    }

    /* Counter goes in before the CRC - it is one of the bytes covered. Only the
     * low nibble is ours; the high nibble belongs to the message. */
    data[E2E_COUNTER_BYTE] = (uint8_t)((data[E2E_COUNTER_BYTE] & 0xF0u) |
                                       (*counter & E2E_COUNTER_MASK));
    data[E2E_CRC_BYTE] = E2E_Crc8(data_id, data, len);

    *counter = (uint8_t)((*counter + 1u) % E2E_COUNTER_MODULO);
}

E2E_Status E2E_Check(uint16_t data_id, const uint8_t *data, size_t len,
                     E2E_Receiver *rx)
{
    uint8_t received;
    uint8_t expected;

    if ((data == NULL) || (rx == NULL) || (len < E2E_MIN_LEN)) {
        return E2E_INVALID_PARAM;
    }

    /* CRC first: until it passes, the counter byte is not trustworthy either. */
    if (E2E_Crc8(data_id, data, len) != data[E2E_CRC_BYTE]) {
        return E2E_CRC_ERROR;
    }

    received = (uint8_t)(data[E2E_COUNTER_BYTE] & E2E_COUNTER_MASK);
    rx->lost_frames = 0u;

    if (!rx->synced) {
        rx->synced = true;
        rx->last_counter = received;
        return E2E_OK;
    }

    if (received == rx->last_counter) {
        return E2E_REPEATED;   /* state deliberately unchanged */
    }

    expected = (uint8_t)((rx->last_counter + 1u) % E2E_COUNTER_MODULO);
    if (received != expected) {
        /* Resync rather than latch: a gap costs one frame, not the link. */
        rx->lost_frames = (uint8_t)((received - expected) % E2E_COUNTER_MODULO);
        rx->last_counter = received;
        return E2E_LOST;
    }

    rx->last_counter = received;
    return E2E_OK;
}

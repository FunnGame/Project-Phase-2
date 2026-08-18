#ifndef DRV_CAN_H
#define DRV_CAN_H

#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    uint32_t std_id;
    uint8_t  dlc;
    uint8_t  data[8];
} CAN_TxHeader_t;

typedef struct {
    uint32_t std_id;
    uint8_t  dlc;
    uint8_t  data[8];
} CAN_RxHeader_t;

/** @brief Test modes, selected at init. See DRV_CAN_Init(). */
typedef enum {
    /** Normal operation: transmit, receive, and require another node to ACK. */
    CAN_MODE_NORMAL = 0,

    /** Loopback. The peripheral treats its own transmissions as received and
     *  DISREGARDS the RX pin - so nothing needs to acknowledge it and it can
     *  never go bus-off for want of an ACK - while STILL driving CANTX.
     *
     *  This is the bring-up mode: the node blasts real frames onto real wire
     *  that a sniffer can decode, with no dependency on the far end being
     *  configured correctly. It cannot hear other nodes, so it is for
     *  one-way bring-up only. */
    CAN_MODE_LOOPBACK,

    /** Silent / listen-only: receives, never drives the bus, never ACKs. */
    CAN_MODE_SILENT,

    /** Silent loopback: entirely internal, the bus is not touched at all.
     *  Validates bit timing, filters and the FIFO path with no transceiver. */
    CAN_MODE_SILENT_LOOPBACK,
} CAN_Mode;

/**
 * @brief Bring up CAN1 on PB8 (RX) / PB9 (TX) at 500 kbit/s.
 * @param mode CAN_MODE_NORMAL for real operation; see CAN_Mode for the
 *             bring-up and diagnostic modes.
 * @return true on success. false if the peripheral could not enter or leave
 *         initialisation mode within the timeout - which in practice means the
 *         transceiver is unpowered or absent, or the bus is stuck dominant.
 *
 * The caller MUST check this. Leaving initialisation mode requires the
 * peripheral to observe 11 consecutive recessive bits on the bus, so a missing
 * transceiver will fail here rather than anywhere later that would be easier to
 * misdiagnose.
 *
 * Automatic bus-off management is enabled: after bus-off the peripheral
 * recovers on its own once the bus is idle again, instead of staying dead until
 * the next reset.
 */
bool     DRV_CAN_Init(CAN_Mode mode);
bool     DRV_CAN_Transmit(const CAN_TxHeader_t *header);
bool     DRV_CAN_Receive(CAN_RxHeader_t *header);

/**
 * @brief Read the peripheral's transmit and receive error counters.
 * @param tec Filled with the transmit error counter, or NULL.
 * @param rec Filled with the receive error counter, or NULL.
 *
 * Non-zero means the bus is having trouble even if frames are still getting
 * through. Published in the node heartbeat so a marginal bus shows up as a
 * trend rather than as a sudden bus-off.
 */
void     DRV_CAN_GetErrorCounters(uint8_t *tec, uint8_t *rec);

/** @brief True when the peripheral has reached the bus-off state. */
bool     DRV_CAN_IsBusOff(void);

#endif /* DRV_CAN_H */
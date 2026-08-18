#include "drv_can.h"

/* Spin count for the mode-change waits. Deliberately a raw loop count rather
 * than a millisecond figure: this runs before any time base is guaranteed to
 * exist. Sized for roughly 50 ms at 72 MHz and ~0.5 s on the 8 MHz HSI, both
 * far longer than the ~200 us the peripheral actually needs. (When this driver
 * adopts drv_common.h, DRV_WaitFlag() replaces it.) */
#define CAN_MODE_TIMEOUT    1000000UL

/** @brief Wait for a bit in CAN1->MSR to reach @p want. false on timeout. */
static bool can_wait_msr(uint32_t mask, bool want) {
    for (uint32_t i = 0U; i < CAN_MODE_TIMEOUT; i++) {
        if (((CAN1->MSR & mask) != 0U) == want) {
            return true;
        }
    }
    return false;
}

bool DRV_CAN_Init(CAN_Mode mode) {
    /* 1. Enable Clocks for AFIO, GPIOB, CAN1 */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    /* 2. Remap CAN1 pins to PB8 (RX) / PB9 (TX) */
    AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;
    AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;

    /* PB8: input with pull-UP. PB9: alternate function push-pull, 50 MHz.
     *
     * The pull direction matters. CAN idles recessive = high, so an input that
     * floats or is pulled DOWN reads as a permanently dominant bus whenever the
     * transceiver is absent or unpowered - and a stuck-dominant bus means the
     * peripheral never sees the 11 recessive bits it needs to leave
     * initialisation mode below. Pulling up makes a disconnected transceiver
     * look like an idle bus instead. */
    GPIOB->CRH &= ~(0xFFU << 0);
    GPIOB->CRH |=  (0x08U << 0) | (0x0BU << 4);
    GPIOB->BSRR = (1U << 8);        /* ODR8 = 1 selects pull-up, atomically */

    /* 3. Enter CAN Initialization Mode */
    CAN1->MCR |= CAN_MCR_INRQ;
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    if (!can_wait_msr(CAN_MSR_INAK, true)) {
        return false;
    }

    /* 4. Bit timing: 500 kbit/s from PCLK1 = 36 MHz.
     *
     *   prescaler 4                    -> tq = 4 / 36 MHz = 111.1 ns
     *   SYNC 1 + BS1 15 + BS2 2 = 18 tq -> 2.0 us  -> 500 kbit/s
     *   sample point = (1 + 15) / 18   -> 88.9 %
     *
     * CiA recommends 87.5 % at this bit rate, and 88.9 % is the closest
     * reachable setting. Exactly 87.5 % would need SYNC + BS1 = 15.75 tq on an
     * 18 tq bit, and no other tq count that divides PCLK1 evenly lands on it
     * either: 24 tq would need BS1 = 20 tq, past the 4-bit TS1 field's 16 tq
     * ceiling, and 12 tq only offers 83.3 % or 91.7 %.
     *
     * Each register field is ONE LESS than the tq count it represents - the
     * most common bxCAN bring-up mistake. */
    CAN1->BTR = (0U << 24) | (1U << 20) | (14U << 16) | (3U << 0);

    /* 4a. Test mode. LBKM makes the peripheral ignore CANRX and accept its own
     * transmissions, so no external ACK is needed; SILM stops it driving
     * CANTX. LBKM alone therefore puts real frames on real wire that cannot
     * bus-off - the bring-up combination. */
    switch (mode) {
    case CAN_MODE_LOOPBACK:         CAN1->BTR |= CAN_BTR_LBKM;                break;
    case CAN_MODE_SILENT:           CAN1->BTR |= CAN_BTR_SILM;                break;
    case CAN_MODE_SILENT_LOOPBACK:  CAN1->BTR |= CAN_BTR_LBKM | CAN_BTR_SILM; break;
    case CAN_MODE_NORMAL:
    default:                                                                  break;
    }

    /* 4b. No automatic retransmission.
     *
     * With NART clear, a frame nobody ACKs is retransmitted forever, holding its
     * mailbox and pushing the error counter to bus-off - which is exactly what
     * happens during single-node bring-up, and it looks like a broken driver
     * rather than a missing peer. It is also the right setting for 20 ms cyclic
     * data: a frame that missed its slot is stale, and the next cycle's frame
     * supersedes it. */
    CAN1->MCR |= CAN_MCR_NART;

    /* 4c. Automatic bus-off management.
     *
     * Without this, bus-off is terminal until the next reset - and bus-off is
     * EASY to reach during bring-up: every unacknowledged frame adds 8 to the
     * transmit error counter and the limit is 255, so a node publishing 50
     * frames/s with nothing listening is off the bus in under a second. With
     * ABOM the peripheral leaves bus-off by itself once the bus is idle again,
     * so fixing the far end is enough - no power cycle required. */
    CAN1->MCR |= CAN_MCR_ABOM;

    /* 5. Exit Initialization Mode */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    if (!can_wait_msr(CAN_MSR_INAK, false)) {
        return false;
    }

    /* 6. Filter Configuration: Accept All Messages into FIFO0 */
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FA1R &= ~(1U << 0);
    CAN1->FS1R |= (1U << 0);   /* 32-bit scale */
    CAN1->FM1R &= ~(1U << 0);  /* Mask mode */
    CAN1->sFilterRegister[0].FR1 = 0x00000000U;
    CAN1->sFilterRegister[0].FR2 = 0x00000000U;
    CAN1->FFA1R &= ~(1U << 0); /* Assign to FIFO 0 */
    CAN1->FA1R |= (1U << 0);   /* Activate filter 0 */
    CAN1->FMR &= ~CAN_FMR_FINIT;

    return true;
}

bool DRV_CAN_Transmit(const CAN_TxHeader_t *header) {
    uint8_t mailbox = (CAN1->TSR >> 24) & 0x03U;
    if ((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0U) {
        return false; /* All mailboxes full */
    }

    CAN1->sTxMailBox[mailbox].TIR  = (header->std_id << 21);
    CAN1->sTxMailBox[mailbox].TDTR = (header->dlc & 0x0FU);
    CAN1->sTxMailBox[mailbox].TDLR = ((uint32_t)header->data[3] << 24) |
                                     ((uint32_t)header->data[2] << 16) |
                                     ((uint32_t)header->data[1] << 8)  |
                                     ((uint32_t)header->data[0]);
    CAN1->sTxMailBox[mailbox].TDHR = ((uint32_t)header->data[7] << 24) |
                                     ((uint32_t)header->data[6] << 16) |
                                     ((uint32_t)header->data[5] << 8)  |
                                     ((uint32_t)header->data[4]);
    CAN1->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;
    return true;
}

void DRV_CAN_GetErrorCounters(uint8_t *tec, uint8_t *rec) {
    uint32_t esr = CAN1->ESR;
    if (tec != NULL) {
        *tec = (uint8_t)((esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);
    }
    if (rec != NULL) {
        *rec = (uint8_t)((esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos);
    }
}

bool DRV_CAN_IsBusOff(void) {
    return (CAN1->ESR & CAN_ESR_BOFF) != 0U;
}

bool DRV_CAN_Receive(CAN_RxHeader_t *header) {
    if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0U) {
        return false; /* FIFO empty */
    }

    header->std_id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x07FFU;
    header->dlc    = CAN1->sFIFOMailBox[0].RDTR & 0x0FU;

    uint32_t low  = CAN1->sFIFOMailBox[0].RDLR;
    uint32_t high = CAN1->sFIFOMailBox[0].RDHR;

    header->data[0] = (uint8_t)(low >> 0);
    header->data[1] = (uint8_t)(low >> 8);
    header->data[2] = (uint8_t)(low >> 16);
    header->data[3] = (uint8_t)(low >> 24);
    header->data[4] = (uint8_t)(high >> 0);
    header->data[5] = (uint8_t)(high >> 8);
    header->data[6] = (uint8_t)(high >> 16);
    header->data[7] = (uint8_t)(high >> 24);

    CAN1->RF0R |= CAN_RF0R_RFOM0; /* Release FIFO */
    return true;
}
/*
 * drv_can.c
 *
 * Author: trong
 */

#include "drv_can.h"

void DRV_CAN_Init(void) {
    /* 1. Enable Clocks for AFIO, GPIOB, CAN1 */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    /* 2. Remap CAN1 pins to PB8 (RX) / PB9 (TX) */
    AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;
    AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;

    /* PB8: Input Floating, PB9: Alternate Function Push-Pull 50MHz */
    GPIOB->CRH &= ~(0xFFU << 0);
    GPIOB->CRH |=  (0x08U << 0) | (0x0BU << 4);

    /* 3. Enter CAN Initialization Mode */
    CAN1->MCR |= CAN_MCR_INRQ;
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    while ((CAN1->MSR & CAN_MSR_INAK) == 0U);

    /* 4. Bit Timing for 500 kbps (PCLK1 = 36 MHz)
     * Prescaler = 4 -> Tq = 1/9 MHz. BS1 = 12Tq, BS2 = 5Tq, Sync = 1Tq (Total 18Tq -> 500 kHz) */
    CAN1->BTR = (0U << 24) | (4U << 20) | (11U << 16) | (3U << 0);

    /* 5. Exit Initialization Mode */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while ((CAN1->MSR & CAN_MSR_INAK) != 0U);

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

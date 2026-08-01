#include "../inc/drv_can.h"

#define CAN_APB1_CLOCK        (36000000U)

#define CAN_DEFAULT_FILTER    (0U)


/*  Private Function Prototypes */

static CAN_TypeDef *CAN_GetInstance(CAN_t can);

static Status_t CAN_EnableClock(CAN_t can);

static Status_t CAN_GPIO_Init(CAN_t can);

static Status_t CAN_EnterInitMode(CAN_TypeDef *CANx);

static Status_t CAN_LeaveInitMode(CAN_TypeDef *CANx);

static Status_t CAN_ConfigureBitTiming(
    CAN_TypeDef *CANx,
    CAN_Baudrate_t baudrate);


/*  Private Functions */

static CAN_TypeDef *CAN_GetInstance(CAN_t can)
{
    switch(can)
    {
        case CAN_1:
            return CAN1;

        default:
            return (CAN_TypeDef *)0;
    }
}


static Status_t CAN_EnableClock(CAN_t can)
{
    if(can != CAN_1)
    {
        return STATUS_INVALID_PARAM;
    }

    /* GPIOA clock */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    /* AFIO clock */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    /* CAN1 clock */
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    return STATUS_OK;
}


static Status_t CAN_GPIO_Init(CAN_t can)
{
    if(can != CAN_1)
    {
        return STATUS_INVALID_PARAM;
    }

    /*
     * PA11 = CAN1_RX
     *
     * Input floating:
     * MODE = 00
     * CNF  = 01
     *
     * Configuration = 0x4
     */
    GPIOA->CRH &= ~(0xFU << 12U);
    GPIOA->CRH |=  (0x4U << 12U);


    /*
     * PA12 = CAN1_TX
     *
     * Alternate Function Push-Pull
     * Output speed = 50 MHz
     *
     * MODE = 11
     * CNF  = 10
     *
     * Configuration = 0xB
     */
    GPIOA->CRH &= ~(0xFU << 16U);
    GPIOA->CRH |=  (0xBU << 16U);

    return STATUS_OK;
}


static Status_t CAN_EnterInitMode(CAN_TypeDef *CANx)
{
    uint32 timeout = 100000U;

    CANx->MCR |= CAN_MCR_INRQ;

    while((CANx->MSR & CAN_MSR_INAK) == 0U)
    {
        timeout--;

        if(timeout == 0U)
        {
            return STATUS_TIMEOUT;
        }
    }

    return STATUS_OK;
}


static Status_t CAN_LeaveInitMode(CAN_TypeDef *CANx)
{
    uint32 timeout = 100000U;

    CANx->MCR &= ~CAN_MCR_INRQ;

    while((CANx->MSR & CAN_MSR_INAK) != 0U)
    {
        timeout--;

        if(timeout == 0U)
        {
            return STATUS_TIMEOUT;
        }
    }

    return STATUS_OK;
}


static Status_t CAN_ConfigureBitTiming(
    CAN_TypeDef *CANx,
    CAN_Baudrate_t baudrate)
{
    uint32 brp;
    uint32 ts1;
    uint32 ts2;

    switch(baudrate)
    {
        case CAN_BAUDRATE_500K:

            brp = 4U;
            ts1 = 13U;
            ts2 = 4U;

            break;

        case CAN_BAUDRATE_250K:

            brp = 8U;
            ts1 = 13U;
            ts2 = 4U;

            break;


        case CAN_BAUDRATE_125K:

            brp = 16U;
            ts1 = 13U;
            ts2 = 4U;

            break;


        case CAN_BAUDRATE_1M:

            brp = 2U;
            ts1 = 13U;
            ts2 = 4U;

            break;


        default:

            return STATUS_INVALID_PARAM;
    }

    CANx->BTR = 0U;

    CANx->BTR |= ((brp - 1U) & 0x3FFU);

    CANx->BTR |= (((ts1 - 1U) & 0x0FU) << 16U);

    CANx->BTR |= (((ts2 - 1U) & 0x07U) << 20U);

    return STATUS_OK;
}


/*  Public Functions */

Status_t DRV_CAN_Init(
    CAN_t can,
    CAN_Baudrate_t baudrate)
{
    CAN_TypeDef *CANx;

    CANx = CAN_GetInstance(can);

    if(CANx == (CAN_TypeDef *)0)
    {
        return STATUS_INVALID_PARAM;
    }

    /* Enable peripheral clocks */
    if(CAN_EnableClock(can) != STATUS_OK)
    {
        return STATUS_ERROR;
    }

    /* Configure CAN GPIO */
    if(CAN_GPIO_Init(can) != STATUS_OK)
    {
        return STATUS_ERROR;
    }

    /* Enter initialization mode */
    if(CAN_EnterInitMode(CANx) != STATUS_OK)
    {
        return STATUS_TIMEOUT;
    }

    /* Disable automatic bus-off management.
       The application can handle bus-off explicitly.
    */
    CANx->MCR &= ~CAN_MCR_ABOM;

    /* Automatic retransmission enabled */
    CANx->MCR &= ~CAN_MCR_NART;

    /* Normal CAN mode */
    CANx->BTR &= ~CAN_BTR_SILM;
    CANx->BTR &= ~CAN_BTR_LBKM;

    /* Configure bit timing */
    if(CAN_ConfigureBitTiming(CANx, baudrate) != STATUS_OK)
    {
        CANx->MCR &= ~CAN_MCR_INRQ;

        return STATUS_INVALID_PARAM;
    }

   
    CANx->FMR |= CAN_FMR_FINIT;

    CANx->FM1R &= ~(1U << CAN_DEFAULT_FILTER);

    CANx->FS1R |= (1U << CAN_DEFAULT_FILTER);

    CANx->FFA1R &= ~(1U << CAN_DEFAULT_FILTER);

    CANx->FA1R |= (1U << CAN_DEFAULT_FILTER);

    CANx->sFilterRegister[CAN_DEFAULT_FILTER].FR1 = 0U;
    CANx->sFilterRegister[CAN_DEFAULT_FILTER].FR2 = 0U;

    CANx->FMR &= ~CAN_FMR_FINIT;

    /* Leave initialization mode */
    if(CAN_LeaveInitMode(CANx) != STATUS_OK)
    {
        return STATUS_TIMEOUT;
    }

    return STATUS_OK;
}


/*  CAN Send */

Status_t DRV_CAN_Send(
    CAN_t can,
    const CAN_Message_t *message)
{
    CAN_TypeDef *CANx;
    uint8 mailbox;
    uint32 timeout = 100000U;

    if(message == (const CAN_Message_t *)0)
    {
        return STATUS_INVALID_PARAM;
    }

    if(message->dlc > 8U)
    {
        return STATUS_INVALID_PARAM;
    }

    CANx = CAN_GetInstance(can);

    if(CANx == (CAN_TypeDef *)0)
    {
        return STATUS_INVALID_PARAM;
    }

    /* Check whether a transmit mailbox is available */
    while((CANx->TSR & CAN_TSR_TME0) == 0U &&
          (CANx->TSR & CAN_TSR_TME1) == 0U &&
          (CANx->TSR & CAN_TSR_TME2) == 0U)
    {
        timeout--;

        if(timeout == 0U)
        {
            return STATUS_TIMEOUT;
        }
    }

    /* Select first available mailbox */
    if((CANx->TSR & CAN_TSR_TME0) != 0U)
    {
        mailbox = 0U;
    }
    else if((CANx->TSR & CAN_TSR_TME1) != 0U)
    {
        mailbox = 1U;
    }
    else
    {
        mailbox = 2U;
    }

    /* Configure identifier */
    if(message->idType == CAN_ID_STANDARD)
    {
        CANx->sTxMailBox[mailbox].TIR =
            ((message->id & 0x7FFU) << 21U);
    }
    else
    {
        CANx->sTxMailBox[mailbox].TIR =
            ((message->id & 0x1FFFFFFFU) << 3U);

        CANx->sTxMailBox[mailbox].TIR |=
            CAN_TI0R_IDE;
    }

    /* Remote frame */
    if(message->frameType == CAN_FRAME_REMOTE)
    {
        CANx->sTxMailBox[mailbox].TIR |=
            CAN_TI0R_RTR;
    }

    /* DLC  */
    CANx->sTxMailBox[mailbox].TDTR =
        (message->dlc & 0x0FU);

    /* Clear data registers */
    CANx->sTxMailBox[mailbox].TDLR = 0U;
    CANx->sTxMailBox[mailbox].TDHR = 0U;

    /* Pack data bytes */
    if(message->dlc > 0U)
    {
        CANx->sTxMailBox[mailbox].TDLR |=
            ((uint32)message->data[0] << 0U);

        if(message->dlc > 1U)
        {
            CANx->sTxMailBox[mailbox].TDLR |=
                ((uint32)message->data[1] << 8U);
        }

        if(message->dlc > 2U)
        {
            CANx->sTxMailBox[mailbox].TDLR |=
                ((uint32)message->data[2] << 16U);
        }

        if(message->dlc > 3U)
        {
            CANx->sTxMailBox[mailbox].TDLR |=
                ((uint32)message->data[3] << 24U);
        }

        if(message->dlc > 4U)
        {
            CANx->sTxMailBox[mailbox].TDHR |=
                ((uint32)message->data[4] << 0U);
        }

        if(message->dlc > 5U)
        {
            CANx->sTxMailBox[mailbox].TDHR |=
                ((uint32)message->data[5] << 8U);
        }

        if(message->dlc > 6U)
        {
            CANx->sTxMailBox[mailbox].TDHR |=
                ((uint32)message->data[6] << 16U);
        }

        if(message->dlc > 7U)
        {
            CANx->sTxMailBox[mailbox].TDHR |=
                ((uint32)message->data[7] << 24U);
        }
    }

    /* Request transmission */
    CANx->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;

    /* Wait until transmission completes */
    timeout = 100000U;

    while(timeout > 0U)
    {
        uint32 rqcp;

        if(mailbox == 0U)
        {
            rqcp = CAN_TSR_RQCP0;
        }
        else if(mailbox == 1U)
        {
            rqcp = CAN_TSR_RQCP1;
        }
        else
        {
            rqcp = CAN_TSR_RQCP2;
        }

        if((CANx->TSR & rqcp) != 0U)
        {
            /*
             * Clear request complete flag.
             */
            CANx->TSR |= rqcp;

            /*
             * Check transmission error.
             */
            if(mailbox == 0U)
            {
                if((CANx->TSR & CAN_TSR_TXOK0) == 0U)
                {
                    return STATUS_ERROR;
                }
            }
            else if(mailbox == 1U)
            {
                if((CANx->TSR & CAN_TSR_TXOK1) == 0U)
                {
                    return STATUS_ERROR;
                }
            }
            else
            {
                if((CANx->TSR & CAN_TSR_TXOK2) == 0U)
                {
                    return STATUS_ERROR;
                }
            }

            return STATUS_OK;
        }

        timeout--;
    }

    return STATUS_TIMEOUT;
}


/* CAN Receive */

Status_t DRV_CAN_Receive(
    CAN_t can,
    CAN_Message_t *message)
{
    CAN_TypeDef *CANx;
    uint32 rir;
    uint32 rdtr;
    uint32 rdlr;
    uint32 rdhr;

    if(message == (CAN_Message_t *)0)
    {
        return STATUS_INVALID_PARAM;
    }

    CANx = CAN_GetInstance(can);

    if(CANx == (CAN_TypeDef *)0)
    {
        return STATUS_INVALID_PARAM;
    }

    /*
     * Check FIFO0.
     */
    if((CANx->RF0R & CAN_RF0R_FMP0) == 0U)
    {
        return STATUS_BUSY;
    }

    /*
     * Read FIFO mailbox.
     */
    rir  = CANx->sFIFOMailBox[0].RIR;
    rdtr = CANx->sFIFOMailBox[0].RDTR;
    rdlr = CANx->sFIFOMailBox[0].RDLR;
    rdhr = CANx->sFIFOMailBox[0].RDHR;

    /*
     * Determine ID type.
     */
    if((rir & CAN_RI0R_IDE) == 0U)
    {
        message->idType = CAN_ID_STANDARD;
        message->id = (rir >> 21U) & 0x7FFU;
    }
    else
    {
        message->idType = CAN_ID_EXTENDED;
        message->id = (rir >> 3U) & 0x1FFFFFFFU;
    }

    /*
     * Determine frame type.
     */
    if((rir & CAN_RI0R_RTR) != 0U)
    {
        message->frameType = CAN_FRAME_REMOTE;
    }
    else
    {
        message->frameType = CAN_FRAME_DATA;
    }

    /*
     * DLC.
     */
    message->dlc = (uint8)(rdtr & 0x0FU);

    if(message->dlc > 8U)
    {
        message->dlc = 8U;
    }

    /*
     * Unpack data.
     */
    if(message->dlc > 0U)
    {
        message->data[0] = (uint8)(rdlr >> 0U);

        if(message->dlc > 1U)
        {
            message->data[1] = (uint8)(rdlr >> 8U);
        }

        if(message->dlc > 2U)
        {
            message->data[2] = (uint8)(rdlr >> 16U);
        }

        if(message->dlc > 3U)
        {
            message->data[3] = (uint8)(rdlr >> 24U);
        }

        if(message->dlc > 4U)
        {
            message->data[4] = (uint8)(rdhr >> 0U);
        }

        if(message->dlc > 5U)
        {
            message->data[5] = (uint8)(rdhr >> 8U);
        }

        if(message->dlc > 6U)
        {
            message->data[6] = (uint8)(rdhr >> 16U);
        }

        if(message->dlc > 7U)
        {
            message->data[7] = (uint8)(rdhr >> 24U);
        }
    }

    /*
     * Release FIFO0 output mailbox.
     */
    CANx->RF0R |= CAN_RF0R_RFOM0;

    return STATUS_OK;
}


/*  CAN Filter */

Status_t DRV_CAN_SetFilter(
    CAN_t can,
    uint16 filterId,
    uint16 filterMask)
{
    CAN_TypeDef *CANx;

    CANx = CAN_GetInstance(can);

    if(CANx == (CAN_TypeDef *)0)
    {
        return STATUS_INVALID_PARAM;
    }

   
    CANx->FMR |= CAN_FMR_FINIT;

    CANx->FM1R &= ~(1U << 0U);

    CANx->FS1R |= (1U << 0U);

    CANx->FFA1R &= ~(1U << 0U);

    CANx->sFilterRegister[0].FR1 =
        ((uint32)(filterId & 0x7FFU) << 21U);

    CANx->sFilterRegister[0].FR2 =
        ((uint32)(filterMask & 0x7FFU) << 21U);

    CANx->FA1R |= (1U << 0U);

    CANx->FMR &= ~CAN_FMR_FINIT;

    return STATUS_OK;
}


Status_t DRV_CAN_Reset(CAN_t can)
{
    CAN_TypeDef *CANx;

    CANx = CAN_GetInstance(can);

    if(CANx == (CAN_TypeDef *)0)
    {
        return STATUS_INVALID_PARAM;
    }

    /* Enter initialization mode */
    if(CAN_EnterInitMode(CANx) != STATUS_OK)
    {
        return STATUS_TIMEOUT;
    }

    /* Reset CAN peripheral registers */
    CANx->MCR |= CAN_MCR_RESET;

    /*  Leave initialization mode */
    if(CAN_LeaveInitMode(CANx) != STATUS_OK)
    {
        return STATUS_TIMEOUT;
    }

    return STATUS_OK;
}
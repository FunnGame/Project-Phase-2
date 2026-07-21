/**
 ******************************************************************************
 * @file    uart.c
 * @brief   USART/UART driver implementation (STM32F446RE).
 ******************************************************************************
 */
#include "uart.h"
#include "rcc.h"

/* Busy-wait budget for a single character's status flag. */
#define UART_FLAG_TIMEOUT   0x00100000UL

/* RX callback registry, one slot per supported instance. */
#define UART_MAX_REGISTERED 6

typedef struct {
    USART_TypeDef     *usart;
    uart_rx_callback_t cb;
    void              *ctx;
} uart_slot_t;

static uart_slot_t s_slots[UART_MAX_REGISTERED];

static uart_slot_t *uart_slot_find(USART_TypeDef *usart)
{
    for (uint32_t i = 0; i < UART_MAX_REGISTERED; ++i) {
        if (s_slots[i].usart == usart) return &s_slots[i];
    }
    return NULL;
}

static uart_slot_t *uart_slot_alloc(USART_TypeDef *usart)
{
    uart_slot_t *s = uart_slot_find(usart);
    if (s != NULL) return s;
    for (uint32_t i = 0; i < UART_MAX_REGISTERED; ++i) {
        if (s_slots[i].usart == NULL) { s_slots[i].usart = usart; return &s_slots[i]; }
    }
    return NULL;
}

/* True for instances clocked from APB2 (USART1, USART6). */
static bool uart_on_apb2(USART_TypeDef *usart)
{
    return (usart == USART1) || (usart == USART6);
}

static bool uart_clock_enable(USART_TypeDef *usart)
{
    if      (usart == USART1) rcc_apb2_enable(RCC_APB2ENR_USART1EN);
    else if (usart == USART2) rcc_apb1_enable(RCC_APB1ENR_USART2EN);
    else if (usart == USART3) rcc_apb1_enable(RCC_APB1ENR_USART3EN);
    else if (usart == UART4)  rcc_apb1_enable(RCC_APB1ENR_UART4EN);
    else if (usart == UART5)  rcc_apb1_enable(RCC_APB1ENR_UART5EN);
    else if (usart == USART6) rcc_apb2_enable(RCC_APB2ENR_USART6EN);
    else return false;
    return true;
}

static IRQn_Type uart_irqn(USART_TypeDef *usart)
{
    if (usart == USART1) return USART1_IRQn;
    if (usart == USART2) return USART2_IRQn;
    if (usart == USART3) return USART3_IRQn;
    if (usart == UART4)  return UART4_IRQn;
    if (usart == UART5)  return UART5_IRQn;
    return USART6_IRQn; /* USART6 */
}

/* -------------------------------------------------------------------------- */
/*  Configuration                                                             */
/* -------------------------------------------------------------------------- */

drv_status_t uart_init(USART_TypeDef *usart, const uart_config_t *cfg)
{
    if (usart == NULL || cfg == NULL || cfg->baud == 0U) {
        return DRV_INVALID_PARAM;
    }
    if (!uart_clock_enable(usart)) {
        return DRV_INVALID_PARAM;
    }

    /* Disable while reconfiguring. */
    usart->CR1 = 0U;
    usart->CR2 = 0U;
    usart->CR3 = 0U;

    /* Baud rate for 16x oversampling: BRR = round(f_ck / baud). */
    const uint32_t pclk = uart_on_apb2(usart) ? rcc_get_pclk2_hz()
                                              : rcc_get_pclk1_hz();
    usart->BRR = (pclk + (cfg->baud / 2U)) / cfg->baud;

    uint32_t cr1 = 0U;
    if (cfg->wordlen == UART_WORDLEN_9)  cr1 |= USART_CR1_M;
    if (cfg->parity != UART_PARITY_NONE) cr1 |= USART_CR1_PCE;
    if (cfg->parity == UART_PARITY_ODD)  cr1 |= USART_CR1_PS;
    if (cfg->direction & UART_DIR_TX)    cr1 |= USART_CR1_TE;
    if (cfg->direction & UART_DIR_RX)    cr1 |= USART_CR1_RE;

    usart->CR2 = ((uint32_t)cfg->stopbits << USART_CR2_STOP_Pos) & USART_CR2_STOP;
    usart->CR1 = cr1 | USART_CR1_UE;
    return DRV_OK;
}

drv_status_t uart_init_8n1(USART_TypeDef *usart, uint32_t baud)
{
    const uart_config_t cfg = {
        .baud      = baud,
        .wordlen   = UART_WORDLEN_8,
        .parity    = UART_PARITY_NONE,
        .stopbits  = UART_STOPBITS_1,
        .direction = UART_DIR_TXRX,
    };
    return uart_init(usart, &cfg);
}

/* -------------------------------------------------------------------------- */
/*  Blocking I/O                                                              */
/* -------------------------------------------------------------------------- */

void uart_write_byte(USART_TypeDef *usart, uint8_t byte)
{
    (void)drv_wait_flag(&usart->SR, USART_SR_TXE, USART_SR_TXE, UART_FLAG_TIMEOUT);
    usart->DR = (uint32_t)byte & 0x1FFU;
}

void uart_write(USART_TypeDef *usart, const uint8_t *data, size_t len)
{
    if (data == NULL) return;
    for (size_t i = 0; i < len; ++i) {
        uart_write_byte(usart, data[i]);
    }
    /* Block until the last frame has fully shifted out. */
    (void)drv_wait_flag(&usart->SR, USART_SR_TC, USART_SR_TC, UART_FLAG_TIMEOUT);
}

void uart_write_string(USART_TypeDef *usart, const char *str)
{
    if (str == NULL) return;
    while (*str != '\0') {
        uart_write_byte(usart, (uint8_t)*str++);
    }
    (void)drv_wait_flag(&usart->SR, USART_SR_TC, USART_SR_TC, UART_FLAG_TIMEOUT);
}

bool uart_rx_ready(USART_TypeDef *usart)
{
    return (usart->SR & USART_SR_RXNE) != 0U;
}

uint8_t uart_read_byte(USART_TypeDef *usart)
{
    (void)drv_wait_flag(&usart->SR, USART_SR_RXNE, USART_SR_RXNE, UART_FLAG_TIMEOUT);
    return (uint8_t)(usart->DR & 0xFFU);
}

size_t uart_read(USART_TypeDef *usart, uint8_t *buf, size_t len)
{
    if (buf == NULL) return 0U;
    for (size_t i = 0; i < len; ++i) {
        buf[i] = uart_read_byte(usart);
    }
    return len;
}

/* -------------------------------------------------------------------------- */
/*  RX interrupt                                                              */
/* -------------------------------------------------------------------------- */

drv_status_t uart_attach_rx_irq(USART_TypeDef *usart, uart_rx_callback_t cb,
                                void *ctx, uint32_t priority)
{
    if (usart == NULL) {
        return DRV_INVALID_PARAM;
    }
    uart_slot_t *slot = uart_slot_alloc(usart);
    if (slot == NULL) {
        return DRV_ERROR;
    }
    slot->cb  = cb;
    slot->ctx = ctx;

    if (cb != NULL) {
        DRV_SET_BITS(usart->CR1, USART_CR1_RXNEIE);
        nvic_enable(uart_irqn(usart), priority);
    } else {
        DRV_CLEAR_BITS(usart->CR1, USART_CR1_RXNEIE);
    }
    return DRV_OK;
}

void uart_irq_dispatch(USART_TypeDef *usart)
{
    if ((usart->SR & USART_SR_RXNE) == 0U) {
        return;
    }
    /* Reading DR clears RXNE (and the ORE overrun flag alongside SR read). */
    const uint8_t byte = (uint8_t)(usart->DR & 0xFFU);

    uart_slot_t *slot = uart_slot_find(usart);
    if (slot != NULL && slot->cb != NULL) {
        slot->cb(usart, byte, slot->ctx);
    }
}

/* -------------------------------------------------------------------------- */
/*  Default vector handlers                                                    */
/* -------------------------------------------------------------------------- */
#if UART_PROVIDE_IRQ_HANDLERS

void USART1_IRQHandler(void) { uart_irq_dispatch(USART1); }
void USART2_IRQHandler(void) { uart_irq_dispatch(USART2); }
void USART3_IRQHandler(void) { uart_irq_dispatch(USART3); }
void UART4_IRQHandler(void)  { uart_irq_dispatch(UART4); }
void UART5_IRQHandler(void)  { uart_irq_dispatch(UART5); }
void USART6_IRQHandler(void) { uart_irq_dispatch(USART6); }

#endif /* UART_PROVIDE_IRQ_HANDLERS */

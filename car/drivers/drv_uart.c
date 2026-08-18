/**
 ******************************************************************************
 * @file    drv_uart.c
 * @brief   USART driver implementation (STM32F103).
 ******************************************************************************
 */
#include "drv_uart.h"
#include "drv_clock.h"
#include "drv_gpio.h"

/* Resolve an instance to its pins, port, bus clock and clock-enable. */
typedef struct {
    GPIO_TypeDef *port;
    uint8_t       tx_pin;
    uint8_t       rx_pin;
    uint32_t      bus_clk;     /* PCLK feeding this USART */
} uart_map_t;

static DRV_Status uart_resolve(const UART_Config *cfg, uart_map_t *m)
{
    if (cfg->uart == USART1) {
        DRV_SET_BITS(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
        if (cfg->remap) {
            DRV_SET_BITS(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
            DRV_SET_BITS(AFIO->MAPR, AFIO_MAPR_USART1_REMAP);
            m->port = GPIOB; m->tx_pin = 6u; m->rx_pin = 7u;
        } else {
            m->port = GPIOA; m->tx_pin = 9u; m->rx_pin = 10u;
        }
        m->bus_clk = DRV_Clock_GetPCLK2();      /* USART1 is on APB2 */
    } else if (cfg->uart == USART2) {
        DRV_SET_BITS(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
        m->port = GPIOA; m->tx_pin = 2u; m->rx_pin = 3u;
        m->bus_clk = DRV_Clock_GetPCLK1();
    } else if (cfg->uart == USART3) {
        DRV_SET_BITS(RCC->APB1ENR, RCC_APB1ENR_USART3EN);
        m->port = GPIOB; m->tx_pin = 10u; m->rx_pin = 11u;
        m->bus_clk = DRV_Clock_GetPCLK1();
    } else {
        return DRV_INVALID_PARAM;
    }
    return DRV_OK;
}

DRV_Status DRV_UART_Init(const UART_Config *cfg)
{
    if (cfg == NULL || cfg->uart == NULL || cfg->baud == 0u) {
        return DRV_INVALID_PARAM;
    }

    uart_map_t m;
    const DRV_Status st = uart_resolve(cfg, &m);
    if (st != DRV_OK) {
        return st;
    }

    /* TX = alternate-function push-pull; RX = input with pull-up (idle high). */
    DRV_GPIO_InitAF(m.port, m.tx_pin);
    DRV_GPIO_InitInput(m.port, m.rx_pin, GPIO_PULL_UP);

    USART_TypeDef *u = cfg->uart;

    /* BRR holds the 12.4 fixed-point divider, which equals f_ck / baud for
     * oversampling by 16. Round to the nearest divider. */
    const uint32_t brr = (m.bus_clk + (cfg->baud / 2u)) / cfg->baud;
    if (brr < 0x10u || brr > 0xFFFFu) {         /* divider must be >= 1.0 */
        return DRV_UNSUPPORTED;
    }

    u->CR1 = 0u;                                 /* 8 data bits, no parity */
    u->CR2 = 0u;                                 /* 1 stop bit             */
    u->CR3 = 0u;                                 /* no flow control        */
    u->BRR = (uint16_t)brr;

    DRV_SET_BITS(u->CR1, USART_CR1_TE | USART_CR1_RE);
    DRV_SET_BITS(u->CR1, USART_CR1_UE);          /* enable last */
    return DRV_OK;
}

void DRV_UART_WriteByte(USART_TypeDef *uart, uint8_t byte)
{
    while ((uart->SR & USART_SR_TXE) == 0u) {
        /* wait for the transmit data register to empty */
    }
    uart->DR = (uint16_t)byte;
}

void DRV_UART_Write(USART_TypeDef *uart, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; ++i) {
        DRV_UART_WriteByte(uart, data[i]);
    }
}

void DRV_UART_WriteString(USART_TypeDef *uart, const char *str)
{
    while (*str != '\0') {
        DRV_UART_WriteByte(uart, (uint8_t)*str++);
    }
}

bool DRV_UART_DataAvailable(USART_TypeDef *uart)
{
    return (uart->SR & USART_SR_RXNE) != 0u;
}

uint8_t DRV_UART_ReadByte(USART_TypeDef *uart)
{
    while ((uart->SR & USART_SR_RXNE) == 0u) {
        /* wait for a received byte */
    }
    return (uint8_t)(uart->DR & 0xFFu);
}

/**
 ******************************************************************************
 * @file    exti.c
 * @brief   External interrupt / event driver implementation (STM32F446RE).
 ******************************************************************************
 */
#include "exti.h"
#include "rcc.h"

/* One callback slot per EXTI line (0..15). */
typedef struct {
    exti_callback_t cb;
    void           *ctx;
} exti_slot_t;

static exti_slot_t s_slots[16];

/* Map an EXTI line to its NVIC vector. Lines 5..9 and 10..15 are shared. */
static IRQn_Type exti_line_to_irqn(uint8_t line)
{
    switch (line) {
        case 0:  return EXTI0_IRQn;
        case 1:  return EXTI1_IRQn;
        case 2:  return EXTI2_IRQn;
        case 3:  return EXTI3_IRQn;
        case 4:  return EXTI4_IRQn;
        default: break;
    }
    if (line <= 9U)  return EXTI9_5_IRQn;
    return EXTI15_10_IRQn;
}

/* SYSCFG_EXTICR port encoding: GPIOA=0, GPIOB=1, ... GPIOH=7. */
static int32_t exti_port_code(GPIO_TypeDef *port)
{
    if      (port == GPIOA) return 0;
    else if (port == GPIOB) return 1;
    else if (port == GPIOC) return 2;
    else if (port == GPIOD) return 3;
    else if (port == GPIOE) return 4;
    else if (port == GPIOF) return 5;
    else if (port == GPIOG) return 6;
    else if (port == GPIOH) return 7;
    return -1;
}

drv_status_t exti_config_line(uint8_t line, const exti_line_config_t *cfg)
{
    if (line > 15U || cfg == NULL) {
        return DRV_INVALID_PARAM;
    }
    const int32_t port_code = exti_port_code(cfg->port);
    if (port_code < 0) {
        return DRV_INVALID_PARAM;
    }

    /* SYSCFG must be clocked to route the GPIO bank onto the EXTI line. */
    rcc_apb2_enable(RCC_APB2ENR_SYSCFGEN);

    /* Route port -> line via SYSCFG->EXTICR[line/4], 4 bits per line. */
    const uint32_t reg = (uint32_t)line >> 2U;
    const uint32_t pos = ((uint32_t)line & 0x3U) * 4U;
    DRV_MODIFY(SYSCFG->EXTICR[reg], (0xFUL << pos),
               ((uint32_t)port_code << pos));

    const uint32_t line_bit = DRV_BIT(line);

    /* Edge selection. */
    if (cfg->trigger == EXTI_TRIGGER_RISING || cfg->trigger == EXTI_TRIGGER_BOTH) {
        DRV_SET_BITS(EXTI->RTSR, line_bit);
    } else {
        DRV_CLEAR_BITS(EXTI->RTSR, line_bit);
    }
    if (cfg->trigger == EXTI_TRIGGER_FALLING || cfg->trigger == EXTI_TRIGGER_BOTH) {
        DRV_SET_BITS(EXTI->FTSR, line_bit);
    } else {
        DRV_CLEAR_BITS(EXTI->FTSR, line_bit);
    }

    /* Register the callback before unmasking to avoid a race. */
    s_slots[line].cb  = cfg->callback;
    s_slots[line].ctx = cfg->ctx;

    /* Clear a possibly-stale pending flag, then unmask the interrupt request. */
    EXTI->PR = line_bit;
    DRV_SET_BITS(EXTI->IMR, line_bit);

    if (cfg->enable_nvic) {
        nvic_enable(exti_line_to_irqn(line), cfg->priority);
    }
    return DRV_OK;
}

void exti_enable_line(uint8_t line)
{
    if (line <= 15U) DRV_SET_BITS(EXTI->IMR, DRV_BIT(line));
}

void exti_disable_line(uint8_t line)
{
    if (line <= 15U) DRV_CLEAR_BITS(EXTI->IMR, DRV_BIT(line));
}

bool exti_is_pending(uint8_t line)
{
    return (line <= 15U) && ((EXTI->PR & DRV_BIT(line)) != 0U);
}

void exti_clear_pending(uint8_t line)
{
    if (line <= 15U) EXTI->PR = DRV_BIT(line); /* write-1-to-clear */
}

void exti_software_trigger(uint8_t line)
{
    if (line <= 15U) DRV_SET_BITS(EXTI->SWIER, DRV_BIT(line));
}

void exti_irq_dispatch(uint8_t line_first, uint8_t line_last)
{
    if (line_last > 15U) line_last = 15U;
    for (uint8_t line = line_first; line <= line_last; ++line) {
        const uint32_t bit = DRV_BIT(line);
        if ((EXTI->PR & bit) == 0U) {
            continue;
        }
        EXTI->PR = bit; /* clear before callback so re-fires are not lost */
        if (s_slots[line].cb != NULL) {
            s_slots[line].cb(line, s_slots[line].ctx);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  Default vector handlers                                                    */
/* -------------------------------------------------------------------------- */
#if EXTI_PROVIDE_IRQ_HANDLERS

void EXTI0_IRQHandler(void)     { exti_irq_dispatch(0, 0); }
void EXTI1_IRQHandler(void)     { exti_irq_dispatch(1, 1); }
void EXTI2_IRQHandler(void)     { exti_irq_dispatch(2, 2); }
void EXTI3_IRQHandler(void)     { exti_irq_dispatch(3, 3); }
void EXTI4_IRQHandler(void)     { exti_irq_dispatch(4, 4); }
void EXTI9_5_IRQHandler(void)   { exti_irq_dispatch(5, 9); }
void EXTI15_10_IRQHandler(void) { exti_irq_dispatch(10, 15); }

#endif /* EXTI_PROVIDE_IRQ_HANDLERS */

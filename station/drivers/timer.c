/**
 ******************************************************************************
 * @file    timer.c
 * @brief   General-purpose timer driver implementation (STM32F446RE).
 ******************************************************************************
 */
#include "timer.h"
#include "rcc.h"

/* Small registry so a shared ISR can find the right callback. */
#define TIMER_MAX_REGISTERED 8

typedef struct {
    TIM_TypeDef     *tim;
    timer_callback_t cb;
    void            *ctx;
} timer_slot_t;

static timer_slot_t s_slots[TIMER_MAX_REGISTERED];

static timer_slot_t *timer_slot_find(TIM_TypeDef *tim)
{
    for (uint32_t i = 0; i < TIMER_MAX_REGISTERED; ++i) {
        if (s_slots[i].tim == tim) return &s_slots[i];
    }
    return NULL;
}

static timer_slot_t *timer_slot_alloc(TIM_TypeDef *tim)
{
    timer_slot_t *s = timer_slot_find(tim);
    if (s != NULL) return s;
    for (uint32_t i = 0; i < TIMER_MAX_REGISTERED; ++i) {
        if (s_slots[i].tim == NULL) { s_slots[i].tim = tim; return &s_slots[i]; }
    }
    return NULL;
}

/* True for timers clocked from APB2 (TIM1, TIM8..TIM11). */
static bool timer_on_apb2(TIM_TypeDef *tim)
{
    return (tim == TIM1) || (tim == TIM8) || (tim == TIM9) ||
           (tim == TIM10) || (tim == TIM11);
}

/* Enable the timer's peripheral clock; returns false for unknown instances. */
static bool timer_clock_enable(TIM_TypeDef *tim)
{
    if      (tim == TIM1)  rcc_apb2_enable(RCC_APB2ENR_TIM1EN);
    else if (tim == TIM2)  rcc_apb1_enable(RCC_APB1ENR_TIM2EN);
    else if (tim == TIM3)  rcc_apb1_enable(RCC_APB1ENR_TIM3EN);
    else if (tim == TIM4)  rcc_apb1_enable(RCC_APB1ENR_TIM4EN);
    else if (tim == TIM5)  rcc_apb1_enable(RCC_APB1ENR_TIM5EN);
    else if (tim == TIM6)  rcc_apb1_enable(RCC_APB1ENR_TIM6EN);
    else if (tim == TIM7)  rcc_apb1_enable(RCC_APB1ENR_TIM7EN);
    else if (tim == TIM8)  rcc_apb2_enable(RCC_APB2ENR_TIM8EN);
    else if (tim == TIM9)  rcc_apb2_enable(RCC_APB2ENR_TIM9EN);
    else if (tim == TIM10) rcc_apb2_enable(RCC_APB2ENR_TIM10EN);
    else if (tim == TIM11) rcc_apb2_enable(RCC_APB2ENR_TIM11EN);
    else if (tim == TIM12) rcc_apb1_enable(RCC_APB1ENR_TIM12EN);
    else if (tim == TIM13) rcc_apb1_enable(RCC_APB1ENR_TIM13EN);
    else if (tim == TIM14) rcc_apb1_enable(RCC_APB1ENR_TIM14EN);
    else return false;
    return true;
}

/* Map a timer to the NVIC vector that carries its UPDATE event. */
static IRQn_Type timer_update_irqn(TIM_TypeDef *tim)
{
    if (tim == TIM1)  return TIM1_UP_TIM10_IRQn;
    if (tim == TIM2)  return TIM2_IRQn;
    if (tim == TIM3)  return TIM3_IRQn;
    if (tim == TIM4)  return TIM4_IRQn;
    if (tim == TIM5)  return TIM5_IRQn;
    if (tim == TIM6)  return TIM6_DAC_IRQn;
    if (tim == TIM7)  return TIM7_IRQn;
    if (tim == TIM8)  return TIM8_UP_TIM13_IRQn;
    if (tim == TIM9)  return TIM1_BRK_TIM9_IRQn;
    if (tim == TIM10) return TIM1_UP_TIM10_IRQn;
    if (tim == TIM11) return TIM1_TRG_COM_TIM11_IRQn;
    if (tim == TIM12) return TIM8_BRK_TIM12_IRQn;
    if (tim == TIM13) return TIM8_UP_TIM13_IRQn;
    return TIM8_TRG_COM_TIM14_IRQn; /* TIM14 */
}

/* -------------------------------------------------------------------------- */
/*  Time base                                                                 */
/* -------------------------------------------------------------------------- */

drv_status_t timer_init(TIM_TypeDef *tim, const timer_base_config_t *cfg)
{
    if (tim == NULL || cfg == NULL) {
        return DRV_INVALID_PARAM;
    }
    if (!timer_clock_enable(tim)) {
        return DRV_INVALID_PARAM;
    }

    tim->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_ARPE | TIM_CR1_OPM | TIM_CR1_CMS);
    if (cfg->direction == TIMER_COUNT_DOWN)   tim->CR1 |= TIM_CR1_DIR;
    if (cfg->auto_reload_preload)             tim->CR1 |= TIM_CR1_ARPE;
    if (cfg->one_pulse)                       tim->CR1 |= TIM_CR1_OPM;

    tim->PSC = cfg->prescaler;
    tim->ARR = cfg->period;
    tim->CNT = 0U;

    /* Generate an update event to load PSC/ARR into the shadow registers,
     * then clear the flag it raised so we don't take a spurious interrupt. */
    tim->EGR = TIM_EGR_UG;
    tim->SR  = ~(uint32_t)TIM_SR_UIF;

    return DRV_OK;
}

drv_status_t timer_init_hz(TIM_TypeDef *tim, uint32_t hz)
{
    if (tim == NULL || hz == 0U) {
        return DRV_INVALID_PARAM;
    }

    const uint32_t f_tim = timer_on_apb2(tim) ? rcc_get_timer_pclk2_hz()
                                              : rcc_get_timer_pclk1_hz();
    uint64_t ticks = (uint64_t)f_tim / hz;      /* (PSC+1) * (ARR+1) */
    if (ticks == 0U) {
        return DRV_INVALID_PARAM;
    }

    /* Smallest prescaler that keeps the period within 16 bits. */
    uint32_t psc_plus = (uint32_t)((ticks + 0xFFFFULL) / 0x10000ULL);
    if (psc_plus == 0U) psc_plus = 1U;
    uint32_t arr_plus = (uint32_t)(ticks / psc_plus);
    if (arr_plus == 0U || psc_plus > 0x10000U) {
        return DRV_UNSUPPORTED;
    }

    const timer_base_config_t cfg = {
        .prescaler           = psc_plus - 1U,
        .period              = arr_plus - 1U,
        .direction           = TIMER_COUNT_UP,
        .auto_reload_preload = true,
        .one_pulse           = false,
    };
    return timer_init(tim, &cfg);
}

void timer_start(TIM_TypeDef *tim) { DRV_SET_BITS(tim->CR1, TIM_CR1_CEN); }
void timer_stop(TIM_TypeDef *tim)  { DRV_CLEAR_BITS(tim->CR1, TIM_CR1_CEN); }

/* -------------------------------------------------------------------------- */
/*  Update interrupt                                                          */
/* -------------------------------------------------------------------------- */

drv_status_t timer_attach_update_irq(TIM_TypeDef *tim, timer_callback_t cb,
                                     void *ctx, uint32_t priority)
{
    if (tim == NULL) {
        return DRV_INVALID_PARAM;
    }
    timer_slot_t *slot = timer_slot_alloc(tim);
    if (slot == NULL) {
        return DRV_ERROR; /* registry full */
    }
    slot->cb  = cb;
    slot->ctx = ctx;

    tim->SR = ~(uint32_t)TIM_SR_UIF;      /* clear stale flag */
    timer_enable_update_irq(tim);
    nvic_enable(timer_update_irqn(tim), priority);
    return DRV_OK;
}

void timer_irq_dispatch(TIM_TypeDef *tim)
{
    if ((tim->SR & TIM_SR_UIF) == 0U) {
        return;
    }
    tim->SR = ~(uint32_t)TIM_SR_UIF;      /* clear update flag */

    timer_slot_t *slot = timer_slot_find(tim);
    if (slot != NULL && slot->cb != NULL) {
        slot->cb(tim, slot->ctx);
    }
}

/* -------------------------------------------------------------------------- */
/*  PWM output                                                                */
/* -------------------------------------------------------------------------- */

/* Access CCRx as an indexable set; the fields are contiguous in the map. */
static volatile uint32_t *timer_ccr(TIM_TypeDef *tim, timer_channel_t ch)
{
    switch (ch) {
        case TIMER_CH1: return &tim->CCR1;
        case TIMER_CH2: return &tim->CCR2;
        case TIMER_CH3: return &tim->CCR3;
        default:        return &tim->CCR4;
    }
}

drv_status_t timer_pwm_config(TIM_TypeDef *tim, timer_channel_t channel,
                              timer_pwm_polarity_t polarity)
{
    if (tim == NULL) {
        return DRV_INVALID_PARAM;
    }

    /* PWM mode 1 (OCxM = 110) with output preload enabled. CH1/CH2 live in
     * CCMR1, CH3/CH4 in CCMR2; the field layout repeats every 8 bits. */
    const uint32_t shift = ((uint32_t)channel & 0x1U) * 8U;
    const uint32_t ocm_pwm1 = (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); /* 110 */
    const uint32_t ocpe     = TIM_CCMR1_OC1PE;
    const uint32_t clr_mask = (0x00FFUL << shift);
    const uint32_t set_mask = ((ocm_pwm1 | ocpe) << shift);

    if (channel == TIMER_CH1 || channel == TIMER_CH2) {
        DRV_MODIFY(tim->CCMR1, clr_mask, set_mask);
    } else {
        DRV_MODIFY(tim->CCMR2, clr_mask, set_mask);
    }

    /* CCER: 4 bits per channel (CCxE, CCxP, CCxNE, CCxNP). */
    const uint32_t ccer_shift = (uint32_t)channel * 4U;
    uint32_t ccer = tim->CCER & ~(0xFUL << ccer_shift);
    if (polarity == TIMER_PWM_ACTIVE_LOW) {
        ccer |= (TIM_CCER_CC1P << ccer_shift);
    }
    tim->CCER = ccer;

    /* Advanced-control timers (TIM1/TIM8) gate outputs behind MOE. */
    if (tim == TIM1 || tim == TIM8) {
        DRV_SET_BITS(tim->BDTR, TIM_BDTR_MOE);
    }
    return DRV_OK;
}

void timer_pwm_set_compare(TIM_TypeDef *tim, timer_channel_t channel,
                           uint32_t compare)
{
    *timer_ccr(tim, channel) = compare;
}

void timer_channel_enable(TIM_TypeDef *tim, timer_channel_t channel)
{
    DRV_SET_BITS(tim->CCER, TIM_CCER_CC1E << ((uint32_t)channel * 4U));
}

void timer_channel_disable(TIM_TypeDef *tim, timer_channel_t channel)
{
    DRV_CLEAR_BITS(tim->CCER, TIM_CCER_CC1E << ((uint32_t)channel * 4U));
}

/* -------------------------------------------------------------------------- */
/*  Default vector handlers                                                    */
/* -------------------------------------------------------------------------- */
#if TIMER_PROVIDE_IRQ_HANDLERS

void TIM2_IRQHandler(void) { timer_irq_dispatch(TIM2); }
void TIM3_IRQHandler(void) { timer_irq_dispatch(TIM3); }
void TIM4_IRQHandler(void) { timer_irq_dispatch(TIM4); }
void TIM5_IRQHandler(void) { timer_irq_dispatch(TIM5); }
void TIM7_IRQHandler(void) { timer_irq_dispatch(TIM7); }

/* TIM6 shares its vector with the DAC underrun interrupt. */
void TIM6_DAC_IRQHandler(void) { timer_irq_dispatch(TIM6); }

/* TIM1 update shares a vector with TIM10; dispatch both. */
void TIM1_UP_TIM10_IRQHandler(void)
{
    timer_irq_dispatch(TIM1);
    timer_irq_dispatch(TIM10);
}

#endif /* TIMER_PROVIDE_IRQ_HANDLERS */

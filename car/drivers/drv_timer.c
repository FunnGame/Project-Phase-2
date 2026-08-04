/**
 ******************************************************************************
 * @file    drv_timer.c
 * @brief   General-purpose timer driver implementation (STM32F103).
 ******************************************************************************
 */
#include "drv_timer.h"
#include "drv_clock.h"

/* ---- Callback registry: lets any timer carry a handler ------------------- */
#define TIMER_MAX_REGISTERED 4

typedef struct {
    TIM_TypeDef       *tim;
    DRV_Timer_Callback cb;
    void              *ctx;
} Timer_Slot;

static Timer_Slot s_slots[TIMER_MAX_REGISTERED];

/* Millisecond tick state (populated by DRV_Timer_StartTick). */
static volatile uint32_t s_tick_ms;

static Timer_Slot *timer_slot_find(TIM_TypeDef *tim)
{
    for (uint32_t i = 0; i < TIMER_MAX_REGISTERED; ++i) {
        if (s_slots[i].tim == tim) return &s_slots[i];
    }
    return NULL;
}

static Timer_Slot *timer_slot_alloc(TIM_TypeDef *tim)
{
    Timer_Slot *s = timer_slot_find(tim);
    if (s != NULL) return s;
    for (uint32_t i = 0; i < TIMER_MAX_REGISTERED; ++i) {
        if (s_slots[i].tim == NULL) { s_slots[i].tim = tim; return &s_slots[i]; }
    }
    return NULL;
}

/* Enable the peripheral clock; false for an instance we do not know. */
static bool timer_clock_enable(TIM_TypeDef *tim)
{
    if      (tim == TIM1) DRV_SET_BITS(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
    else if (tim == TIM2) DRV_SET_BITS(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
    else if (tim == TIM3) DRV_SET_BITS(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
    else if (tim == TIM4) DRV_SET_BITS(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);
    else return false;
    return true;
}

/* Map a timer to the NVIC vector carrying its UPDATE event. */
static IRQn_Type timer_update_irqn(TIM_TypeDef *tim)
{
    if (tim == TIM1) return TIM1_UP_IRQn;
    if (tim == TIM2) return TIM2_IRQn;
    if (tim == TIM3) return TIM3_IRQn;
    return TIM4_IRQn;
}

/* -------------------------------------------------------------------------- */
/*  Time base                                                                 */
/* -------------------------------------------------------------------------- */

DRV_Status DRV_Timer_InitRaw(TIM_TypeDef *tim, uint16_t psc, uint16_t arr)
{
    if (tim == NULL || !timer_clock_enable(tim)) {
        return DRV_INVALID_PARAM;
    }

    tim->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);   /* upcounting, edge-aligned */
    tim->CR1 |= TIM_CR1_ARPE;                   /* buffer ARR */
    tim->PSC = psc;
    tim->ARR = arr;
    tim->CNT = 0u;

    /* Load PSC/ARR into their shadow registers now, then drop the update flag
     * that the software event raised so no spurious interrupt fires. */
    tim->EGR = TIM_EGR_UG;
    tim->SR  = (uint32_t)~TIM_SR_UIF;

    return DRV_OK;
}

DRV_Status DRV_Timer_InitHz(TIM_TypeDef *tim, uint32_t freq_hz)
{
    if (tim == NULL || freq_hz == 0u) {
        return DRV_INVALID_PARAM;
    }

    /* Derive from the REAL kernel clock instead of assuming 72 MHz. */
    const uint32_t f_tim = DRV_Clock_GetTimerClock(tim);
    const uint32_t ticks = f_tim / freq_hz;       /* (PSC+1) * (ARR+1) */
    if (ticks == 0u) {
        return DRV_INVALID_PARAM;                 /* frequency above f_tim */
    }

    /* Smallest prescaler that keeps the reload within 16 bits. */
    uint32_t psc_plus = (ticks + 0xFFFFu) / 0x10000u;
    if (psc_plus == 0u) psc_plus = 1u;
    const uint32_t arr_plus = ticks / psc_plus;
    if (arr_plus == 0u || psc_plus > 0x10000u) {
        return DRV_UNSUPPORTED;
    }

    return DRV_Timer_InitRaw(tim, (uint16_t)(psc_plus - 1u),
                             (uint16_t)(arr_plus - 1u));
}

void DRV_Timer_Start(TIM_TypeDef *tim) { DRV_SET_BITS(tim->CR1, TIM_CR1_CEN); }
void DRV_Timer_Stop(TIM_TypeDef *tim)  { DRV_CLEAR_BITS(tim->CR1, TIM_CR1_CEN); }

/* -------------------------------------------------------------------------- */
/*  Update interrupt                                                          */
/* -------------------------------------------------------------------------- */

DRV_Status DRV_Timer_AttachCallback(TIM_TypeDef *tim, DRV_Timer_Callback cb,
                                    void *ctx, uint32_t priority)
{
    if (tim == NULL) {
        return DRV_INVALID_PARAM;
    }
    Timer_Slot *slot = timer_slot_alloc(tim);
    if (slot == NULL) {
        return DRV_ERROR;                 /* registry full */
    }
    slot->cb  = cb;
    slot->ctx = ctx;

    tim->SR = (uint32_t)~TIM_SR_UIF;      /* drop any stale flag */
    DRV_Timer_EnableUpdateIRQ(tim);

    const IRQn_Type irqn = timer_update_irqn(tim);
    NVIC_SetPriority(irqn, priority);
    NVIC_EnableIRQ(irqn);
    return DRV_OK;
}

void DRV_Timer_Dispatch(TIM_TypeDef *tim)
{
    if ((tim->SR & TIM_SR_UIF) == 0u) {
        return;
    }
    /* Write 1s everywhere except UIF: clears only that flag, and cannot lose a
     * flag that sets between the read and the write. */
    tim->SR = (uint32_t)~TIM_SR_UIF;

    Timer_Slot *slot = timer_slot_find(tim);
    if (slot != NULL && slot->cb != NULL) {
        slot->cb(tim, slot->ctx);
    }
}

/* -------------------------------------------------------------------------- */
/*  Millisecond tick                                                          */
/* -------------------------------------------------------------------------- */

static void timer_tick_cb(TIM_TypeDef *tim, void *ctx)
{
    DRV_UNUSED(tim);
    DRV_UNUSED(ctx);
    s_tick_ms++;
}

DRV_Status DRV_Timer_StartTick(TIM_TypeDef *tim, uint32_t priority)
{
    DRV_Status st = DRV_Timer_InitHz(tim, 1000u);   /* 1 kHz => 1 ms */
    if (st != DRV_OK) {
        return st;
    }
    s_tick_ms = 0u;
    st = DRV_Timer_AttachCallback(tim, timer_tick_cb, NULL, priority);
    if (st != DRV_OK) {
        return st;
    }
    DRV_Timer_Start(tim);
    return DRV_OK;
}

uint32_t DRV_Timer_GetTick(void)
{
    return s_tick_ms;
}

bool DRV_Timer_Elapsed(uint32_t start_ms, uint32_t timeout_ms)
{
    return (uint32_t)(s_tick_ms - start_ms) >= timeout_ms;
}

/* -------------------------------------------------------------------------- */
/*  Blocking delays                                                           */
/* -------------------------------------------------------------------------- */

/* Cycles per microsecond, refreshed by DRV_Delay_Init(). */
static uint32_t s_cycles_per_us = 8u;   /* safe default for the 8 MHz HSI */

DRV_Status DRV_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    const uint32_t hclk = DRV_Clock_GetHCLK();
    s_cycles_per_us = (hclk / 1000000u) ? (hclk / 1000000u) : 1u;

    /* The counter must actually be ticking, or Delay_Us would spin forever. */
    const uint32_t first = DWT->CYCCNT;
    for (volatile uint32_t i = 0; i < 100u; ++i) { /* let it advance */ }
    return (DWT->CYCCNT != first) ? DRV_OK : DRV_UNSUPPORTED;
}

void DRV_Delay_Us(uint32_t us)
{
    const uint32_t start  = DWT->CYCCNT;
    const uint32_t cycles = us * s_cycles_per_us;
    /* Unsigned subtraction stays correct across the 32-bit wrap. */
    while ((DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}

void DRV_Delay_Ms(uint32_t ms)
{
    while (ms-- > 0u) {
        DRV_Delay_Us(1000u);
    }
}

/* -------------------------------------------------------------------------- */
/*  Default vector handlers                                                    */
/* -------------------------------------------------------------------------- */
#if DRV_TIMER_PROVIDE_IRQ_HANDLERS

void TIM1_UP_IRQHandler(void) { DRV_Timer_Dispatch(TIM1); }
void TIM2_IRQHandler(void)    { DRV_Timer_Dispatch(TIM2); }
void TIM3_IRQHandler(void)    { DRV_Timer_Dispatch(TIM3); }
void TIM4_IRQHandler(void)    { DRV_Timer_Dispatch(TIM4); }

#endif /* DRV_TIMER_PROVIDE_IRQ_HANDLERS */

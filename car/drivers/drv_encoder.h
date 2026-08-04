/**
 ******************************************************************************
 * @file    drv_encoder.h
 * @brief   Quadrature (incremental) encoder driver for the STM32F103.
 *
 * Uses a timer's hardware encoder interface, so counting costs no CPU time —
 * the peripheral tracks position and direction from the A/B phases directly.
 *
 * Everything is passed in: which timer, which A/B pins, the counting range and
 * the encoder mode. Adding a third encoder needs no change to this driver.
 *
 * Usable timers on the F103C8: TIM1, TIM2, TIM3, TIM4 (each provides CH1/CH2
 * on fixed pins — check the datasheet's alternate-function table).
 ******************************************************************************
 */
#ifndef DRV_ENCODER_H_
#define DRV_ENCODER_H_

#include "drv_common.h"
#include "drv_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Which edges the counter reacts to.
 *
 * Mode 3 gives 4x resolution (both edges of both phases) and is the usual
 * choice; modes 1 and 2 count on a single phase for 2x resolution.
 */
typedef enum {
    ENCODER_MODE_TI1 = 1,  /**< Count on TI1 edges only (2x).              */
    ENCODER_MODE_TI2 = 2,  /**< Count on TI2 edges only (2x).              */
    ENCODER_MODE_BOTH = 3, /**< Count on both TI1 and TI2 edges (4x).      */
} Encoder_Mode;

/** @brief One encoder channel. */
typedef struct {
    TIM_TypeDef  *tim;      /**< Timer providing the encoder interface.    */
    GPIO_TypeDef *port_a;   /**< Port of phase A (timer CH1 pin).          */
    uint8_t       pin_a;    /**< Pin of phase A.                           */
    GPIO_TypeDef *port_b;   /**< Port of phase B (timer CH2 pin).          */
    uint8_t       pin_b;    /**< Pin of phase B.                           */
    uint16_t      arr;      /**< Counting range, e.g. 0xFFFF for full.     */
    Encoder_Mode  mode;     /**< Edge sensitivity.                         */
    bool          invert;   /**< Swap the counting direction.              */
} Encoder_Config;

/**
 * @brief Configure and start the encoder described by @p cfg.
 * @return DRV_OK or DRV_INVALID_PARAM.
 *
 * Enables the port and timer clocks, sets both phase pins to floating input,
 * puts the timer in encoder mode and starts counting from 0.
 */
DRV_Status DRV_Encoder_Init(const Encoder_Config *cfg);

/** @brief Current raw count of @p tim (wraps at the configured range). */
static inline uint16_t DRV_Encoder_Get(TIM_TypeDef *tim)
{
    return (uint16_t)tim->CNT;
}

/** @brief Force the count of @p tim to @p value. */
static inline void DRV_Encoder_Set(TIM_TypeDef *tim, uint16_t value)
{
    tim->CNT = value;
}

/** @brief Reset the count of @p tim to zero. */
static inline void DRV_Encoder_Reset(TIM_TypeDef *tim)
{
    tim->CNT = 0u;
}

/**
 * @brief Signed ticks counted since the previous call for this encoder.
 * @param tim       Timer to read.
 * @param prev_count Caller-owned storage holding the last raw count; updated
 *                   on return.
 * @return Delta in ticks, negative when running backwards.
 *
 * Handles 16-bit wrap-around in both directions, so it stays correct across an
 * overflow — the usual way to feed a speed calculation.
 */
int16_t DRV_Encoder_GetDelta(TIM_TypeDef *tim, uint16_t *prev_count);

#ifdef __cplusplus
}
#endif

#endif /* DRV_ENCODER_H_ */

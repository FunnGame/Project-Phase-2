#ifndef DRV_ENCODER_H_
#define DRV_ENCODER_H_

#include "drv_common.h"
#include "drv_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ENCODER_MODE_TI1 = 1,  
    ENCODER_MODE_TI2 = 2,  
    ENCODER_MODE_BOTH = 3, 
} Encoder_Mode;

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

DRV_Status DRV_Encoder_Init(const Encoder_Config *cfg);

static inline uint16_t DRV_Encoder_Get(TIM_TypeDef *tim)
{
    return (uint16_t)tim->CNT;
}

static inline void DRV_Encoder_Set(TIM_TypeDef *tim, uint16_t value)
{
    tim->CNT = value;
}

static inline void DRV_Encoder_Reset(TIM_TypeDef *tim)
{
    tim->CNT = 0u;
}

int16_t DRV_Encoder_GetDelta(TIM_TypeDef *tim, uint16_t *prev_count);

#ifdef __cplusplus
}
#endif

#endif 
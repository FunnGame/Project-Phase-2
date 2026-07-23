#include "stm32f10x.h"
#include "drv_timer.h"
#include "drv_gpio.h"


static void (*TIM2_Update_Callback)(void) = 0;

void Timer_Init(void) {
    GPIO_Init(PA, 8, IN, I_F);   // TIM1_CH1 = Fase A encoder trai
    GPIO_Init(PA, 9, IN, I_F);   // TIM1_CH2 = Fase B encoder trai
    GPIO_Init(PB, 6, IN, I_F);   // TIM4_CH1 = Fase A encoder phai 
    GPIO_Init(PB, 7, IN, I_F);   // TIM4_CH2 = Fase B encoder phai

	  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	
    TIM1->PSC = 0;
    TIM1->ARR = 0xFFFF;
    TIM1->CCMR1 = 0x0101;  // Get signal in PA8, PA9 map l?n lu?t vào TI1 và TI2
    TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P); // (Rising Edge and Falling Edge) at Encoder Mode
    TIM1->SMCR = 0x0003; // Encoder Mode 3 decide CNT by TI1 and TI2 

    TIM4->PSC = 0;
    TIM4->ARR = 0xFFFF;
    TIM4->CCMR1 = 0x0101;
    TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM4->SMCR = 0x0003;

    TIM1->CNT = 0;
    TIM4->CNT = 0;
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM4->CR1 |= TIM_CR1_CEN;
}

uint16_t Timer_Get_Counter(Timers tim) {
    if (tim == T1) return TIM1->CNT;
    if (tim == T4) return TIM4->CNT;
    return 0;
}

void Timer2_Interrupt_Init(void (*callback)(void)) {
    TIM2_Update_Callback = callback;

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 719;
    TIM2->ARR = 999;

    TIM2->DIER |= TIM_DIER_UIE;

    NVIC->ISER[0] |= (1 << 28);

    TIM2->CR1 |= TIM_CR1_CEN;
}


void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF; 

        if (TIM2_Update_Callback != 0) {
            TIM2_Update_Callback();
        }
    }
}
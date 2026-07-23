#ifndef DRV_TIMER_H
#define DRV_TIMER_H

#include "stm32f10x.h"

typedef enum {
    T1, T2, T3, T4
} Timers;


void Timer_Init(void);

void Timer2_Interrupt_Init(void (*callback)(void));

uint16_t Timer_Get_Counter(Timers tim);
void Timer_Set_Counter(Timers tim, uint16_t value);

#endif
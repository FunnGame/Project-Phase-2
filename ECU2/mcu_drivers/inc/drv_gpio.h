#ifndef __DRV_GPIO_H
#define __DRV_GPIO_H

#include "stm32f10x.h"

typedef enum{
	PA, PB, PC
} Ports;

typedef enum{
	IN, OUT10, OUT2, OUT50
} Mode;

enum inputMode{
	I_AN, I_F, I_PP
};

enum outputMode{
	O_GP_PP, O_GP_OD, O_AF_PP, O_AF_OD
};

void GPIO_Init(Ports port, unsigned short pin, Mode mode, unsigned short cnf);

void GPIO_Write(Ports port, unsigned short pin, unsigned short state);

int GPIO_Read(Ports port, unsigned short pin);

void GPIO_Toggle(Ports port, unsigned short pin);

#endif
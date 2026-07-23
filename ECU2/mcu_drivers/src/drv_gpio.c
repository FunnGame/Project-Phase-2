#include "stm32f10x.h"
#include "drv_gpio.h"

void GPIO_Init(Ports port, unsigned short pin, Mode mode, unsigned short cnf){
	GPIO_TypeDef *cr;
	int tPin = pin;


	RCC->APB2ENR |= 1 << (port + 2);

	switch(port){
		case PA: cr = GPIOA; break;
		case PB: cr = GPIOB; break;
		case PC: cr = GPIOC; break;
		default: return;
	}


	unsigned long pin_config = ((cnf & 0x03) << 2) | (mode & 0x03);

	if(pin > 7){
		tPin -= 8;

		cr->CRH &= ~(0xF << (tPin * 4));

		cr->CRH |= (pin_config << (tPin * 4));
	}
	else{

		cr->CRL &= ~(0xF << (tPin * 4));

		cr->CRL |= (pin_config << (tPin * 4));
	}


	if(mode == IN && cnf == I_PP){
		cr->ODR |= (1 << pin);
	}
}

void GPIO_Write(Ports port, unsigned short pin, unsigned short state){
	GPIO_TypeDef *cr;
	switch(port){
		case PA: cr = GPIOA; break;
		case PB: cr = GPIOB; break;
		case PC: cr = GPIOC; break;
		default: return;
	}


	(state) ? (cr->ODR |= (1 << pin)) : (cr->ODR &= ~(1 << pin));
}


int GPIO_Read(Ports port, unsigned short pin){
	GPIO_TypeDef *cr;
	switch(port){
		case PA: cr = GPIOA; break;
		case PB: cr = GPIOB; break;
		case PC: cr = GPIOC; break;
		default: return -1;
	}

	return (cr->IDR & (1 << pin)) != 0;
}


void GPIO_Toggle(Ports port, unsigned short pin){
	GPIO_TypeDef *cr;
	switch(port){
		case PA: cr = GPIOA; break;
		case PB: cr = GPIOB; break;
		case PC: cr = GPIOC; break;
		default: return;
	}

	cr->ODR ^= (1 << pin);
}
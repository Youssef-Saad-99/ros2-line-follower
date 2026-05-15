/*
 * TIM0_reg.h
 *
 * Created: 10/24/2025 10:45:39 AM
 *  Author: Al Jazeera
 */ 


#ifndef TIM0_REG_H_
#define TIM0_REG_H_


#define TCCR0_REG	*((volatile u8*)0x53)
#define TCNT0_REG	*((volatile u8*)0x52)
#define OCR0_REG	*((volatile u8*)0x5C)
#define TIMSK_REG	*((volatile u8*)0x59)
#define TIFR_REG	*((volatile u8*)0x58)
#define SFIOR_REG	*((volatile u8*)0x50)

#endif /* TIM0_REG_H_ */
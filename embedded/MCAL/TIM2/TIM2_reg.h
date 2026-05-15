/*
 * TIM2_reg.h
 *
 * Created: 11/7/2025 12:22:03 AM
 *  Author: Al Jazeera
 */ 


#ifndef TIM2_REG_H_
#define TIM2_REG_H_


#define TCCR2_REG	*((volatile u8*)0x45)
#define TCNT2_REG	*((volatile u8*)0x44)
#define OCR2_REG	*((volatile u8*)0x43)
#define TIMSK_REG	*((volatile u8*)0x59)
#define TIFR_REG	*((volatile u8*)0x58)
#define SFIOR_REG	*((volatile u8*)0x50)


#endif /* TIM2_REG_H_ */
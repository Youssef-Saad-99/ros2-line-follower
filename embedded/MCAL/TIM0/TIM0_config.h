/*
 * TIM0_config.h
 *
 * Created: 10/24/2025 11:44:11 AM
 *  Author: Al Jazeera
 */ 


#ifndef TIM0_CONFIG_H_
#define TIM0_CONFIG_H_


 #define TIM0_0_PRESCALER	1
 #define TIM0_8_PRESCALER	2
 #define TIM0_64_PRESCALER	3
 #define TIM0_256_PRESCALER	4
 #define TIM0_1024_PRESCALER 5
 
 
 #define  TIM0_PRESCALER   TIM0_64_PRESCALER

#define  OV_MODE	0
#define  CTC_MODE	1

#define  FAST_MODE	0
#define  PHASE_CORRECT_MODE	1

#define OCR_VAL     250

#endif /* TIM0_CONFIG_H_ */
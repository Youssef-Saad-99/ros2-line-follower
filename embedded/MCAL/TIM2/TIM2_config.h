/*
 * TIM2_config.h
 *
 * Created: 11/7/2025 12:21:26 AM
 *  Author: Al Jazeera
 */ 


#ifndef TIM2_CONFIG_H_
#define TIM2_CONFIG_H_


 #define TIM2_0_PRESCALER	1
 #define TIM2_8_PRESCALER	2
 #define TIM2_32_PRESCALER	3
 #define TIM2_64_PRESCALER	4
 #define TIM2_128_PRESCALER	5
 #define TIM2_256_PRESCALER	6
 #define TIM2_1024_PRESCALER 7
 
 
 #define  TIM2_PRESCALER   TIM2_64_PRESCALER

 #define  OV_MODE	0
 #define  CTC_MODE	1

 #define  FAST_MODE	0
 #define  PHASE_CORRECT_MODE	1

 #define OCR_VAL     250


#endif /* TIM2_CONFIG_H_ */
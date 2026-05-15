/*
 * TIM1_interface.h
 *
 * Created: 11/8/2025 10:17:50 AM
 *  Author: Al Jazeera
 */ 


#ifndef TIM1_INTERFACE_H_
#define TIM1_INTERFACE_H_

void TIM1_voidFastPWM_ICR1(void);
void TIM1_voidSetOcrVal(u16 copy_u16val);
void TIM1_voidSetCallBack(void(*ptr)(void),u8 copy_u8mode);
void TIM1_voidICU_Init(void);

#endif /* TIM1_INTERFACE_H_ */
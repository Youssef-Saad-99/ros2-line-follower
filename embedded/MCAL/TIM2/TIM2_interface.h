/*
 * TIM2_inteface.h
 *
 * Created: 11/7/2025 12:20:59 AM
 *  Author: Al Jazeera
 */ 


#ifndef TIM2_INTEFACE_H_
#define TIM2_INTEFACE_H_

void TIM2_voidInit(u8 copy_u8mode);

void TIM2_voidSetCallBack(void(*ptr)(void),u8 copy_u8mode);

void TIM2_voidPWM(u8 copy_u8dc,u8 copy_u8mode);



#endif /* TIM2_INTEFACE_H_ */
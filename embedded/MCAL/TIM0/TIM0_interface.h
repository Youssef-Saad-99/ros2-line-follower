/*
 * TIM0_interface.h
 *
 * Created: 10/24/2025 10:46:05 AM
 *  Author: Al Jazeera
 */ 


#ifndef TIM0_INTERFACE_H_
#define TIM0_INTERFACE_H_



void TIM0_voidInit(u8 copy_u8mode);
void TIM0_voidSetCallBack(void(*ptr)(void),u8 copy_u8mode);

void TIM0_voidPWM(u8 copy_u8dc,u8 copy_u8mode);


#endif /* TIM0_INTERFACE_H_ */
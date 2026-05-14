/*
 * EXTI_interface.h
 *
 * Created: 10/3/2025 5:33:11 AM
 *  Author: Al Jazeera
 */ 


#ifndef EXTI_INTERFACE_H_
#define EXTI_INTERFACE_H_

#include "../../LIB/BIT_MATH.h"
#include "../../LIB/STD_TYPE.h"
#include "../../LIB/INTEERUPT_NUM.h"
#include "EXTI_reg.h"

#define INT_0   0
#define INT_1   1
#define INT_2   2


#define LOW_LEVEL              0
#define ANY_LOGICAL_CHANGE     1
#define FALLING_AGE            2
#define RISING_AGE             3

void EXTI_voidInit(u8 copyu8_sence,u8 copyu8_exti_num) ;


void EXTI0_voidEnable(void) ;
void EXTI1_voidEnable(void) ;
void EXTI2_voidEnable(void) ;


void EXTI0_voidDisable(void) ;
void EXTI1_voidDisable(void) ;
void EXTI2_voidDisable(void) ;


void EXTI_voidSetCallBack(void (*ptr)(void),u8 copy_u8exti_num);

#endif /* EXTI_INTERFACE_H_ */
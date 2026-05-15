/*
 * TIM2_prog.c
 *
 * Created: 11/7/2025 12:20:33 AM
 *  Author: Al Jazeera
 */ 

#include "../../LIB/BIT_MATH.h"
#include "../../LIB/STD_TYPE.h"
#include "../../LIB/INTEERUPT_NUM.h"
#include "TIM2_interface.h"
#include "TIM2_reg.h"
#include "TIM2_config.h"


void (*TIM2_PTR[])(void)={null};
void TIM2_voidInit(u8 copy_u8mode)
{
	TCCR2_REG&=0b11111000;
	TCCR2_REG|=TIM2_PRESCALER;
	
	switch(copy_u8mode){
		case OV_MODE:
		CLR_BIT(TCCR2_REG,3);
		CLR_BIT(TCCR2_REG,6);

		
		SET_BIT(TIMSK_REG,6);
		
		break;
		
			case CTC_MODE:
			
			SET_BIT(TCCR2_REG,3);
			CLR_BIT(TCCR2_REG,6);
			
			OCR2_REG=OCR_VAL;
			SET_BIT(TIMSK_REG,7);
			break;
		}
}


void TIM2_voidSetCallBack(void(*ptr)(void),u8 copy_u8mode){
	
	switch(copy_u8mode){
		case OV_MODE:
		TIM2_PTR[0]=ptr;
		break;
		
		case CTC_MODE:
		TIM2_PTR[1]=ptr;
		
	}
}

ISR(TIMER2_OVF){

	TIM2_PTR[0]();
}

ISR(TIMER2_COMP){
	
	TIM2_PTR[1]();

}


void TIM2_voidPWM(u8 copy_u8dc,u8 copy_u8mode){
	
	TCCR2_REG&=0b11111000;
	TCCR2_REG|=TIM2_PRESCALER;
	


	switch(copy_u8mode){
		case FAST_MODE:
		SET_BIT(TCCR2_REG,3);
		SET_BIT(TCCR2_REG,6);

		SET_BIT(TCCR2_REG,5);
		CLR_BIT(TCCR2_REG,4);
		
		OCR2_REG=copy_u8dc *2.55;
		
		break;
		
		case PHASE_CORRECT_MODE:
		
		SET_BIT(TCCR2_REG,6);
		CLR_BIT(TCCR2_REG,3);
		
		SET_BIT(TCCR2_REG,5);
		CLR_BIT(TCCR2_REG,4);
		
		OCR2_REG=copy_u8dc *2.55;
		break;
	}
	
	
}

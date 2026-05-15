
/*
 * TIM0_prog.c
 *
 * Created: 10/24/2025 10:43:25 AM
 *  Author: Al Jazeera
 */ 


#include "../../LIB/BIT_MATH.h"
#include "../../LIB/STD_TYPE.h"
#include "../../LIB/INTEERUPT_NUM.h"
#include "TIM0_interface.h"
#include "TIM0_reg.h"
#include "TIM0_config.h"

void (*TIM0_PTR[])(void)={null};


void TIM0_voidInit(u8 copy_u8mode)
{
	TCCR0_REG&=0b11111000;
	TCCR0_REG|=TIM0_PRESCALER;
	
	switch(copy_u8mode){
		case OV_MODE:
	CLR_BIT(TCCR0_REG,3);
	CLR_BIT(TCCR0_REG,6);

		
	SET_BIT(TIMSK_REG,0);
	
	break;
	
	case CTC_MODE:
		
		SET_BIT(TCCR0_REG,3);
		CLR_BIT(TCCR0_REG,6);
		
		OCR0_REG=OCR_VAL;
		SET_BIT(TIMSK_REG,1);
		break;
	}
}

void TIM0_voidSetCallBack(void(*ptr)(void),u8 copy_u8mode){
	
	switch(copy_u8mode){
	case OV_MODE:
	TIM0_PTR[0]=ptr;
	break;
	
	case CTC_MODE:
	TIM0_PTR[1]=ptr;
	
	}
}

ISR(TIMER0_OVF){

		TIM0_PTR[0]();
}

ISR(TIMER0_COMP){
	
TIM0_PTR[1]();

}



void TIM0_voidPWM(u8 copy_u8dc,u8 copy_u8mode){
	
	TCCR0_REG&=0b11111000;
	TCCR0_REG|=TIM0_PRESCALER;
	


	switch(copy_u8mode){
		case FAST_MODE:
		SET_BIT(TCCR0_REG,3);
		SET_BIT(TCCR0_REG,6);

		SET_BIT(TCCR0_REG,5);
		CLR_BIT(TCCR0_REG,4);
		
		OCR0_REG=copy_u8dc *2.55;
		
		break;
		
		case PHASE_CORRECT_MODE:
		
		SET_BIT(TCCR0_REG,6);
		CLR_BIT(TCCR0_REG,3);
		
		SET_BIT(TCCR0_REG,5);
		CLR_BIT(TCCR0_REG,4);
		
		OCR0_REG=copy_u8dc *2.55;
		break;
	}
	
	
}
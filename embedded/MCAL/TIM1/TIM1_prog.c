/*
 * TIM1_prog.c
 *
 * Created: 11/8/2025 10:18:54 AM
 *  Author: Al Jazeera
 */ 
#include "../../LIB/BIT_MATH.h"
#include "../../LIB/STD_TYPE.h"
#include "../../LIB/INTEERUPT_NUM.h"
#include "TIM1_interface.h"
#include "TIM1_reg.h"
#include "TIM1_config.h"

volatile u16 t1 = 0;
volatile u16 t2 = 0;
volatile u16 distance = 0;
volatile u8 state = 1;

void TIM1_voidFastPWM_ICR1(void)
{
	TCCR1B_REG&=0b11111000;
	TCCR1B_REG|=TIM1_PRESCALER;
	
	
	CLR_BIT(TCCR1A_REG,0);
	SET_BIT(TCCR1A_REG,1);
	SET_BIT(TCCR1B_REG,3);
	SET_BIT(TCCR1B_REG,4);
	
	
	CLR_BIT(TCCR1A_REG,6);
	SET_BIT(TCCR1A_REG,7);
	
	ICR1_REG=TOP_VAL;
}


void TIM1_voidSetOcrVal(u16 copy_u16val)
{
	
	OCR1A_REG=copy_u16val;
}

ISR(TIMER1_CAPT){

if(state==1){
	t1=ICR1_REG;
	
	CLR_BIT(TCCR1B_REG,6);
	state=0;
}
else if (state==0)
{

	t2=ICR1_REG;
	
	SET_BIT(TCCR1B_REG,6);
	distance=t2-t1/58;
	if (distance<20&&distance>0)
	{ 
	}
	state=0;
}
}

void TIM1_voidICU_init(void)
{
	TCCR1A_REG = 0x00;

	TCCR1B_REG &= 0b11111000;     
	TCCR1B_REG |= TIM1_PRESCALER; 
	
	SET_BIT(TCCR1B_REG,6);
	SET_BIT(TIMSK_REG,5);
	
	

	}


	



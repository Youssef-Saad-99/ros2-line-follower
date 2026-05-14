/*
 * ADC_prog.c
 *
 * Created: 10/12/2025 9:16:02 AM
 *  Author: Al Jazeera
 */ 
#include "../../LIB/BIT_MATH.h"
#include "../../LIB/STD_TYPE.h"
#include "ADC_interface.h"
#include "ADC_reg.h"

 
 void ADC_voidInit(void){

	CLR_BIT(ADMUX_REG,7);
	SET_BIT(ADMUX_REG,6);
	
	
	ADCSRA_REG&=0b11111000;
	ADCSRA_REG|=ADC_128_PRESCALER;
	
	
	SET_BIT(ADCSRA_REG,7);
	 
 }

u16 ADC_u16ReadChannel(u8 copy_u8channel)
{
	ADMUX_REG&=0b11100000;
	ADMUX_REG|=copy_u8channel;
	
	
	SET_BIT(ADCSRA_REG,6);
	
	while (GET_BIT(ADCSRA_REG,4)==0);
	
return ADC_REG;			  
			  
	}

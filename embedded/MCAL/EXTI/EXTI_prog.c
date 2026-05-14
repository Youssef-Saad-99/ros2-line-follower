#include "EXTI_interface.h"


void (*EXTI0_PTR) (void)=null;
void (*EXTI1_PTR) (void)=null;
void (*EXTI2_PTR) (void)=null;



void EXTI_voidInit(u8 copyu8_sence,u8 copyu8_exti_num) {
	
	switch (copyu8_exti_num){
		
		case INT_0:
		if (copyu8_sence==LOW_LEVEL){
			
			CLR_BIT(MCUCR_REG,0);
			CLR_BIT(MCUCR_REG,1);
			
		}
		else if (copyu8_sence==ANY_LOGICAL_CHANGE){
			
			SET_BIT(MCUCR_REG,0);
			CLR_BIT(MCUCR_REG,1);
			
		}
		
		else if(copyu8_sence==FALLING_AGE){
			CLR_BIT(MCUCR_REG,0);
			SET_BIT(MCUCR_REG,1);
		}
		
		else if (copyu8_sence==RISING_AGE){
			
			SET_BIT(MCUCR_REG,0);
			SET_BIT(MCUCR_REG,1);
			
		}
		
		
		break;
		case INT_1:
		if (copyu8_sence==LOW_LEVEL){
			
			CLR_BIT(MCUCR_REG,3);
			CLR_BIT(MCUCR_REG,2);
			
		}
		else if (copyu8_sence==ANY_LOGICAL_CHANGE){
			
			SET_BIT(MCUCR_REG,2);
			CLR_BIT(MCUCR_REG,3);
			
		}
		else if (copyu8_sence==FALLING_AGE){
			
			CLR_BIT(MCUCR_REG,2);
			SET_BIT(MCUCR_REG,3);
			
		}
		
		else if (copyu8_sence==RISING_AGE) {
			SET_BIT(MCUCR_REG,3);
			SET_BIT(MCUCR_REG,2);
		}
		
		break;
		case INT_2:
		
		
		if (copyu8_sence==FALLING_AGE){
			
			CLR_BIT(MCUCSR_REG,6);
			
			
		}
		
		else {
			SET_BIT(MCUCSR_REG,6);
			
		}
		
		break;
		
		
	}
	
	
	
}




	
	
	



void EXTI0_voidEnable(void) {
	
	SET_BIT(GICR_REG,6);
	
}


void EXTI1_voidEnable(void) {

	SET_BIT(GICR_REG,7);

}

void EXTI2_voidEnable(void) {

	SET_BIT(GICR_REG,5);

}

void EXTI0_voidDisable(void) {
	
	CLR_BIT(GICR_REG,6);
	
}


void EXTI1_voidDisable(void) {

	CLR_BIT(GICR_REG,7);

}

void EXTI2_voidDisable(void) {

	CLR_BIT(GICR_REG,5);

}
void EXTI_voidSetCallBack(void (*ptr)(void),u8 copy_u8exti_num){
	
	switch(copy_u8exti_num){
		
		case INT_0:EXTI0_PTR=ptr;break;
		case INT_1:EXTI1_PTR=ptr;break;
		case INT_2:EXTI2_PTR=ptr;break;
		
		
		
	}
}

	
	ISR(_INT0){
		
		EXTI0_PTR();
		
	}
	
	ISR(_INT1){
		
	EXTI1_PTR();
	
	}
	
	ISR(_INT2){
		
	EXTI2_PTR();
	
	}




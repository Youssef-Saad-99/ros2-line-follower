/*
 * GI_prog.c
 *
 * Created: 9/27/2025 11:52:13 AM
 *  Author: fathi
 */ 
#include "../../LIB/BIT_MATH.h"
#include "../../LIB/STD_TYPE.h" 
#include "GI_interface.h"
#include "GI_reg.h"


void GI_voidEnable(void) {
	
	/* Enable  GIE */

  SET_BIT(SREG_REG,7) ; 
}

void GI_voidDisable(void) {
	
	/* Disable GIE */
	CLR_BIT(SREG_REG ,7) ; 
}

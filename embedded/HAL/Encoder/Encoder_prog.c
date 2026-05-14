/*
 * ENCODER_prog.c
 *
 * HAL Layer for Wheel Encoders
 */

#include "../../LIB/STD_TYPE.h"
#include "../../LIB/BIT_MATH.h"
#include <avr/io.h>        // ⁄‘«‰ «·‹ DDRD Ê«·‹ PORTD
#include <avr/interrupt.h> // ⁄‘«‰ «·‹ cli() Ê«·‹ sei()

/* «⁄„· Include ·„·› «·‹ EXTI » «⁄ﬂ (« √ﬂœ „‰ «·„”«— Õ”» «·›Ê·œ—«  ⁄‰œﬂ) */
#include "../../MCAL/EXTI/EXTI_interface.h" 
#include "ENCODER_interface.h"

/* =======================================================================
 * PRIVATE STATE VARIABLES
 * ======================================================================= */
volatile s32 encoder_L_ticks = 0;
volatile s32 encoder_R_ticks = 0;

volatile s8 dir_L = 1; 
volatile s8 dir_R = 1;

/* =======================================================================
 * PRIVATE CALLBACK FUNCTIONS
 * ÕÿÌ‰« ﬂ·„… static ⁄‘«‰ «·œÊ«· œÌ  »ﬁÏ Œ«’… »«·›«Ì· œÂ »” Ê„Õœ‘ Ì‰«œÌÂ« „‰ »—Â
 * ======================================================================= */
static void ENCODER_voidLeftAction(void) {
    encoder_L_ticks += dir_L;
}

static void ENCODER_voidRightAction(void) {
    encoder_R_ticks += dir_R;
}

/* =======================================================================
 * FUNCTION IMPLEMENTATIONS
 * ======================================================================= */

void ENCODER_voidInit(void) {
    /* 1. Ÿ»ÿ «·‹ Pins » «⁄  INT0 (PD2) Ê INT1 (PD3) ﬂ‹ Input */
    /* „·«ÕŸ…: ·Ê ⁄‰œﬂ œ—«Ì›— DIO_interface.h Ì›÷·  ” Œœ„Â Â‰« »œ· «· ⁄«„· «·„»«‘— „⁄ «·‹ Registers */
    CLR_BIT(DDRD, 2);
    CLR_BIT(DDRD, 3);
    
    /*  ›⁄Ì· «·‹ Pull-up «·œ«Œ·Ì */
    SET_BIT(PORTD, 2);
    SET_BIT(PORTD, 3);

    /* 2. ≈⁄œ«œ «·‹ EXTI »«” Œœ«„ œÊ«· «·‹ MCAL */
    EXTI_voidInit(RISING_AGE, INT_0);
    EXTI_voidInit(RISING_AGE, INT_1);

    /* 3.  ”·Ì„ «·œÊ«· ﬂ‹ Callback ··‹ EXTI */
    EXTI_voidSetCallBack(ENCODER_voidLeftAction, INT_0);
    EXTI_voidSetCallBack(ENCODER_voidRightAction, INT_1);

    /* 4.  ›⁄Ì· «·‹ Interrupts */
    EXTI0_voidEnable();
    EXTI1_voidEnable();
}

s32 ENCODER_s32GetLeftTicks(void) {
    s32 local_s32Ticks;
    
    /* Õ„«Ì… «·„ €Ì— „‰ ≈‰ «·‹ Interrupt Ì€Ì—Â Ê≈Õ‰« »‰ﬁ—«Â */
    cli(); 
    local_s32Ticks = encoder_L_ticks;
    sei(); 
    
    return local_s32Ticks;
}

s32 ENCODER_s32GetRightTicks(void) {
    s32 local_s32Ticks;
    
    cli();
    local_s32Ticks = encoder_R_ticks;
    sei();
    
    return local_s32Ticks;
}

void ENCODER_voidSetLeftDirection(s8 copy_s8Dir) {
    dir_L = copy_s8Dir;
}

void ENCODER_voidSetRightDirection(s8 copy_s8Dir) {
    dir_R = copy_s8Dir;
}
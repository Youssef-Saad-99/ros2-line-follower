/*
 * ENCODER_interface.h
 *
 * HAL Layer for Wheel Encoders
 */

#ifndef ENCODER_INTERFACE_H_
#define ENCODER_INTERFACE_H_

#include "../../LIB/STD_TYPE.h"

/* =======================================================================
 * FUNCTION PROTOTYPES
 * ======================================================================= */

/* œ«·… «· ÂÌ∆…: » Ÿ»ÿ «·‹ Pins Ê«·‹ EXTI Ê«·‹ Callbacks */
void ENCODER_voidInit(void);

/* œÊ«· ﬁ—«¡… «·⁄œ«œ«  */
s32 ENCODER_s32GetLeftTicks(void);
s32 ENCODER_s32GetRightTicks(void);

/* œÊ«·  ÕœÌœ « Ã«Â «·⁄œ (1 ·ﬁœ«„° -1 ·Ê—«) */
void ENCODER_voidSetLeftDirection(s8 copy_s8Dir);
void ENCODER_voidSetRightDirection(s8 copy_s8Dir);

#endif /* ENCODER_INTERFACE_H_ */
/*
 * encoder_reg.h
 *
 * Low-level AVR register and bit-mask definitions used by the encoder module.
 * All hardware references are centralised here — do NOT scatter register
 * names throughout encoder.c or main.c.
 *
 * Target : AVR ATmega32 (16 MHz)
 * Author : Youssef Mohamed
 */

#ifndef ENCODER_REG_H_
#define ENCODER_REG_H_

#include <avr/io.h>

/* =======================================================================
 * EXTERNAL INTERRUPT CONTROL REGISTER A  (MCUCR)
 * Controls the sense mode for INT0 and INT1.
 * ======================================================================= */

/** MCUCR bit positions for INT0 sense control */
#define ENCODER_ISC00_BIT            0U   /* INT0 sense control bit 0 */
#define ENCODER_ISC01_BIT            1U   /* INT0 sense control bit 1 */

/** MCUCR bit positions for INT1 sense control */
#define ENCODER_ISC10_BIT            2U   /* INT1 sense control bit 0 */
#define ENCODER_ISC11_BIT            3U   /* INT1 sense control bit 1 */

/** Sense-mode constants (written into ISCx1:ISCx0) */
#define ENCODER_ISC_LOW_LEVEL        0x00U   /* 0b00 – low level triggers */
#define ENCODER_ISC_ANY_EDGE         0x01U   /* 0b01 – any logical change  */
#define ENCODER_ISC_FALLING_EDGE     0x02U   /* 0b10 – falling edge        */
#define ENCODER_ISC_RISING_EDGE      0x03U   /* 0b11 – rising edge         */

/* =======================================================================
 * GENERAL INTERRUPT CONTROL REGISTER  (GICR)
 * Enables individual external interrupts.
 * ======================================================================= */
#define ENCODER_INT0_ENABLE_BIT      6U   /* GICR bit 6 ? INT0 enable */
#define ENCODER_INT1_ENABLE_BIT      7U   /* GICR bit 7 ? INT1 enable */

/* =======================================================================
 * GENERAL INTERRUPT FLAG REGISTER  (GIFR)
 * Cleared by writing logic-1 (hardware auto-clears on ISR entry).
 * ======================================================================= */
#define ENCODER_INTF0_BIT            6U   /* INT0 flag */
#define ENCODER_INTF1_BIT            7U   /* INT1 flag */

/* =======================================================================
 * REGISTER ALIASES
 * Use these names inside encoder.c instead of the raw AVR names so that
 * porting to another AVR variant only requires changing this file.
 * ======================================================================= */

/** Interrupt sense control register */
#define ENCODER_SENSE_REG            MCUCR

/** Interrupt enable register */
#define ENCODER_INT_ENABLE_REG       GICR

/** Interrupt flag register */
#define ENCODER_INT_FLAG_REG         GIFR

/* =======================================================================
 * CONVENIENCE MACROS
 * ======================================================================= */

/** Enable INT0 (left encoder) */
#define ENCODER_ENABLE_INT0()   \
    ( ENCODER_INT_ENABLE_REG |= (1U << ENCODER_INT0_ENABLE_BIT) )

/** Enable INT1 (right encoder) */
#define ENCODER_ENABLE_INT1()   \
    ( ENCODER_INT_ENABLE_REG |= (1U << ENCODER_INT1_ENABLE_BIT) )

/** Disable INT0 (left encoder) */
#define ENCODER_DISABLE_INT0()  \
    ( ENCODER_INT_ENABLE_REG &= ~(1U << ENCODER_INT0_ENABLE_BIT) )

/** Disable INT1 (right encoder) */
#define ENCODER_DISABLE_INT1()  \
    ( ENCODER_INT_ENABLE_REG &= ~(1U << ENCODER_INT1_ENABLE_BIT) )

/** Clear INT0 pending flag */
#define ENCODER_CLEAR_INT0_FLAG()   \
    ( ENCODER_INT_FLAG_REG |= (1U << ENCODER_INTF0_BIT) )

/** Clear INT1 pending flag */
#define ENCODER_CLEAR_INT1_FLAG()   \
    ( ENCODER_INT_FLAG_REG |= (1U << ENCODER_INTF1_BIT) )

#endif /* ENCODER_REG_H_ */
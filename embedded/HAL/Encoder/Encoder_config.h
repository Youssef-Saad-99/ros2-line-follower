/*
 * encoder_config.h
 *
 * User-configurable parameters for the wheel encoder module.
 * Modify this file to match your hardware setup.
 *
 * Author : Youssef Mohamed
 * Target : AVR ATmega32 (or compatible)
 */

#ifndef ENCODER_CONFIG_H_
#define ENCODER_CONFIG_H_

/* -----------------------------------------------------------------------
 * ENCODER SEND INTERVAL
 * How often (in milliseconds) encoder data is transmitted over UART.
 * Default: 50 ms  ?  20 Hz update rate.
 * ----------------------------------------------------------------------- */
#define ENCODER_SEND_INTERVAL_MS     50U

/* -----------------------------------------------------------------------
 * PULSES PER REVOLUTION
 * Number of encoder pulses counted per full wheel revolution.
 * Adjust to match your specific encoder disc / sensor combination.
 * ----------------------------------------------------------------------- */
#define ENCODER_PPR                  20U

/* -----------------------------------------------------------------------
 * WHEEL CIRCUMFERENCE (mm)
 * Used by higher-level odometry calculations (not in this module directly).
 * ----------------------------------------------------------------------- */
#define ENCODER_WHEEL_CIRCUMFERENCE_MM   200U

/* -----------------------------------------------------------------------
 * INTERRUPT SENSE CONTROL
 * Choose rising / falling / any-edge triggering for encoder ISRs.
 *
 * Options (defined in encoder_reg.h):
 *   ENCODER_ISC_RISING_EDGE
 *   ENCODER_ISC_FALLING_EDGE
 *   ENCODER_ISC_ANY_EDGE
 * ----------------------------------------------------------------------- */
#define ENCODER_LEFT_ISC             ENCODER_ISC_RISING_EDGE
#define ENCODER_RIGHT_ISC            ENCODER_ISC_RISING_EDGE

#endif /* ENCODER_CONFIG_H_ */
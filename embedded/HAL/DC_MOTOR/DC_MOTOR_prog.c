/*
 * DC_MOTOR_prog.c
 *
 * Created: 11/7/2025 10:40:09 AM
 *  Author: Al Jazeera
 */
#include "../../LIB/BIT_MATH.h"
#include "../../LIB/STD_TYPE.h"
#include "DC_MOTOR_interface.h"
#include "DC_MOTOR_config.h"
#include "../../MCAL/DIO/DIO_interface.h"
#include "../../MCAL/TIM1/TIM1_interface.h"
#include "../../MCAL/TIM1/TIM1_config.h"

// ?? Calibration ???????????????????????????????????????????????????????????????
// لو العربية بتلف يمين ? زود LEFT_CALIBRATION
// لو العربية بتلف شمال ? زود RIGHT_CALIBRATION
// ابدأ بـ 0.85 وعدّل بـ 0.05 في كل مرة لحد ما تمشي تقيلة صح
#define FORWARD_RIGHT_CALIBRATION   1.00
#define FORWARD_LEFT_CALIBRATION    0.87   // اليمين

#define BACKWARD_RIGHT_CALIBRATION  1.00
#define BACKWARD_LEFT_CALIBRATION   1.00
// ?????????????????????????????????????????????????????????????????????????????

void DcMotor_voidInit(void)
{
    DIO_voidSetPortDir(MOTOR_PORT, 0xff);
    DIO_voidSetPinDir(DIO_PORTD, DIO_PIN4, OUTPUT);
    DIO_voidSetPinDir(DIO_PORTD, DIO_PIN5, OUTPUT);
    TIM1_voidInit();
}

void DcMotor_voidForward(f32 copyu8_speed)
{
    f32 rightSpeed = (f32)(copyu8_speed * FORWARD_RIGHT_CALIBRATION);
    f32 leftSpeed  = (f32)(copyu8_speed * FORWARD_LEFT_CALIBRATION);

    TIM1_voidDualPWM(rightSpeed, leftSpeed);

    DIO_voidSetPinVal(MOTOR_PORT, MOTOR1_PIN1, HIGH);
    DIO_voidSetPinVal(MOTOR_PORT, MOTOR1_PIN2, LOW);

    DIO_voidSetPinVal(MOTOR_PORT, MOTOR2_PIN1, HIGH);
    DIO_voidSetPinVal(MOTOR_PORT, MOTOR2_PIN2, LOW);
}

void DcMotor_voidBackward(u8 copyu8_speed)
{
    u8 rightSpeed = (u8)(copyu8_speed * BACKWARD_RIGHT_CALIBRATION);
    u8 leftSpeed  = (u8)(copyu8_speed * BACKWARD_LEFT_CALIBRATION);

    TIM1_voidDualPWM(rightSpeed, leftSpeed);

    DIO_voidSetPinVal(MOTOR_PORT, MOTOR1_PIN2, HIGH);
    DIO_voidSetPinVal(MOTOR_PORT, MOTOR1_PIN1, LOW);

    DIO_voidSetPinVal(MOTOR_PORT, MOTOR2_PIN2, HIGH);
    DIO_voidSetPinVal(MOTOR_PORT, MOTOR2_PIN1, LOW);
}

void DcMotor_voidRight(u8 copyu8_speed)
{
    TIM1_voidDualPWM(copyu8_speed, copyu8_speed);

    DIO_voidSetPinVal(MOTOR_PORT, MOTOR1_PIN1, HIGH);
    DIO_voidSetPinVal(MOTOR_PORT, MOTOR1_PIN2, LOW);

    DIO_voidSetPinVal(MOTOR_PORT, MOTOR2_PIN1, LOW);
    DIO_voidSetPinVal(MOTOR_PORT, MOTOR2_PIN2, LOW);
}

void DcMotor_voidLeft(u8 copyu8_speed)
{
    TIM1_voidDualPWM(copyu8_speed, copyu8_speed);

    DIO_voidSetPinVal(MOTOR_PORT, MOTOR1_PIN1, LOW);
    DIO_voidSetPinVal(MOTOR_PORT, MOTOR1_PIN2, LOW);

    DIO_voidSetPinVal(MOTOR_PORT, MOTOR2_PIN1, HIGH);
    DIO_voidSetPinVal(MOTOR_PORT, MOTOR2_PIN2, LOW);
}

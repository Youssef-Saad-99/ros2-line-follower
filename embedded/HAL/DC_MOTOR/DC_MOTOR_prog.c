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
#include "../../MCAL//DIO/DIO_interface.h"
#include "../../MCAL//TIM2/TIM2_interface.h"
#include "../../MCAL//TIM2/TIM2_config.h"

void DcMotor_voidInit(void)
{
	DIO_voidSetPortDir(MOTOR_PORT, 0xff);
	DIO_voidSetPinDir(DIO_PORTD,DIO_PIN7,OUTPUT);
}

void DcMotor_voidForward(u8 copyu8_speed)
{
	
	TIM2_voidPWM(copyu8_speed,FAST_MODE);
	
	
	
	DIO_voidSetPinVal(MOTOR_PORT,MOTOR1_PIN1,HIGH);
	DIO_voidSetPinVal(MOTOR_PORT,MOTOR1_PIN2,LOW);
	
	DIO_voidSetPinVal(MOTOR_PORT,MOTOR2_PIN1,HIGH);
	DIO_voidSetPinVal(MOTOR_PORT,MOTOR2_PIN2,LOW);

}






void DcMotor_voidBackward(u8 copyu8_speed)
{
	
	TIM2_voidPWM(copyu8_speed,FAST_MODE);
	
	
	
	DIO_voidSetPinVal(MOTOR_PORT,MOTOR1_PIN2,HIGH);
	DIO_voidSetPinVal(MOTOR_PORT,MOTOR1_PIN1,LOW);
	
	DIO_voidSetPinVal(MOTOR_PORT,MOTOR2_PIN2,HIGH);
	DIO_voidSetPinVal(MOTOR_PORT,MOTOR2_PIN1,LOW);

}
void DcMotor_voidRight(u8 copyu8_speed)
{
	
	TIM2_voidPWM(copyu8_speed,FAST_MODE);
	
	
	
	DIO_voidSetPinVal(MOTOR_PORT,MOTOR1_PIN1,HIGH);
	DIO_voidSetPinVal(MOTOR_PORT,MOTOR1_PIN2,LOW);
	
	DIO_voidSetPinVal(MOTOR_PORT,MOTOR2_PIN1,LOW);
	DIO_voidSetPinVal(MOTOR_PORT,MOTOR2_PIN2,LOW);
	


}
void DcMotor_voidLeft(u8 copyu8_speed)
{
	

TIM2_voidPWM(copyu8_speed,FAST_MODE);


DIO_voidSetPinVal(MOTOR_PORT,MOTOR1_PIN1,LOW);
DIO_voidSetPinVal(MOTOR_PORT,MOTOR1_PIN2,LOW);

DIO_voidSetPinVal(MOTOR_PORT,MOTOR2_PIN1,HIGH);
DIO_voidSetPinVal(MOTOR_PORT,MOTOR2_PIN2,LOW);
}
/*
 * DC_MOTOR_interface.h
 *
 * Created: 11/7/2025 10:40:30 AM
 *  Author: Al Jazeera
 */ 


#ifndef DC_MOTOR_INTERFACE_H_
#define DC_MOTOR_INTERFACE_H_

void DcMotor_voidInit(void);
void DcMotor_voidForward(u8 copyu8_speed);
void DcMotor_voidBackward(u8 copyu8_speed);
void DcMotor_voidRight(u8 copyu8_speed);
void DcMotor_voidLeft(u8 copyu8_speed);

#endif /* DC_MOTOR_INTERFACE_H_ */
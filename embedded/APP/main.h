/*
 * main.h
 *
 * Created: 9/6/2025 12:29:33 PM
 *  Author: hp
 */ 


#ifndef MAIN_H_
#define MAIN_H_

#include "../LIB/BIT_MATH.h"
#include "../LIB/STD_TYPE.h"
#include "../LIB/INTEERUPT_NUM.h"

#define F_CPU 16000000UL

#include <util/delay.h>

#include "../MCAL/DIO/DIO_interface.h"
#include "../MCAL/DIO/DIO_reg.h"
#include "../MCAL/GI/GI_interface.h"
#include "../MCAL/GI/GI_reg.h"
#include "../MCAL/ADC/ADC_interface.h"
#include "../MCAL/EXTI/EXTI_interface.h"
#include "../HAL/LED/LED_interface.h"
#include "../HAL/LCD/LCD_interface.h"
#include "../HAL/LCD/LCD_config.h"
#include "../HAL/KPD/KPD_interface.h"
#include "../HAL/KPD/KPD_config.h"
#include "../MCAL/TIM0/TIM0_interface.h"
#include "../MCAL/TIM0/TIM0_config.h"
#include "../MCAL/TIM2/TIM2_interface.h"
#include "../MCAL/TIM2/TIM2_config.h"
#include "../MCAL/TIM1/TIM1_interface.h"
#include "../MCAL/TIM1/TIM1_config.h"
#include "../HAL/DC_MOTOR/DC_MOTOR_interface.h"
#include "../HAL/DC_MOTOR/DC_MOTOR_config.h"
#include "../MCAL/UART/UART_interface.h"
#include "../MCAL/SPI/SPI_interface.h"
#include "../HAL/Encoder/ENCODER_interface.h" 

#endif /* MAIN_H_ */
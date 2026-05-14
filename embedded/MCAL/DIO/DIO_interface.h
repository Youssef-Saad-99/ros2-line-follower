/*
 * DIO_interface.h
 *
 * Created: 9/6/2025 10:42:29 AM
 *  Author: fathi
 */ 


#ifndef DIO_INTERFACE_H_
#define DIO_INTERFACE_H_




/* pin dir */

#define  OUTPUT           1 
#define  INPUT            0 
				         
/*PIN VAL */	         
				         
#define  HIGH             1 
#define  LOW              0 


/* dio ports */

#define  DIO_PORTA        0 
#define  DIO_PORTB        1
#define  DIO_PORTC        2
#define  DIO_PORTD        3



/*dio pins */


#define   DIO_PIN0         0
#define   DIO_PIN1         1
#define   DIO_PIN2         2
#define   DIO_PIN3         3
#define   DIO_PIN4         4
#define   DIO_PIN5         5
#define   DIO_PIN6         6
#define   DIO_PIN7         7



















/*PINS */
/*****************************************************************************
* Function Name: DIO_voidSetPinDir
* Purpose      : set pin dir (OUTPUT , INPUT)
* Parameters   : u8 Copy_u8_port,u8 Copy_u8_pin,u8 Copy_u8_dir
* Return value : void
*****************************************************************************/

void DIO_voidSetPinDir( u8 copy_u8_port, u8 copy_u8pin , u8 copy_u8dir) ; 
/*****************************************************************************
* Function Name: DIO_voidSetPinVal
* Purpose      : set pin value (HIGH , LOW)
* Parameters   : u8 copy_u8_port, u8 copy_u8pin ,u8 copy_u8val
* Return value : void
*****************************************************************************/

void DIO_voidSetPinVal( u8 copy_u8_port, u8 copy_u8pin ,u8 copy_u8val) ; 

/*****************************************************************************
* Function Name: DIO_voidTogglePinVal
* Purpose      : toggle pin value (HIGH , LOW)
* Parameters   : u8 copy_u8_port, u8 copy_u8pin 
* Return value : void
*****************************************************************************/

void DIO_voidTogglePinVal(u8 copy_u8_port, u8 copy_u8pin ) ; 

/*****************************************************************************
* Function Name: DIO_u8ReadPinVal
* Purpose      : read  pin value (HIGH , LOW)
* Parameters   : u8 copy_u8_port, u8 copy_u8pin
* Return value : u8
*****************************************************************************/



u8 DIO_u8ReadPinVal(u8 copy_u8_port, u8 copy_u8pin) ; 



/* ports */

/*****************************************************************************
* Function Name: DIO_voidSetPortDir
* Purpose      : set  port dirction (OUTPUT , INPUT)
* Parameters   : u8 copy_u8_port, u8 copy_u8dir
* Return value : void
*****************************************************************************/

void DIO_voidSetPortDir(u8 copy_u8_port , u8 copy_u8dir) ; 

/*****************************************************************************
* Function Name: DIO_voidSetPortVal
* Purpose      : set  port value (HIGH , LOW)
* Parameters   : u8 copy_u8_port, u8 copy_u8val
* Return value : void
*****************************************************************************/
void DIO_voidSetPortVal(u8 copy_u8_port , u8 copy_u8val) ;



















#endif /* DIO_INTERFACE_H_ */
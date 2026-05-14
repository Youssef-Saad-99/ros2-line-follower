/*
 * ADC_interface.h
 *
 * Created: 10/12/2025 9:16:49 AM
 *  Author: Al Jazeera
 */ 


#ifndef ADC_INTERFACE_H_
#define ADC_INTERFACE_H_




 //ADC Prescaler Selections
 #define ADC_2_PRESCALER	1
 #define ADC_4_PRESCALER	2
 #define ADC_8_PRESCALER	3
 #define ADC_16_PRESCALER	4
 #define ADC_32_PRESCALER	5
 #define ADC_64_PRESCALER	6
 #define ADC_128_PRESCALER	7
 
 
  #define  ADC_PRESCALER   ADC_128_PRESCALER
 
  #define ADC0		0
  #define ADC1	    1
  #define ADC2	    2
  #define ADC3	    3
  #define ADC4	    4
  #define ADC5	    5
  #define ADC6	    6
  #define ADC7	    7
 

 
 
 
void ADC_voidInit(void);

u16 ADC_u16ReadChannel(u8 copy_u8channel);





#endif /* ADC_INTERFACE_H_ */
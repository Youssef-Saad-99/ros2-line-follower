#include "main.h"
#include <string.h>
#include <avr/interrupt.h>

/* ADDED FOR ENCODER: „ﬂ »… «· ÕÊÌ· Êœ—«Ì›— «·«‰ﬂÊœ— */
#include <stdlib.h> 


#define IR_THRESHOLD  600
#define NUM_SENSORS   5

volatile char rx_buffer[10];
volatile uint8_t rx_index   = 0;
volatile uint8_t data_ready = 0;
volatile uint8_t handshake_done = 0;

// ?? ISR: UART Receive ????????????????????????????????????????????????????????
ISR(USART_RXC_vect) {
    char c = UDR;
    if (c == '\n' || c == '\r') {
        if (rx_index > 0) {
            rx_buffer[rx_index] = '\0';
            data_ready = 1;
            rx_index = 0;
        }
        } else {
        if (!data_ready) {
            if (rx_index < 9) rx_buffer[rx_index++] = c;
            else rx_index = 0;
        }
    }
}

// ?? Read all 5 IR sensors and send result to ROS ?????????????????????????????
// Format: "S:XXXXX\n" where X is 1 (line/dark) or 0 (no line/white)
static void read_and_send_ir(void) {
    uint8_t ir_array[NUM_SENSORS];
    char buf[10];   // "S:XXXXX\n\0"

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        uint16_t adc_val = ADC_u16ReadChannel(i);          // read channel i
        ir_array[i] = (adc_val < IR_THRESHOLD) ? 1 : 0;  // 1=dark(line), 0=white
    }

    buf[0] = 'S';
    buf[1] = ':';
    buf[2] = '0' + ir_array[0];
    buf[3] = '0' + ir_array[1];
    buf[4] = '0' + ir_array[2];
    buf[5] = '0' + ir_array[3];
    buf[6] = '0' + ir_array[4];
    buf[7] = '\n';
    buf[8] = '\0';

    UART_voidSendString(buf);
}

/* =======================================================================
 * ADDED FOR ENCODER: œ«·… ﬁ—«¡… «·⁄œ«œ«  Ê≈—”«·Â«
 * ======================================================================= */
static void send_encoder_data(void) {
    char buf[32];
    char temp[12];

    long l_ticks = (long)ENCODER_s32GetLeftTicks();
    long r_ticks = (long)ENCODER_s32GetRightTicks();

    buf[0] = 'E';
    buf[1] = ':';
    buf[2] = '\0';

    ltoa(l_ticks, temp, 10);
    strcat(buf, temp);
    strcat(buf, ",");
    ltoa(r_ticks, temp, 10);
    strcat(buf, temp);
    strcat(buf, "\n");

    UART_voidSendString(buf);
}
/* ======================================================================= */


// ?? Main ?????????????????????????????????????????????????????????????????????
int main(void) {
    char process_buffer[10];
    process_buffer[0] = '\0';
    uint8_t speed = 0;
    char dirc = 'S';

    DcMotor_voidInit();
    UART_voidInit();
    GI_voidEnable();
    ADC_voidInit();
    
    ENCODER_voidInit(); /* ADDED FOR ENCODER:  ÂÌ∆… «·«‰ﬂÊœ— */

    while (1) {

        // stream IR readings to ROS after handshake
        if (handshake_done) {
            read_and_send_ir();
            send_encoder_data(); /* ADDED FOR ENCODER: ≈—”«· «·œ« « ··—Ê“ */
            _delay_ms(50);
        }

        // process incoming UART commands
        if (data_ready) {
            cli();
            memcpy(process_buffer, (const char*)rx_buffer, 10);
            data_ready = 0;
            sei();

            if (strcmp(process_buffer, "PING") == 0) {
                UART_voidSendString("PONG\n");
                handshake_done = 1;

                } else if (handshake_done) {
                char received = process_buffer[0];

                if (received >= '0' && received <= '9') {
                    speed = (received - '0') * 10;
                    switch (dirc) {
                        case 'F': DcMotor_voidForward(speed);  break;
                        case 'B': DcMotor_voidBackward(speed); break;
                        case 'R': DcMotor_voidRight(speed);    break;
                        case 'L': DcMotor_voidLeft(speed);     break;
                        case 'S': /* stay stopped */           break;
                    }
                    } else {
                    dirc = received;
                    switch (received) {
                        case 'F': 
                            ENCODER_voidSetLeftDirection(1);  /* ADDED FOR ENCODER */
                            ENCODER_voidSetRightDirection(1); /* ADDED FOR ENCODER */
                            DcMotor_voidForward(speed);  
                            break;
                        case 'B': 
                            ENCODER_voidSetLeftDirection(-1); /* ADDED FOR ENCODER */
                            ENCODER_voidSetRightDirection(-1);/* ADDED FOR ENCODER */
                            DcMotor_voidBackward(speed); 
                            break;
                        case 'R': 
                            ENCODER_voidSetLeftDirection(1);  /* ADDED FOR ENCODER */
                            ENCODER_voidSetRightDirection(-1);/* ADDED FOR ENCODER */
                            DcMotor_voidRight(speed);    
                            break;
                        case 'L': 
                            ENCODER_voidSetLeftDirection(-1); /* ADDED FOR ENCODER */
                            ENCODER_voidSetRightDirection(1); /* ADDED FOR ENCODER */
                            DcMotor_voidLeft(speed);     
                            break;
                        case 'S': 
                            DcMotor_voidForward(0);      
                            break;
                    }
                }
            }
        }
    }
}
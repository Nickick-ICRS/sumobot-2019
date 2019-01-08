#include"mbed.h"
#include "TextLCD.h"

#define START 0x02
#define END 0x03
#define LEFT_PWM 0x04
#define RIGHT_PWM 0x05
#define LEFT_DIRECTION 0x06
#define RIGHT_DIRECTION 0x07

#define MAX_MESSAGE_LENGTH 8

Serial pc(USBTX, USBRX); // tx, rx
PwmOut red(LED1);
PwmOut blue(LED2);

TextLCD lcd(PTD6, PTD7, PTD1, PTD3, PTD2, PTD0);

volatile bool start_received;
volatile bool end_received;
volatile char msg[MAX_MESSAGE_LENGTH];
volatile char msg_ptr;

void receive_interrupt() {
    while(pc.readable()) {
        char c = pc.getc();
        switch(c) {
        case START:
            if(!start_received) {
                msg_ptr = 0;
                start_received = true;
            }
            break;
        case END:
            if(start_received && !end_received) {
                end_received = true;
            }
            break;
        default:
            if((msg_ptr < MAX_MESSAGE_LENGTH) && start_received 
                                              && !end_received) {
                msg[msg_ptr] = c;
                msg_ptr++;
            }
            break;
        }
    }
}

void process_msg() {
    // char lcd_message[16];
    for(char i = 0; i < msg_ptr; i+=2) {
        switch(msg[i]) {
        case LEFT_PWM:
            red = 1.0f - float(msg[i+1]) / 100.0f;
            // sprintf(lcd_message, "%s%s%d", lcd_message, "L", msg[i+1]);
            break;
        case RIGHT_PWM:
            blue = 1.0f - float(msg[i+1]) / 100.0f;
            // sprintf(lcd_message, "%s%s%d", lcd_message, "R", msg[i+1]);
            break;
        case LEFT_DIRECTION:
            // sprintf(lcd_message, "%s%s", lcd_message, "Ld");
            break;
        case RIGHT_DIRECTION:
            // sprintf(lcd_message, "%s%s", lcd_message, "Rd");
            break;
        default:
            break;
        }
    }
    // lcd.locate(0, 0);
    // lcd.printf("                ");
    // __disable_irq();
    // lcd.locate(1, 1);
    // lcd.printf("%s                ", lcd_message);
    // __enable_irq();
    start_received = false;
    end_received = false;
}

int main() {
    start_received = false;
    end_received = false;

    pc.attach(&receive_interrupt, Serial::RxIrq);

    lcd.printf("Hello World\n");
    
    pc.baud(9600);

    red.period(1.0f/10000.0f);
    blue.period(1.0f/10000.0f);
    while(1) {
        if(end_received)
            process_msg();
    }
}

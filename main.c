
// PIC16F1705 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit cannot be cleared once it is set by software)
#pragma config ZCDDIS = ON      // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PLLEN = OFF      // Phase Lock Loop enable (4x PLL is enabled when software sets the SPLLEN bit)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ 16000000
#define _SLAVE_ADDRESS 4
#define PWM_PERCENT_COMMAND 1
#define WHEEL_DIR_COMMAND 2
#define LED_COMMAND 5

#include <xc.h>
#include "pic_libs/i2c.h"
#include "pic_libs/pwm.h"
#include "pic_libs/speed.h"


char pwm_speed = 0;

char get_led(){
    return RC3;
}

void set_led(char on_off){
    RC3 = (__bit) on_off;
}

void setup_led(){
    ANSELC = 0;
    TRISC3 = 0;
    RC3 = 0;
}

void setup_dir(){
    TRISA5 = 0;
    RA5 = 0;
}

char get_dir(){
    return RA5;
}

void set_dir(char fw_bw){
    RA5 = (__bit) fw_bw;
}

char on_byte_read(char offset){
    switch(offset){
        case LED_COMMAND:
            return get_led();
            break;
        case PWM_PERCENT_COMMAND:
            return pwm_speed;
            break;
        case WHEEL_DIR_COMMAND:
            return get_dir();
            break;
        default:
            return 0xFF;
    }
}

void on_byte_write(char offset, char byte){
    switch(offset){
        case LED_COMMAND:
            set_led(byte);
            break;
        case PWM_PERCENT_COMMAND:
            pwm_speed = byte;
            set_duty_percent(pwm_speed);
            break;
        case WHEEL_DIR_COMMAND:
            set_dir(byte);
            break;
    }
}

char hi_ = 0, lo_ = 0;
void on_speed_interrupt(char hi, char lo){
    hi_ = hi;
    lo_ = lo;
}

void setup_clock(){
    IRCF3 = 1;
    IRCF2 = 1;
    IRCF1 = 1;
    IRCF0 = 1;
}

void setup(){
    setup_clock();
    setup_led();
    // TODO: Check if this ANSEL line can be removed
    ANSELA = 0;
    setup_dir();
    setup_pwm();
    // slave on address _SLAVE_ADDRESS
    setup_i2c(0, _SLAVE_ADDRESS, on_byte_write, on_byte_read);
    setup_speed(on_speed_interrupt);
}

void __interrupt() int_routine(void){
    if (SSPIF){ // received data through i2c
        process_interrupt_i2c();
    } else if(CCP2IF){
        process_interrupts_speed();
    }
}

void main(void) {
    setup();
    
    while(1){
        continue;
    }
}

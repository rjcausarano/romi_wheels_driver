
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

/*
 * This PIC is a driver to a robot wheel and thus is in charge of applying the 
 * commanded velocity to the DC motor that it is attached to. This chip has to 
 * manage I2C communication to the master, calculate the current wheel speed and
 * apply the correct PWM output to the wheel via PID to match the requested 
 * wheel velocity requested by the master. 
 * 
 * The code below has the goal of maximizing performance. Fast response times 
 * at a vast range of wheel speeds is prefferred at the cost of more power drawn
 * and maybe more resources could be used.
 * 
 * Interrupts are kept to a minimum to delay PID and I2C communication 
 * processing as least as possible.
 * 
 * If you think you can improve this chip's performance PLEASE OPEN A PR!! It
 * will be greatly appreciated.
 */

#define _XTAL_FREQ 16000000
#define INSTRUCTION_FREQ 4000000
// Here we set whether this PIC controls the left of right wheel.
#define _SLAVE_ADDRESS 3
#define MOTOR_ENABLE_OFFSET 0
#define PWM_PERCENT_OFFSET 1
#define WHEEL_DIR_OFFSET 2
#define SPEED_LO_OFFSET 3
#define SPEED_HI_OFFSET 4
#define LED_OFFSET 5
#define PWM_LO_OFFSET 6
#define PWM_HI_OFFSET 7

#include <xc.h>
#include "pic_libs/i2c.h"
#include "pic_libs/pwm.h"
#include "encoder.h"

unsigned char pwm_speed_percent_ = 0, pwm_pid_speed_lo_ = 0, 
        is_right_wheel_ = 0;
unsigned int pwm_period_us_ = 0, encoder_counts_ = 0;
int pwm_pid_speed_ = 0;

char get_led(){
    return RC3;
}

void set_led(char on_off){
    RC3 = (__bit) on_off;
}

void enable_motor(char on_off){
    RC5 = (__bit) on_off;
    set_led(on_off);
}

char get_motor(){
    return RC5;
}

void setup_led(){
    TRISC3 = 0;
    RC3 = 0;
}

void setup_motor(){
    TRISC5 = 0;
    enable_motor(0);
    TRISC4 = 1;
    is_right_wheel_ = RC4;
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

void set_pwm_pid_speed(){
    int speed = (int) get_duty_cycle_pwm();
    speed += pwm_pid_speed_;
    if(speed < 0){
        speed = 0;
    } else if (speed > 1023){
        speed = 1023;
    }
    set_duty_cycle_pwm((unsigned int) speed);
}

char on_byte_read(char offset){
    unsigned char ret = 0xFF;
    switch(offset){
        case SPEED_LO_OFFSET:
            IOCIE = 0;
            ret = (char) encoder_counts_;
            break;
        case SPEED_HI_OFFSET:
            ret = (char) (encoder_counts_ >> 8);
            encoder_counts_ = 0;
            IOCIE = 1;
            break;
        case LED_OFFSET:
            ret = get_led();
            break;
        case PWM_PERCENT_OFFSET:
            ret = pwm_speed_percent_;
            break;
        case WHEEL_DIR_OFFSET:
            ret = get_dir();
            break;
        case MOTOR_ENABLE_OFFSET:
            ret = get_motor();
            break;
    }
    return ret;
}

void on_byte_write(char offset, char byte){
    switch(offset){
        case PWM_LO_OFFSET:
            pwm_pid_speed_lo_ = byte;
            break;
        case PWM_HI_OFFSET:
            pwm_pid_speed_ = ((int) byte) << 8;
            pwm_pid_speed_ += pwm_pid_speed_lo_;
            set_pwm_pid_speed();
            break;
        case PWM_PERCENT_OFFSET:
            pwm_speed_percent_ = byte;
            set_duty_percent_pwm(pwm_speed_percent_);
            break;
        case WHEEL_DIR_OFFSET:
            set_dir(byte);
            break;
        case MOTOR_ENABLE_OFFSET:
            enable_motor(byte);
            break;
    }
}

void setup_clock(){
    IRCF3 = 1;
    IRCF2 = 1;
    IRCF1 = 1;
    IRCF0 = 1;
}

void setup(){
    ANSELA = 0;
    ANSELC = 0;

    setup_clock();
    setup_led();
    setup_motor();
    setup_dir();
    setup_pwm(16);
    pwm_period_us_ = get_period_us_pwm();
    period_interrupt_pwm(0); // interrupt on every full period
    // slave on address _SLAVE_ADDRESS
    setup_i2c(0, _SLAVE_ADDRESS + is_right_wheel_, on_byte_write, on_byte_read);
    setup_encoder();
}

void __interrupt() int_routine(void){
    if(SSPIF){ // received data through i2c
        process_interrupt_i2c();
    } else if(IOCIF){
        IOCIF = 0;
        IOCAF = 0;
        IOCCF = 0;
        encoder_counts_++;
    }
}

void main(void) {
    setup();
    while(1){
        continue;
    }
}
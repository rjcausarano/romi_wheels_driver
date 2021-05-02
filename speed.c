#include "speed.h"
#include <pic16f1705.h>

// 360 counts per wheel revolution

static void map_pins(){
    // RC0 to count channel A (T1CKI)
    T1CKIPPS = 0b10000;
}

static void pins_setup(){
    // RC0 is an input
    TRISC0 = 1;
    // RA2 is an input
    TRISA2 = 1;
}

void setup_speed(){
    pins_setup();
    map_pins();
    
    // Enable TMR1 as counter
    TMR1ON = 1;
    TMR1GE = 0;
    
    // TMR1 clock source set to external T1CKI pin
    T1CONbits.TMR1CS = 0b10;
    T1CONbits.T1OSCEN = 0;
    
    //TMR1 prescaler set to 1
    T1CONbits.T1CKPS = 0b00;
    
    // run asynchronously
    T1CONbits.nT1SYNC = 1;

    // reset tmr1
    TMR1L = 0;
    TMR1H = 0;
    //Interrupts
    GIE = 1;
    PEIE = 1;
    TMR1IE = 1;
    TMR1IF = 0;
}
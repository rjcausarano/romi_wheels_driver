#include "encoder.h"
#include <pic16f1705.h>

// 360 rising counts per wheel revolution per channel
// 720 counts if counting both edges
// 1440 counts if counting both channels


static void pins_setup(){
    // RC0 is an input
    TRISC0 = 1;
    // RA2 is an input
    TRISA2 = 1;
    
    IOCAN2 = 1; // Interrupt when RA2 goes negative
    IOCAP2 = 1; // Interrupt when RA2 goes positive
    
    IOCCN0 = 1; // Interrupt when RC0 goes negative
    IOCCP0 = 1; // Interrupt when RC0 goes positive
}

void setup_encoder(){
    pins_setup();
    
    //Interrupts
    GIE = 1;
    PEIE = 1;
    IOCIE = 1;
    IOCIF = 0;
}
/*
#include <xc.h>

#define TIMER1 1
#define TIMER2 2
#define FOSC 144000000

void tmr_setup_period(int timer, int ms) {
    int prescaler = 1;
    long fcy = (FOSC / 2) * (ms / 1000.0); // number of clocks in one second
    long fcy_new = fcy;

    if (fcy > 65535) {
        fcy_new = fcy / 8;
        prescaler = 1; // prescaler 1:8
    }
    if (fcy_new > 65535) {
        fcy_new = fcy / 64;
        prescaler = 2; // prescaler 1:64
    }
    if (fcy_new > 65535) {
        fcy_new = fcy / 256;
        prescaler = 3; // prescaler 1:256
    }

    switch(timer) {
        case TIMER1: {
            TMR1 = 0; // reset T1 counter
            T1CONbits.TCKPS = prescaler;
            PR1 = fcy_new;
            T1CONbits.TON = 1; // start T1
        }
        break;
        
        case TIMER2: {
            TMR2 = 0; // reset T2 counter
            T2CONbits.TCKPS = prescaler;
            PR2 = fcy_new;
            T2CONbits.TON = 1; // start T2
        }
        break;
    }
}

void tmr_wait_period(int timer) {
    switch(timer) {
        case TIMER1: {
            while(!IFS0bits.T1IF);
            IFS0bits.T1IF = 0;
        }
        break;

        case TIMER2: {
            while(!IFS0bits.T2IF);
            IFS0bits.T2IF = 0;
        }
        break;
    }
}

void tmr_setup_ms(int timer) {
    switch (timer) {
        case TIMER1: {
            TMR1 = 0; // reset T1 counter
            IFS0bits.T1IF = 0; // reset T1 flag
            T1CONbits.TON = 1; // start T1
        }
        break;
        
        case TIMER2: {
            TMR2 = 0; // reset T2 counter
            IFS0bits.T2IF = 0; // reset T2 flag
            T2CONbits.TON = 1; // start T2
        }
        break;
    }
}

void tmr_wait_ms(int timer, int ms) {
    int prescaler = 1;
    long fcy = (FOSC / 2) * (ms / 1000.0);
    long fcy_new = fcy;

    if (fcy > 65535) {
        fcy_new = fcy / 8;
        prescaler = 1; // prescaler 1:8
    }
    if (fcy_new > 65535) {
        fcy_new = fcy / 64;
        prescaler = 2; // prescaler 1:64
    }
    if (fcy_new > 65535) {
        fcy_new = fcy / 256;
        prescaler = 3; // prescaler 1:256
    }

    switch (timer) {
        case TIMER1: {
            T1CONbits.TCKPS = prescaler;
            PR1 = fcy_new;

            // wait...
            while(!IFS0bits.T1IF);
            IFS0bits.T1IF = 0;
        }
        break;

        case TIMER2: {
            T2CONbits.TCKPS = prescaler;
            PR2 = fcy_new;

            // wait...
            while(!IFS0bits.T2IF);
            IFS0bits.T2IF = 0;
        }
        break;
    }
}

void __attribute__ (( __interrupt__ , __auto_psv__ )) _INT1Interrupt() {
    IFS1bits.INT1IF = 0; // reset interrupt flag

    // start timer form 20ms
    tmr_setup_period(TIMER2, 10);
}

void __attribute__ (( __interrupt__ , __auto_psv__ )) _T2Interrupt() {
    IFS0bits.T2IF = 0; // reset interrupt flag

    int pinValue = 0;
    pinValue = PORTEbits.RE8;

    T2CONbits.TON = 0; // stop T2
    if (!pinValue)
        LATAbits.LATA0 = !LATAbits.LATA0;
}

void initPins() {
    TRISAbits.TRISA0 = 0;
    TRISGbits.TRISG9 = 0;
    TRISEbits.TRISE8 = 1;
}

int main() {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    initPins();

   // Remap INT1 to btn E8
    RPINR0bits.INT1R = 0x58;
    INTCON2bits.GIE = 1;
    INTCON2bits.INT1EP = 1;
    IFS1bits.INT1IF = 0;

    IEC0bits.T2IE = 1;
    IEC1bits.INT1IE = 1;

    // setup and start timer
    tmr_setup_period(TIMER1, 200);

    while(1) {
        LATGbits.LATG9 = !LATGbits.LATG9;
        tmr_wait_period(TIMER1);
    }

    return 0;
}
*/
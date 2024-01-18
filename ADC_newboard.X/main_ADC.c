#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

void initUART2() {
    const int baund = 9600;
    U2BRG = (FOSC / 2) / (16L * baund) - 1; // = 11
    U2MODEbits.UARTEN = 1; // enable UART2
    U2STAbits.UTXEN = 1; // enable U2TX (must be after UARTEN)
}

void initADC1() {
    AD1CON3bits.ADCS = 14; // Tad
    AD1CON1bits.ASAM = 1; // manual sampling
    AD1CON1bits.SSRC = 7; // automatic convertion
    AD1CON3bits.SAMC = 16; // how long the sampling should last
    AD1CON2bits.CHPS = 0; // how many channels you want to use
    AD1CHS0bits.CH0SA = 5; // selects the inputs to channel 0: Select AN5 for CH0 +ve input

    ANSELBbits.ANSB15 = 1; // Set the appropriate bits to 0 for analog input pins
    TRISBbits.TRISA3 = 0; // enable pin sensor
    LATBbits.LATA3 = 1;

    AD1CON1bits.ADON = 1; // turn on ADC1
}
int main() {    
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;

    initUART2();
    initADC1();
    
    char buff[16];

    // Remap UART2 pins
    RPOR0bits.RP64R = 0x03;
    RPINR19bits.U2RXR = 0x4B;
    
    TRISAbits.TRISA0 = 0;

    tmr_setup_period(TIMER1, 1);

    while(1) {
        while (!AD1CON1bits.DONE);

        // formula altezza
        float read_value = ADC1BUF0;
        float v = read_value / 1023.0 * 3.3; // Value in Volts
        float value = 2.34 - 4.74 * v + 4.06 * v*v - 1.60 * v*v*v + 0.24 * v*v*v*v;

        sprintf(buff, "%f", value);
        for (int i = 0; i < strlen(buff); i++){
            while (!U2STAbits.TRMT); // Wait for UART2 transmit buffer to be empty
            U2TXREG = buff[i];
        }

        if (value < 0.5)
            LATAbits.LATA0 = 1;
        else
             LATAbits.LATA0 = 0;

        tmr_wait_period(TIMER1);
    }

    return 0;
}

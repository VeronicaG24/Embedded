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

void initPins() {
    TRISAbits.TRISA0 = 0;
    TRISGbits.TRISG9 = 0;
    TRISEbits.TRISE8 = 1;
}

void initUART2() {
    const int baund = 9600;
    U2BRG = (FOSC / 2) / (16L * baund) - 1; // = 11
    U2MODEbits.UARTEN = 1; // enable UART2
    U2STAbits.UTXEN = 1; // enable U2TX (must be after UARTEN)
}

void initADC1() {
    AD1CON3bits.ADCS = 7; // Tad
    AD1CON1bits.ASAM = 0; // manual sampling
    AD1CON1bits.SSRC = 7; // automatic convertion
    AD1CON3bits.SAMC = 15; // how long the sampling should last
    AD1CON2bits.CHPS = 0; // how many channels you want to use
    AD1CHS0bits.CH0SA = 5; // selects the inputs to channel 0: Select AN5 for CH0 +ve input
    //AD1CHS123bits.CH123SA = 2 ; // selects the inputs to channels 1, 2 and 3:  Select AN2 for CH1 +ve input
    ANSELBbits.ANSB5 = 1; // Set the appropriate bits to 0 for analog input pins
    TRISBbits.TRISB4 = 1; // enable pin sensor
    AD1CON1bits.ADON = 1; // turn on ADC1     
}
int main() {
    
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    initPins();
    initUART2();
    
    char buff[16];
    float read_value;
    float y;
    
   
    
    initADC1();

    // Remap UART2 pins
    RPOR0bits.RP64R = 0x03;
    RPINR19bits.U2RXR = 0x4B;

    // start sampling
    AD1CON1bits.SAMP = 1;
    while(!AD1CON1bits.DONE);
    
    // Read from sensor
    read_value = ADC1BUF0; 
    
    // formula altezza
    y = 2.34 - 4.74*read_value + 4.06 * read_value*read_value - 1.60 * read_value*read_value*read_value + 0.24 * read_value*read_value*read_value*read_value;
    
    
    sprintf(buff, "%d", y);
    for (int i = 0; i < strlen(buff); i++){
        while (!U2STAbits.TRMT); // Wait for UART2 transmit buffer to be empty
        U2TXREG = buff[i];
    }
    while(1);

    return 0;
}

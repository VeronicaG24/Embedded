#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FOSC 144000000

void initUART2() {
    const int baund = 9600;
    U2BRG = (FOSC / 2) / (16L * baund) - 1; // = 11
    U2MODEbits.UARTEN = 1; // enable UART2
    U2STAbits.UTXEN = 1; // enable U2TX (must be after UARTEN)
}

void initADC1() {
    AD1CON3bits.ADCS = 8; // Tad
    AD1CON1bits.ASAM = 0; // manual sampling
    AD1CON1bits.SSRC = 7; // automatic convertion
    AD1CON3bits.SAMC = 15; // how long the sampling should last
    AD1CON2bits.CHPS = 0; // how many channels you want to use
    AD1CHS0bits.CH0SA = 5; // selects the inputs to channel 0: Select AN5 for CH0 +ve input
    //AD1CHS123bits.CH123SA = 2 ; // selects the inputs to channels 1, 2 and 3:  Select AN2 for CH1 +ve input
    ANSELBbits.ANSB5 = 1; // Set the appropriate bits to 0 for analog input pins
    TRISBbits.TRISB4 = 0; // enable pin sensor
    LATBbits.LATB9 = 1;
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

    // start sampling
    AD1CON1bits.SAMP = 1;
    while(!AD1CON1bits.DONE);
    
    // formula altezza
    float read_value = ADC1BUF0;
    float v = read_value / 1023.0 * 3.3; // Value in Volts
    float value = 2.34 - 4.74 * v + 4.06 * v*v - 1.60 * v*v*v + 0.24 * v*v*v*v;
    
    sprintf(buff, "%f", value);
    for (int i = 0; i < strlen(buff); i++){
        while (!U2STAbits.TRMT); // Wait for UART2 transmit buffer to be empty
        U2TXREG = buff[i];
    }
    while(1);

    return 0;
}

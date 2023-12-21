#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIMER1 1
#define TIMER2 2
#define FOSC 144000000
#define MINTH 2.0
#define MAXTH 10.0

int start = 0;
int startCount = 0;

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

    // start timer form 10ms
    tmr_setup_period(TIMER2, 10);
}

void __attribute__ (( __interrupt__ , __auto_psv__ )) _T2Interrupt() {
    IFS0bits.T2IF = 0; // reset interrupt flag

    int pinValue = 0;
    pinValue = PORTEbits.RE8;

    T2CONbits.TON = 0; // stop T2
    if (!pinValue) {
        start = !start;
        startCount = 0;
    }
}

void initPins() {
    TRISAbits.TRISA0 = 0; // led A0 as output
    TRISEbits.TRISE8 = 1; // Btn E8 as input
    TRISBbits.TRISB8 = 0; // Left lights
    TRISFbits.TRISF1 = 0; // Right lights
    TRISFbits.TRISF0 = 0; // Breaks
    TRISGbits.TRISG1 = 0; // Low intensity lights
    TRISAbits.TRISA7 = 0; // Beam headlights
}

void initUART2() {
    const int baund = 9600;
    U2BRG = (FOSC / 2) / (16L * baund) - 1;
    U2MODEbits.UARTEN = 1; // enable UART2
    U2STAbits.UTXEN = 1; // enable U2TX (must be after UARTEN)
}

void remapUARTPins() {
    RPOR0bits.RP64R = 0x03;
    RPINR19bits.U2RXR = 0x4B;
}

// IR sensor init
void initADC1() {
    AD1CON3bits.ADCS = 7; // Tad
    AD1CON1bits.ASAM = 0; // manual sampling
    AD1CON1bits.SSRC = 7; // automatic convertion
    AD1CON3bits.SAMC = 15; // how long the sampling should last
    AD1CON2bits.CHPS = 0; // how many channels you want to use
    AD1CHS0bits.CH0SA = 5; // selects the inputs to channel 0: Select AN5 for CH0 +ve input
    //AD1CHS123bits.CH123SA = 2 ; // selects the inputs to channels 1, 2 and 3:  Select AN2 for CH1 +ve input
    ANSELBbits.ANSB14 = 0; // Set the appropriate bits to 0 for analog input pins
    TRISBbits.TRISB9 = 1;
    LATBbits.LATB9 = 1;
    AD1CON1bits.ADON = 1; // turn on ADC1     
}

// Battery sensor init
void initADC2() {
    AD2CON3bits.ADCS = 7; // Tad
    AD2CON1bits.ASAM = 0; // manual sampling
    AD2CON1bits.SSRC = 7; // automatic convertion
    AD2CON3bits.SAMC = 15; // how long the sampling should last
    AD2CON2bits.CHPS = 0; // how many channels you want to use
    AD2CHS0bits.CH0SA = 5; // selects the inputs to channel 0: Select AN5 for CH0 +ve input
    //AD1CHS123bits.CH123SA = 2 ; // selects the inputs to channels 1, 2 and 3:  Select AN2 for CH1 +ve input
    ANSELBbits.ANSB11 = 1; // Set the appropriate bits to 0 for analog input pins
    AD2CON1bits.ADON = 1; // turn on ADC1   
}

void initOCPWM() {
    // OC1
    OC1CON1bits.OCTSEL = 7; // Peripheral clock
    OC1CON2bits.SYNCSEL = 0x1F; // No sync source
    OC1CON1bits.OCM = 6; // Edge-aligned PWM mode
    
    // OC2
    OC2CON1bits.OCTSEL = 7; // Peripheral clock
    OC2CON2bits.SYNCSEL = 0x1F; // No sync source
    OC2CON1bits.OCM = 6; // Edge-aligned PWM mode

    // OC3
    OC3CON1bits.OCTSEL = 7; // Peripheral clock
    OC3CON2bits.SYNCSEL = 0x1F; // No sync source
    OC3CON1bits.OCM = 6; // Edge-aligned PWM mode

    // OC4
    OC4CON1bits.OCTSEL = 7; // Peripheral clock
    OC4CON2bits.SYNCSEL = 0x1F; // No sync source
    OC4CON1bits.OCM = 6; // Edge-aligned PWM mode
}

void remapOCPins() {
    RPOR0bits.RP65R = 0x10;
    RPOR1bits.RP66R = 0x11;
    RPOR1bits.RP67R = 0x12;
    RPOR2bits.RP68R = 0x13;
}

void remapINTPins() {
    // Remap INT1 to btn E8
    RPINR0bits.INT1R = 0x58;
    INTCON2bits.GIE = 1;
    INTCON2bits.INT1EP = 1;
    IFS1bits.INT1IF = 0;
}

void stopMotors() {
    OC1R = 0;
    OC2R = 0;
    OC3R = 0;
    OC4R = 0;
}

void forward() {
    
}

void turn() {
    
}

void blinkLed(int ms) {
    if (startCount == ms)
    {
        LATAbits.LATA0 = !LATAbits.LATA0;
        startCount = 0;
    }
    startCount++;
}

double computeDist(double read_val) {
    double v = read_val / 1023.0 * 3.3; // Value in Volts
    return 2.34 - 4.74*v + 4.06 * v*v - 1.60 * v*v*v + 0.24 * v*v*v*v;
}

int main() {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;

    initPins();
    // initUART2();
    initADC1();
    initOCPWM();

    // remapUARTPins();
    remapOCPins();
    remapINTPins();
    
    // Enable interrupts
    IEC0bits.T2IE = 1;
    IEC1bits.INT1IE = 1;
    
    // Start sampling
    AD1CON1bits.SAMP = 1;
    while(!AD1CON1bits.DONE);
    
    float value;
    double surge;
    double yaw_rate;
    // char buff[16];

    tmr_setup_period(TIMER1, 1);

    while(1) {
        // While in wait for start...
        if (!start)
        {
            stopMotors();
        }
        else {
            // Update PWM and read sensors at 1kHz
            // Read from sensor
            // value = computeDist(ADC1BUF0);
            float read_value = ADC1BUF0;
            float v = read_value / 1023.0 * 3.3; // Value in Volts
            value = 2.34 - 4.74*v + 4.06 * v*v - 1.60 * v*v*v + 0.24 * v*v*v*v;
            // sprintf(buff, "%f", value);
            // for (int i = 0; i < strlen(buff); i++){
            //    while (!U2STAbits.TRMT); // Wait for UART2 transmit buffer to be empty
            //    U2TXREG = buff[i];
            // }
            if (value > MINTH && value < MAXTH) {
                LATBbits.LATB8 = 1;
                OC1R += 500;
                OC3R += 500;
                if (OC1R > 4000) OC1R = 4000;
                if (OC3R > 4000) OC3R = 4000;
            }
            else {
                LATFbits.LATF1 = 1;
                OC1R -= 500;
                OC3R -= 500;
                if (OC1R < 0) OC1R = 0;
                if (OC3R < 0) OC3R = 0;
            }

            // LATBbits.LATB8 = 1;
            // LATFbits.LATF1 = 1;
            // LATFbits.LATF0 = 1;
            // LATGbits.LATG1 = 1;
            // LATAbits.LATA7 = 1;
        }

        blinkLed(1000);
        tmr_wait_period(TIMER1);
    };

    return 0;
}
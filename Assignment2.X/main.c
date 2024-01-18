#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIMER1 1
#define TIMER2 2
#define FOSC 144000000
#define MINTH 0.25
#define MAXTH 0.6
#define INCR 500
#define OC_MAX 9000

int start = 0;
int startCount = 0;
int blinkRightLight = 0;

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
    // IR Sensor analog configuratiom AN15
    TRISBbits.TRISB15 = 1;
    ANSELBbits.ANSB15 = 1;
    // Battery sensing analog configuration AN11
    TRISBbits.TRISB11 = 1;
    ANSELBbits.ANSB11 = 1;
    
    AD1CON3bits.ADCS = 14; // 14*T_CY
    AD1CON1bits.ASAM = 1; // automatic sampling start
    AD1CON1bits.SSRC = 7; // automatic conversion
    AD1CON3bits.SAMC = 16; // sampling lasts 16 Tad
    AD1CON2bits.CHPS = 0; // use CH0 2-channels sequential sampling mode
    AD1CON1bits.SIMSAM = 0; // sequential sampling

	// Scan mode specific configuration
	AD1CON2bits.CSCNA = 1; // scan mode enabled
    AD1CSSLbits.CSS11 = 1;   // scan for AN11 battery
    AD1CSSLbits.CSS15 = 1;   // scan for AN15 ir sensor
	AD1CON2bits.SMPI = 1; // N-1 channels

    AD1CON1bits.ADON = 1; // turn on ADC

    // IR distance sensor enable line
    TRISAbits.TRISA3 = 0;
    LATAbits.LATA3 = 1;
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

    OC1RS = OC_MAX;
    OC2RS = OC_MAX;
    OC3RS = OC_MAX;
    OC4RS = OC_MAX;
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

void blinkLed(int ms) {
    if (startCount == ms)
    {
        LATAbits.LATA0 = !LATAbits.LATA0;

        if (!start) {
            LATBbits.LATB8 = !LATBbits.LATB8;
            LATFbits.LATF1 = !LATFbits.LATF1;
        }
        
        if (blinkRightLight && start) LATFbits.LATF1 = !LATFbits.LATF1;

        startCount = 0;
    }
    startCount++;
}

double computeDist(double read_val) {
    double v = read_val / 1023.0 * 3.3; // Value in Volts
    return 2.34 - 4.74*v + 4.06 * v*v - 1.60 * v*v*v + 0.24 * v*v*v*v;
}

void setPWM_Left(int pwm) {
    if (pwm > 0) {
        // Set PWM for forward motion
        OC1R = 0;
        OC2R = pwm;
    } else {
        // Set PWM for backward motion
        OC1R = pwm;
        OC2R = 0;
    }
}
 
void setPWM_Right(int pwm) {
    if (pwm > 0) {
        // Set PWM for forward motion
        OC3R = 0;
        OC4R = pwm;
    } else {
        // Set PWM for backward motion
        OC3R = pwm;
        OC4R = 0;
    }
}

double computeSurge(double dist) {
    double surge = 0;
    
    if (dist < MINTH) surge = 0;
    else if (dist > MAXTH) surge = 0.8;
    else {
        surge = OC_MAX / 4 + (OC_MAX - OC_MAX / 4) * (dist - MINTH) / (MAXTH - MINTH);
        surge = surge / OC_MAX;
    }

    return surge;
}

double computeYaw(double dist) {
    double yaw_rate = 0;
    
    if (dist < MINTH) yaw_rate = 0.8;
    else if (dist > MAXTH) yaw_rate = 0;
    else {
        yaw_rate = OC_MAX * (1.0 / 4 + (1.0 - 1.0 / 4) * (dist - MINTH) / (MAXTH - MINTH));
        yaw_rate = yaw_rate / OC_MAX;
    }

    return yaw_rate;
}

/* void controlRobotBasedOnDistance(double sensedDistance) {
    if (sensedDistance < MINTH) {
        surge = 0;
        yaw = 0.4 * OC_VAL;

        // Turn clockwise on the spot
        setPWM_Left(surge-yaw);  // Reverse left
        setPWM_Right(surge+yaw);  // Forward right
        
    } else if (sensedDistance > MAXTH) {
        surge = -0.6 * OC_VAL;
        yaw = 0;
        // Go forward
        setPWM_Left(surge-yaw);   // Forward left
        setPWM_Right(surge+yaw);  // Forward right
    } else {
        surge = OC_VAL / 8 + (OC_VAL / 2 - OC_VAL / 8) * (sensedDistance - MINTH) / (MAXTH - MINTH);
        surge = -surge;
        yaw = OC_VAL *(1.0 / 8 + (1.0 / 2 - 1.0 / 8) * (sensedDistance - MINTH) / (MAXTH - MINTH));
        // modificare sopra
        setPWM_Left(2*(surge-yaw)); 
        setPWM_Right(4*(surge+yaw-1000));
    }
} */

void printUART(double value, char* buff) {
    sprintf(buff, "%.2f ", value);
    for (int i = 0; i < strlen(buff); i++) {
        while (!U2STAbits.TRMT); // Wait for UART2 transmit buffer to be empty
        U2TXREG = buff[i];
    }
}

void checkLights(double surge, double yaw_rate) {
    if (surge > 0.5) {
        LATAbits.LATA7 = 1;
        LATFbits.LATF0 = 0;
        LATGbits.LATG1 = 0;
    }
    else {
        LATAbits.LATA7 = 0;
        LATFbits.LATF0 = 1;
        LATGbits.LATG1 = 1;
    }
    
    if (yaw_rate < 0.15) {
        LATBbits.LATB8 = 0;
        blinkRightLight = 1;
    }
    else {
        blinkRightLight = 0;
        LATBbits.LATB8 = 0;
        LATFbits.LATF1 = 0;
    }
}

void turnOffLights() {
    LATFbits.LATF0 = 0;
    LATGbits.LATG1 = 0;
    LATAbits.LATA7 = 0;
}
        
int main() {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;

    initPins();
    initUART2();
    initADC1();
    initOCPWM();

    remapUARTPins();
    remapOCPins();
    remapINTPins();
    
    // Enable interrupts
    IEC0bits.T2IE = 1;
    IEC1bits.INT1IE = 1;
    
    double v, dist, adc_battery, adc_ir;
    double surge, yaw_rate;
    char buff[16];

    tmr_setup_period(TIMER1, 1);

    while(1) {
        if (!start) {
            turnOffLights();
            stopMotors();
        }
        else {
            while (!AD1CON1bits.DONE);
            adc_battery = ADC1BUF0;
            adc_ir = ADC1BUF1;
            dist = computeDist(adc_ir);

            printUART(dist, buff);

            surge = computeSurge(dist);
            yaw_rate = computeYaw(dist);
            
            checkLights(surge, yaw_rate);

            setPWM_Left(OC_MAX * (surge+yaw_rate));
            setPWM_Right(OC_MAX * (surge-yaw_rate));
        }

        blinkLed(1000);
        tmr_wait_period(TIMER1);
    };

    return 0;
}

// LATBbits.LATB8 = 1;
// LATFbits.LATF1 = 1;
// LATFbits.LATF0 = 1;
// LATGbits.LATG1 = 1;
// LATAbits.LATA7 = 1;
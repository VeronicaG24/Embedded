// Assignment 2 - Embedded Systems
// Group 6
// Gabriele Nicchiarelli - S4822677
// Veronica Gavagna - S5487110
// Andrea Bolla - S4482930

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "parser.h"

#define TIMER1 1
#define TIMER2 2
#define FOSC 144000000
#define MINTH 0.2
#define MAXTH 0.5
#define OC_MAX 14000.0
#define BUFF_SIZE 16

// Circular buffer structure
typedef struct {
    char buff[BUFF_SIZE];
    int readIdx;
    int writeIdx;
} CircBuff;

CircBuff circBuff;

int start = 0;
int startCount = 0;
int countTx = 0;
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

// UART2 interrupt
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt() {
    IFS1bits.U2RXIF = 0;  // Reset del flag di interrupt

    while (U2STAbits.URXDA)
        buffWrite(&circBuff, U2RXREG);
}

int parse_byte(parser_state* ps, char byte) {
    switch (ps->state) {
        case STATE_DOLLAR:
            if (byte == '$') {
                ps->state = STATE_TYPE;
                ps->index_type = 0;
            }
            break;
        case STATE_TYPE:
            if (byte == ',') {
                ps->state = STATE_PAYLOAD;
                ps->msg_type[ps->index_type] = '\0';
                ps->index_payload = 0; // initialize properly the index
            } else if (ps->index_type == 6) { // error! 
                ps->state = STATE_DOLLAR;
                ps->index_type = 0;
			} else if (byte == '*') {
				ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_type[ps->index_type] = '\0';
				ps->msg_payload[0] = '\0'; // no payload
                return NEW_MESSAGE;
            } else {
                ps->msg_type[ps->index_type] = byte; // ok!
                ps->index_type++; // increment for the next time;
            }
            break;
        case STATE_PAYLOAD:
            if (byte == '*') {
                ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_payload[ps->index_payload] = '\0';
                return NEW_MESSAGE;
            } else if (ps->index_payload == 100) { // error
                ps->state = STATE_DOLLAR;
                ps->index_payload = 0;
            } else {
                ps->msg_payload[ps->index_payload] = byte; // ok!
                ps->index_payload++; // increment for the next time;
            }
            break;
    }
    return NO_MESSAGE;
}

// Function to initialize the circular buffer
void buffInit(CircBuff* buff) {
    buff->readIdx = 0;
    buff->writeIdx = 0;
}

// Function to write a char to the circular buffer
void buffWrite(CircBuff *buff, char data) {
    // If writeIdx - readIdx == 1, then thorugh away the oldest character
    if ((buff->readIdx % BUFF_SIZE) - (buff->writeIdx % BUFF_SIZE) <= 1 && buff->writeIdx < buff->readIdx)
        buff->readIdx = (buff->readIdx + 1) % BUFF_SIZE;

    buff->buff[buff->writeIdx] = data;
    buff->writeIdx = (buff->writeIdx + 1) % BUFF_SIZE;
}

// Function to read a char from the circular buffer
char buffRead(CircBuff *buff) {
    char data = buff->buff[buff->readIdx];
    buff->readIdx = (buff->readIdx + 1) % BUFF_SIZE;
    return data;
}

// Function to check if there are characters to flush on the LCD
int checkAvailableBytes(CircBuff* buff) {
    if(buff->readIdx <= buff->writeIdx)
        return buff->writeIdx - buff->readIdx;
    else
        return BUFF_SIZE - buff->readIdx + buff->writeIdx;
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
    double v = read_val * 3.3 / 1024.0;
    return 2.34 - 4.74*v + 4.06 * v*v - 1.60 * v*v*v + 0.24 * v*v*v*v;
}

double computeBattVolt(double read_val) {
    const double R1 = 200.0, R2 = 100.0;

    double v = read_val * 3.3 / 1024.0;
    return v * (R1 + R2) / R2;
}

void setPWM_Left(int pwm) {
    if (abs(pwm) < OC_MAX / 4) pwm = 3000;

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
    if (abs(pwm) < OC_MAX / 4) pwm = 3000;

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
    double surge = 0.0;
    
    if (dist < MINTH) surge = 0.0;
    else if (dist > MAXTH) surge = 0.7;
    else {
        surge = OC_MAX / 4 + (OC_MAX - OC_MAX / 4) * (dist - MINTH) / (MAXTH - MINTH);
        surge = surge / OC_MAX;
    }

    return surge;
}

double computeYaw(double dist) {
    double yaw_rate = 0.0;
    
    if (dist < MINTH) yaw_rate = 0.5;
    else if (dist > MAXTH) yaw_rate = 0.0;
    else {
        yaw_rate = OC_MAX * (1.0 / 4 + (1.0 - 1.0 / 4) * (dist - MINTH) / (MAXTH - MINTH));
        yaw_rate = yaw_rate / OC_MAX;
    }

    return yaw_rate;
}

void sendDistUART(double value) {
    char buff[16];

    if (countTx % 100 == 0) {
        value *= 100;
        sprintf(buff, "$MDIST,%d*\n", (int)value);
    }

    for (int i = 0; i < strlen(buff); i++)
        buffWrite(&circBuff, buff[i]);
}

void sendBattUART(double value) {
    char buff[16];

    if (countTx == 1000) {
        sprintf(buff, "$MBATT,%.2f*\n", value);
        countTx = 0;
    }

    for (int i = 0; i < strlen(buff); i++)
        buffWrite(&circBuff, buff[i]);
}

void sendDcsUART(double* values) {
    char buff[16];

    if (countTx % 100 == 0) {
        sprintf(buff, "$MPWM,%d,%d,%d,%d*", (int)values[0], (int)values[1], (int)values[2], (int)values[3]);
    }

    for (int i = 0; i < strlen(buff); i++)
        buffWrite(&circBuff, buff[i]);
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
    // IEC1bits.U2TXIE = 1;
    IEC1bits.U2RXIE = 1;
    U2STAbits.URXISEL = 3; // UART2 interrupt mode (1: every char received, 2: 3/4 char buffer, 3: full)

    // Init circular buffer
    buffInit(&circBuff);

    // Parser initialization
    parser_state pstate;
	pstate.state = STATE_DOLLAR;
	pstate.index_type = 0; 
	pstate.index_payload = 0;

    double v, dist, batt_val, adc_batt, adc_ir, surge, yaw_rate;
    // double dcs[4] = { 1.0, 2.0, 3.0, 4.0 };

    tmr_setup_period(TIMER1, 1);

    while(1) {
        while (!AD1CON1bits.DONE);
        adc_batt = ADC1BUF0;
        adc_ir = ADC1BUF1;

        dist = computeDist(adc_ir);
        batt_val = computeBattVolt(adc_batt);

        if (!start) {
            turnOffLights();
            stopMotors();
        }
        else {
            surge = computeSurge(dist);
            yaw_rate = computeYaw(dist);

            checkLights(surge, yaw_rate);

            setPWM_Left(OC_MAX * (surge+yaw_rate));
            setPWM_Right(OC_MAX * (surge-yaw_rate));
        }

        // sendDistUART(dist);
        // sendBattUART(batt_val);
        // sendDcsUART(dcs);
        // while (!U2STAbits.TRMT);
        //    U2TXREG = buffRead(&circBuff);
        
        blinkLed(1000);
        countTx++;

        tmr_wait_period(TIMER1);
    };

    return 0;
}
// Assignment 1 - Embedded Systems
// Gabriele Nicchiarelli - S4822677
// Veronica Gavagna - S5487110
// Andrea Bolla - S4482930

// DSPIC30F4011 Configuration Bit Settings

// FOSC
#pragma config FPR = XT            // Primary Oscillator Mode (XT)
#pragma config FOS = PRI           // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF// Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16   // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512  // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF       // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64  // POR Timer Value (64ms)
#pragma config BODENV = BORV20  // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON  // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI// Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI// High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN// PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN  // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF      // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF  // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD       // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <string.h>

#define TIMER1 1
#define TIMER2 2
#define FOSC 7372800
#define BAUND 9600
#define BUFF_SIZE 16
#define FIRST_ROW 0x80
#define SECOND_ROW 0xC0
#define CR '\r'
#define LF '\n'

char receivedChar;
int charCount = 0;
int rowCount = 0;
int btn_press = 0; // 0: S5, 1: S6

void tmr_setup_period(int timer) {
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

void tmr_setup_period_2(int timer, int ms) {
    switch(timer) {
        case TIMER1: {
            TMR1 = 0; // reset T1 counter

            long fcy = (FOSC / 4) * (ms / 1000.0);
            long fcy_new = fcy;

            if (fcy > 65535) {
                fcy_new = fcy / 8;
                T1CONbits.TCKPS = 1; // prescaler 1:8
            }
            if (fcy_new > 65535) {
                fcy_new = fcy / 64;
                T1CONbits.TCKPS = 2; // prescaler 1:64
            }
            if (fcy_new > 65535) {
                fcy_new = fcy / 256;
                T1CONbits.TCKPS = 3; // prescaler 1:256
            }

            PR1 = fcy_new;

            T1CONbits.TON = 1; // start T1
        }
        break;
        
        case TIMER2: {
            TMR2 = 0; // reset T2 counter

            long fcy = (FOSC / 4) * (ms / 1000.0);
            long fcy_new = fcy;

            if (fcy > 65535) {
                fcy_new = fcy / 8;
                T2CONbits.TCKPS = 1; // prescaler 1:8
            }
            if (fcy_new > 65535) {
                fcy_new = fcy / 64;
                T2CONbits.TCKPS = 2; // prescaler 1:64
            }
            if (fcy_new > 65535) {
                fcy_new = fcy / 256;
                T2CONbits.TCKPS = 3; // prescaler 1:256
            }

            PR2 = fcy_new;

            T2CONbits.TON = 1; // start T2
        }
        break;
    }
}

void tmr_wait_ms(int timer, int ms) {
    int prescaler = 1;
    long fcy = (FOSC / 4) * (ms / 1000.0);
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

void __attribute__ (( __interrupt__ , __auto_psv__ )) _INT0Interrupt() {
    IFS0bits.INT0IF = 0; // reset interrupt flag

    btn_press = 0;
    // start timer form 20ms
    tmr_setup_period_2(TIMER2, 20);
}

void __attribute__ (( __interrupt__ , __auto_psv__ )) _INT1Interrupt() {
    IFS1bits.INT1IF = 0; // reset interrupt flag

    btn_press = 1;
    // start timer form 20ms
    tmr_setup_period_2(TIMER2, 20);
}

void __attribute__ (( __interrupt__ , __auto_psv__ )) _T2Interrupt() {
    IFS0bits.T2IF = 0; // reset interrupt flag

    // when timer elapsed read if the btn is still pressed, if not toggle
    int pinValue = 0;

    if (!btn_press)
        pinValue = PORTEbits.RE8;
    else
        pinValue = PORTDbits.RD0;

    if (!pinValue)
        T2CONbits.TON = 0; // stop T2
    else {
        // stop T2
        T2CONbits.TON = 0;

        if (!btn_press) {
            // Send the current number of characters received via UART2
            char buff[BUFF_SIZE];
            sprintf(buff, "%d", charCount); // Convert charCount to a string
            // Send the buffer via UART2
            for (int i = 0; i < strlen(buff); i++) {
                while (!U2STAbits.TRMT); // Wait for UART2 transmit buffer to be empty
                U2TXREG = buff[i];
            }
        }
        else {
            LCD_ClearFirstRow();
            LCD_ClearSecondRow();
            charCount = 0;
            UpdateSecondRow();
        }

    }
}

// Function to initialize SPI1 for LCD
void SPI1_Init() {
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8-bit mode --> 1MHz
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 3; // 5:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI

    // Wait for LCD to go up
    tmr_wait_ms(TIMER1, 1000);
}

// Function to initialize UART2
void UART2_Init() {
    U2BRG = ((FOSC / 4) / (16L * BAUND)) - 1; // = 11
    U2MODEbits.UARTEN = 1; // enable UART2
    U2STAbits.UTXEN = 1; // enable U2TX (must be after UARTEN)
}

// Function to send data to the LCD via SPI1
void LCD_SendData(char data) {
    while(SPI1STATbits.SPITBF == 1); // wait until not full
    SPI1BUF = data;
}

// Function to clear the first row of the LCD
void LCD_ClearFirstRow() {
    LCD_SendData(FIRST_ROW);

    for (int i = 0; i < rowCount; i++)
        LCD_SendData(' ');

    LCD_SendData(FIRST_ROW);

    rowCount = 0;
}

// Function to clear the second row of the LCD
void LCD_ClearSecondRow() {
    LCD_SendData(SECOND_ROW);

    for (int i = 0; i < BUFF_SIZE; i++)
        LCD_SendData(' ');
}

// Function to update the second row of the LCD with the character count
void UpdateSecondRow() {
    LCD_SendData(SECOND_ROW);

    char buff[BUFF_SIZE];

    sprintf(buff, "Char Recv: %d", charCount);
    for (int i = 0; i < strlen(buff); i++) {
        LCD_SendData(buff[i]);
    }

    LCD_SendData(FIRST_ROW + rowCount);
}

char UART2_ReadChar() {
    while (!U2STAbits.URXDA); // Wait until data is received
    return U2RXREG; // Return the received data
}

void algorithm() {
    tmr_wait_ms(TIMER1, 7);
}

int main(void) {
    // Init timers
    tmr_setup_period(TIMER1);

    // Init UART2 and SPI1
    UART2_Init();
    SPI1_Init();

    // Init buttons S5 and S6 as inputs
    TRISEbits.TRISE8 = 1; // S5
    TRISDbits.TRISD0 = 1; // S6

    IEC0bits.INT0IE = 1; // enable INT0 interrupt
    IEC0bits.T2IE = 1; // enable T2 interrupt
    IEC1bits.INT1IE = 1; // enable INT1 interrupt

    while (1) {
        algorithm();

        // Check for received characters from UART2
        if (U2STAbits.URXDA) {
            receivedChar = U2RXREG;

            // Check for CR and LF characters and handle accordingly
            if (receivedChar == CR || receivedChar == LF || rowCount == 16)
                LCD_ClearFirstRow();
            else {
                LCD_SendData(FIRST_ROW + rowCount);
                // Display the received character on the first row of the LCD
                LCD_SendData(receivedChar);
                rowCount++;
                charCount++;
                // Update the second row with the character count
                UpdateSecondRow();
            }
        }

        tmr_wait_ms(TIMER1, 10);
    }

    return 0;
}

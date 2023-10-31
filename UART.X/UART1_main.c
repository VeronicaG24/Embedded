// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

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
#include <stdint.h>
#include <stdio.h>

#define TIMER1 1
#define TIMER2 2
#define FOSC 7372800
#define BAUND 9600
#define FIRST_ROW 0x80
#define SECOND_ROW 0xC0

char receivedChar;
int charCount = 0;

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

// Function to initialize SPI1 for LCD
void SPI1_Init() {
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8-bit mode --> 1MHz
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 3; // 5:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI

    // Wait for LCD to go up
    tmr_setup_period(TIMER1);
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
    charCount++;
}

// Function to clear the first row of the LCD
void LCD_ClearFirstRow() {
    LCD_SendData(FIRST_ROW);
    for (int i = 0; i < charCount; i++)
        LCD_SendData(' ');

    LCD_SendData(FIRST_ROW);
    
    charCount = 0;
}

// Function to update the second row of the LCD with the character count
void UpdateSecondRow() {
    LCD_SendData(SECOND_ROW);
    
    char buff[16];    
    int i = 0;
    sprintf(buff, "Char Recv: %d", charCount);

    while(buff[i] != '\0') {
        LCD_SendData(buff[i]);
        i++;
    }
}

char UART2_ReadChar() {
    while (!U2STAbits.URXDA); // Wait until data is received
    return U2RXREG; // Return the received data
}

int main(void) {
    // Init UART2 and SPI1
    UART2_Init();
    SPI1_Init();
    
    // Init buttons S5 and S6 as inputs
    TRISEbits.TRISE8 = 1; // S5
    TRISEbits.TRISE5 = 1; // S6

    while (1) {
        // Check for received characters from UART2
        if (U2STAbits.URXDA) {
            receivedChar = U2RXREG;
            
            LCD_SendData(FIRST_ROW);

            // Check for CR and LF characters and handle accordingly
            if (receivedChar == '\r' || receivedChar == '\n')
                LCD_ClearFirstRow();
            else {
                // Display the received character on the first row of the LCD
                LCD_SendData(receivedChar);

                if (charCount > 16)
                    LCD_ClearFirstRow();
            }

            // Update the second row with the character count
            UpdateSecondRow();
        }

        // Check for button S5 and S6 presses and handle as described
    }

    return 0;
}

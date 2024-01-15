// Assignment 1 - Embedded Systems
// Group 6
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
#include <stdlib.h>
#include <string.h>

#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
#define FOSC 7372800
#define BAUND 9600
#define LINE_SIZE 16
#define BUFF_SIZE 12
#define FIRST_ROW 0x80
#define SECOND_ROW 0xC0
#define CR '\r'
#define LF '\n'

/* How to compute the circular buffer size:
 * baud rate = 9600
 * read time = 10ms
 * bits transmitted = 1start + 1end + 8data = 10bits
 * (9600bps*0.001s)/10bits = 9.6 --> a little bit bigger: 12
 */

// Circular buffer structure
typedef struct {
    char buff[BUFF_SIZE];
    int readIdx;
    int writeIdx;
} CircBuff;

CircBuff circBuff;

int charCount = 0;     // Numeber of characters
int rowCount = 0;      // Row index
int btn_press = 0;     // Flag to select which btn was pressed (0: S5, 1: S6)
int char_recv_len = 0; // Length of the string "Char Recv: "

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
        
        case TIMER3: {
            TMR3 = 0; // reset T3 counter
            IFS0bits.T3IF = 0; // reset T3 flag
            T3CONbits.TON = 1; // start T3
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
        
        case TIMER3: {
            T3CONbits.TCKPS = prescaler;
            PR3 = fcy_new;

            // wait...
            while(!IFS0bits.T3IF);
            IFS0bits.T3IF = 0;
        }
        break;
    }
}

void tmr_setup_period(int timer, int ms) {
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
        
        case TIMER3: {
            TMR3 = 0; // reset T3 counter
            T1CONbits.TCKPS = prescaler;
            PR3 = fcy_new;
            T3CONbits.TON = 1; // start T3
        }
        break;
    }
}

void tmr_wait_period(int timer) {
    switch(timer) {
        case TIMER1: {
            while(!IFS0bits.T1IF);
            IFS0bits.T1IF = 0;
        } break;
        case TIMER2: {
            while(!IFS0bits.T2IF);
            IFS0bits.T2IF = 0;
        } break;
        case TIMER3: {
            while(!IFS0bits.T3IF);
            IFS0bits.T3IF = 0;
        } break;
    }
}

// Button S5 interrupt
void __attribute__ (( __interrupt__ , __auto_psv__ )) _INT0Interrupt() {
    IFS0bits.INT0IF = 0; // reset interrupt flag
    btn_press = 0;
    // start timer (20ms)
    tmr_setup_period(TIMER2, 20);
}

// Button S6 interrupt
void __attribute__ (( __interrupt__ , __auto_psv__ )) _INT1Interrupt() {
    IFS1bits.INT1IF = 0; // reset interrupt flag
    btn_press = 1;
    // start timer (20ms)
    tmr_setup_period(TIMER2, 20);
}

// Timer T2 interrupt
void __attribute__ (( __interrupt__ , __auto_psv__ )) _T2Interrupt() {
    IFS0bits.T2IF = 0; // reset interrupt flag

    // when timer elapsed read if the btn is still pressed, if not toggle
    int pinValue = 0;

    // Select the btn
    if (!btn_press)
        pinValue = PORTEbits.RE8;
    else
        pinValue = PORTDbits.RD0;

    T2CONbits.TON = 0; // stop T2
    
    if (pinValue) {
        if (!btn_press) {
            // If S5 button was pressed, send the current charCount over UART2
            UART2write(charCount);
        } else {
            // If S6 button was pressed, reset the display and charCount
            LCD_ClearFirstRow();
            LCD_ClearSecondRow();
            charCount = 0;
            UpdateSecondRow();
        }
    }
}

void UART2write(int charRecvNum) {
    char temp[12]; // Increase buffer size for larger numbers
    snprintf(temp, sizeof(temp), "%d", charRecvNum); // Use snprintf for safer string formatting

    for (int i = 0; i < strlen(temp); i++) {
        if (temp[i] != '\0') {
            while (!U2STAbits.TRMT); // Wait until the transmit buffer is empty
            U2TXREG = temp[i]; // Transmit the character
        }
    }
}

// UART2 interrupt
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt() {
    IFS1bits.U2RXIF = 0;  // Reset del flag di interrupt
    // Check for received characters from UART2
    while (U2STAbits.URXDA)
        Buff_Write(&circBuff, U2RXREG);
}

// Function to initialize the circular buffer
void Buff_Init(CircBuff* buff) {
    buff->readIdx = 0;
    buff->writeIdx = 0;
}

// Function to write a char to the circular buffer
void Buff_Write(CircBuff *buff, char data) {
    // If writeIdx - readIdx == 1, then thorugh away the oldest character
    if ((buff->readIdx % BUFF_SIZE) - (buff->writeIdx % BUFF_SIZE) <= 1 && buff->writeIdx < buff->readIdx)
        buff->readIdx = (buff->readIdx + 1) % BUFF_SIZE;

    buff->buff[buff->writeIdx] = data;
    buff->writeIdx = (buff->writeIdx + 1) % BUFF_SIZE;
}

// Function to read a char from the circular buffer
char Buff_Read(CircBuff *buff) {
    char data = buff->buff[buff->readIdx];
    buff->readIdx = (buff->readIdx + 1) % BUFF_SIZE;
    return data;
}

// Function to initialize SPI1 for LCD
void SPI1_Init() {
    SPI1CONbits.MSTEN = 1;  // master mode
    SPI1CONbits.MODE16 = 0; // 8-bit mode --> 1MHz
    SPI1CONbits.PPRE = 3;   // 1:1 primary prescaler
    SPI1CONbits.SPRE = 3;   // 5:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI

    // Wait for the LCD to go up
    tmr_wait_ms(TIMER3, 1000);
}

// Function to initialize UART2
void UART2_Init() {
    U2BRG = ((FOSC / 4) / (16L * BAUND)) - 1; // = 11
    U2MODEbits.UARTEN = 1; // enable UART2
    U2STAbits.UTXEN = 1; // enable U2TX (must be after UARTEN)
}

// Function to send data to the LCD
void LCD_WriteChar(char data) {
    while (SPI1STATbits.SPITBF == 1); // wait until buffer not full
    SPI1BUF = data;
}

// Function to clear the first row of the LCD
void LCD_ClearFirstRow() {
    LCD_WriteChar(FIRST_ROW);

    for (int i = 0; i < rowCount; i++)
        LCD_WriteChar(' ');

    LCD_WriteChar(FIRST_ROW);

    rowCount = 0;
}

// Function to clear the second row of the LCD
void LCD_ClearSecondRow() {
    LCD_WriteChar(SECOND_ROW + char_recv_len);

    for (int i = 0; i < LINE_SIZE - char_recv_len; i++)
        LCD_WriteChar(' ');
}

// Function to update the second row of the LCD with the character count
void UpdateSecondRow() {
    LCD_WriteChar(SECOND_ROW + char_recv_len);

    char buff[LINE_SIZE];

    sprintf(buff, "%d", charCount);
    for (int i = 0; i < strlen(buff); i++)
        LCD_WriteChar(buff[i]);

    LCD_WriteChar(FIRST_ROW + rowCount);
}

void SecondRow_Init() {
    LCD_WriteChar(SECOND_ROW);

    char buff[LINE_SIZE];

    sprintf(buff, "Char Recv: ");
    char_recv_len = strlen(buff);
    for (int i = 0; i < char_recv_len; i++)
        LCD_WriteChar(buff[i]);

    LCD_WriteChar(FIRST_ROW + rowCount);
}

// Algorithm function that runs for 7ms
void algorithm() {
    tmr_wait_ms(TIMER3, 7);
}

// Function to check if there are characters to flush on the LCD
int checkAvailableBytes(CircBuff* buff) {
    if(buff->readIdx <= buff->writeIdx)
        return buff->writeIdx - buff->readIdx;
    else
        return BUFF_SIZE - buff->readIdx + buff->writeIdx;
}

int main(void) {
    // Init timer 3
    tmr_setup_ms(TIMER3);

    // Init UART2 and SPI1
    UART2_Init();
    SPI1_Init();

    // Init buttons S5 and S6 as inputs
    TRISEbits.TRISE8 = 1; // S5
    TRISDbits.TRISD0 = 1; // S6

    // Enable interrupts
    IEC0bits.INT0IE = 1; // INT0
    IEC1bits.INT1IE = 1; // INT1
    IEC0bits.T2IE = 1;   // T2
    IEC1bits.U2RXIE = 1; // UART2
    U2STAbits.URXISEL = 3; // UART2 interrupt mode (1: every char received, 2: 3/4 char buffer, 3: full)
    
    // Init circular buffer
    Buff_Init(&circBuff);
    
    // Init second row with "Char Recv: "
    SecondRow_Init();

    // Init and setup timer 1
    tmr_setup_period(TIMER1, 10);

    // Main loop
    while (1) {
        algorithm();
        
        IEC1bits.U2RXIE = 0; // Disable UART2 interrupt

        while (U2STAbits.URXDA)
            Buff_Write(&circBuff, U2RXREG);
        IEC1bits.U2RXIE = 1; // Enable UART2 interrupt

        // If there are chars to read from the buffer
        while (checkAvailableBytes(&circBuff) > 0) {
            char readChar = Buff_Read(&circBuff); // Safely read a character from the buffer

            if (readChar == CR || readChar == LF) {
                LCD_ClearFirstRow();
                circBuff.readIdx = (circBuff.readIdx + 1) % BUFF_SIZE;
            }
            else if (rowCount == LINE_SIZE) {
                LCD_ClearFirstRow();
            }
            else {
                LCD_WriteChar(FIRST_ROW + rowCount);
                LCD_WriteChar(Buff_Read(&circBuff));

                rowCount++;
                charCount++;

                // Update the second row with the character count
                UpdateSecondRow();
            }
        }

        tmr_wait_period(TIMER1);
    }

    return 0;
}

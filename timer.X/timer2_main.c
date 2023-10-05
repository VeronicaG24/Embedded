/*
 * File:   timer1.c
 * Author: veronicagavagna
 *
 * Created on September 26, 2023, 12:59 PM
 */



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

#define TIMER1 1
#define FOSC 7372800

void tmr_wait_ms(int timer, int ms) {
    switch(timer) {
        case TIMER1: {
            TMR1 = 0; // reset timer counter

            // FOSC = 7.3728
            // Fcy = 7.3728 MHz / 4 = 1843200
            // Fcy * ms / 1000 = 184320 (over 65535, prescaler 1:8)
            // 184320 / 8 = 23040 clock steps

            long fcy = (FOSC / 4) * (ms / 1000.0);

            long fcy_new = 0.0;

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

            T1CONbits.TON = 1; //starts the timer
            
            if (IFS0bits.T1IF && TMR1 == 0) {
                LATBbits.LATB0 = 1;
                while(1);
            }
            
            while(!IFS0bits.T1IF);
            IFS0bits.T1IF = 0;
        }
        break;
    }
    
};

    
int main(void) {
    
    TRISBbits.TRISB0 = 0; // set the pin of the led as output
    
    // ES3
    LATBbits.LATB0 = 1;
    tmr_wait_ms(TIMER1, 1000);
    
    LATBbits.LATB0 = 0;
    tmr_wait_ms(TIMER1, 10000);
    
    LATBbits.LATB0 = 1;
    tmr_wait_ms(TIMER1, 500);
    
    LATBbits.LATB0 = 0; 
    tmr_wait_ms(TIMER1, 5000);
    
    return 0;
}

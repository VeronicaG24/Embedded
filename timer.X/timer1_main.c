/*
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
#define TIMER2 2
#define FOSC 7372800

void tmr_setup_period(int timer, int ms) {
    // FOSC = 7.3728 MHz
    // fcy = FOSC / 4 = 7.3728 / 4 = 1843200 Hz
    // fcy * ms / 1000 = 184320 (over 65535 --> prescaler 1:8)
    // PRX = 184320 / 8 = 23040 clock steps
    // if PRX still > 65535, then apply prescaler 1:64, eventually 1:256

    switch(timer) {
        case TIMER1: {
            TMR1 = 0; // reset T1 counter

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

            T1CONbits.TON = 1; // start T1
        }
        break;

        case TIMER2: {
            TMR2 = 0; // reset T2 counter

            long fcy = (FOSC / 4) * (ms / 1000.0);
            long fcy_new = 0.0;

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

void tmr_wait_period(int timer) {
    switch(timer) {
        case TIMER1: {
            while(!IFS0bits.T1IF);
            IFS0bits.T1IF = 0;
        }
        case TIMER2: {
            while(!IFS0bits.T2IF);
            IFS0bits.T2IF = 0;
        }
    }
}

int main(void) {
    TRISBbits.TRISB0 = 0; // led B0 as output

    // setup and start timer
    tmr_setup_period(TIMER1, 500);

    while(1) {
        LATBbits.LATB0 = !LATBbits.LATB0; // toggle led status
        // wait...
        tmr_wait_period(TIMER1);
    }

    return 0;
}
*/
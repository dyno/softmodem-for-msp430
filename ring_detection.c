//////////////////////////////////////////////////////////////////////////////////////////
//
// file        : ring_detection.c
// author      : Wolfgang Lutsch
//               Texas Instruments
// last update : 06/15/2004
// description : contains the routines to verify a valid ring from the DAA
//               Built with IAR Embedded Workbench Version: 2.21B
//
//////////////////////////////////////////////////////////////////////////////////////////
#ifdef __MSP430__
#include <legacymsp430.h>
#endif
#include  "msp430x14x.h"               // MSP430 specific header file
#include  "v_21.h"
#include  "ring_detection.h"

//////////////////////////////////////////////////////////////////////////////////////////
unsigned char ring_puls_cntr = 0;   // counter for LITELINK ring impulses
unsigned int  wdt_cntr = 0;         // counter to control Watchdog Interval Timer ISR
unsigned char ValidRing_cntr = 0;   // counter for valid ring signals

//////////////////////////////////////////////////////////////////////////////////////////
// S e t u p _ R i n g D e t e c t i o n
// function  : setup LITELINK DAA for ring detection state
//             setup MCU ring input pin to generate interrupts
//             reset all ring counters
void Setup_RingDetection(void)
{
// setup Port 2 outputs
    CTL_PORT_OUT |= OH + CID;        // set OH and CID logic high for on-hook state
// setup Port 2 inputs
    CTL_PORT_IES |= RING;            // high-low transition sensivity
    CTL_PORT_IE |= RING;             // P2.5 as input for LITELINK RING dectect signal, interrupt enable
    CTL_PORT_IFG &= ~RING;           // clear P2.5 interrupt flag
// setup variables
    ValidRing_cntr = 0;
    ring_puls_cntr = 0;
    wdt_cntr = 0;
    ModemStatusReg = 0;
}// end void Setup_RingDetection(void)

//////////////////////////////////////////////////////////////////////////////////////////
// I n i t _ R i n g D e t e c t i o n
// function  : for the most part doing the same as Setup_RingDetection above
//             ValidRing_cnt is not cleared
//             reset all the other ring counters
void Init_RingDetection(void)
{
    ModemStateMachine = CommandMode; // set modem state machine back to command mode
    LED_PORT_OUT |= STATUS_LED;      // LED off
    ModemStatusReg &= ~ValidRingFG;  // reset valid ring signal flag in status reg
    ring_puls_cntr = 0;              // clear
    wdt_cntr = 0;                    // counter
    CTL_PORT_IE |= RING;             // P2.5 as input for LITELINK RING detect signal, interrupt enable
    CTL_PORT_IFG &= ~RING;           // clear P2.5 interrupt flag
}// end void Init_RingDetection(void)

//////////////////////////////////////////////////////////////////////////////////////////
// S t a r t W D T _ I n t e r v a l T i m e r
// function  : start Watchdog Timer as Intervall Timmer
//             enable WDT interrupt generation
void StartWDT_IntervalTimer(void)
{
    wdt_cntr = 0;                    // clear counter
    WDTCTL = WDT_INIT;               // WDT 250ms, ACLK, interval timer
    IE1 |= WDTIE;                    // enable WDT interrupt
}// end void StartWDT_IntervalTimer(void)

//////////////////////////////////////////////////////////////////////////////////////////
// S t a r t W D T _ I n t e r v a l T i m e r
// function  : stop Watchdog Timer
//             disable WDT interrupt generation
void StopWDT(void)
{
    WDTCTL = WDTPW + WDTHOLD;        // stop WDT
    IE1 &= ~WDTIE;                   // disable WDT interrupt
}// end void StopWDT(void)

//////////////////////////////////////////////////////////////////////////////////////////
// g o _ o f f _ h o o k
// function  : accept an incoming call
void go_off_hook(void)
{
    CTL_PORT_OUT &= ~OH;             // assert LITELINK OH input and go off-hook
}// end void go_off_hook(void)

//////////////////////////////////////////////////////////////////////////////////////////
// g o _ o n _ h o o k
// function  : terminate the communication
void go_on_hook(void)
{
    CTL_PORT_OUT |= OH;              // de-assert LITELINK OH input and go on-hook
    LED_PORT_OUT |= STATUS_LED;      // LED off
    LED_PORT_OUT |= DATA_LED;        // LED off
    Setup_RingDetection();
}// end void go_on_hook(void)

//////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINE : P 1 _ I S R
// function  : count ring impulses
#ifdef __ICC430__
#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR(void)
#endif
#ifdef __MSP430__
interrupt(PORT2_VECTOR) P2_ISR(void)
#endif
{
    if (P2IFG & RING) {
        ring_puls_cntr++;

        if (ring_puls_cntr == 1) {     // first ring pulse detected
            LED_PORT_OUT |= STATUS_LED; // LED off
            ModemStateMachine = RingDetection; // change modem state
            StartWDT_IntervalTimer();
        }

        if (ring_puls_cntr == 15) {
            CTL_PORT_IE &= ~RING;       // disable interrupt
            ModemStatusReg |= ValidRingFG; // indicate valid ring signal in status reg
        }

        CTL_PORT_IFG &= ~ RING;        // clear interrupt flag
    } else {
        CTL_PORT_IFG = 0;
    }
}// end P2_ISR

//////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINE : w a t c h d o g _ t i m e r
// function  : RingDetection, verify ring impulses as valid ring signal
//             by providing the needed time base for the detection algorithm
#ifdef __ICC430__
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
#endif
#ifdef __MSP430__
interrupt(WDT_VECTOR) watchdog_timer(void)
#endif
{
    wdt_cntr++;

    switch (ModemStateMachine) {
    case RingDetection:

        switch (wdt_cntr) {
        case _250ms:             // 250ms expired

            if (!(ring_puls_cntr > 1)) {
                Init_RingDetection();
                StopWDT();
            }

            break;
        case _750ms:             // 750ms expired

            if (ModemStatusReg & ValidRingFG) {
                LED_PORT_OUT &= ~STATUS_LED;    // LED on
                ModemStatusReg &= ~ValidRingFG; // reset valid ring signal flag in status reg
            } else {
                Init_RingDetection();
                StopWDT();
            }

            break;
        case _2000ms:                         // 2000ms expired
            ValidRing_cntr++;

            if (ValidRing_cntr == MAX_RINGS) {
                LED_PORT_OUT |= STATUS_LED;     // LED off
                go_off_hook();
                // changes
                ModemStatusReg |= STABILIZE_DCO;
            } else {
                Init_RingDetection();
            }

            break;
        }

        if (ValidRing_cntr < MAX_RINGS && wdt_cntr == _10000ms) { // 10 seconds passed
            ValidRing_cntr = 0;                                   // reset valid ring counter
            StopWDT();                                            // stop watchdog timer
        } else {                                                  // otherwise we're off-hook
            switch (wdt_cntr) {
            case _2250ms:                      // 2250ms
                LED_PORT_OUT &= ~STATUS_LED;
                break;
            case _2500ms:                      // 2500ms
                LED_PORT_OUT |= STATUS_LED;
                break;
            case _2750ms:                      // 2750ms
                LED_PORT_OUT &= ~STATUS_LED;
                break;
            case _3000ms:                      // 3000ms
                LED_PORT_OUT |= STATUS_LED;
                break;
            case _3250ms:                      // 3250ms
                LED_PORT_OUT &= ~STATUS_LED;
                break;
            case _3500ms:                      // 3500ms
                LED_PORT_OUT |= STATUS_LED;
                break;
            case _3750ms:                      // 3750ms
                LED_PORT_OUT &= ~STATUS_LED;
                break;
            case _4000ms:                      // 4000ms
                LED_PORT_OUT |= STATUS_LED;
                StopWDT();                      // stop WDT
                ModemStatusReg |= OFF_HOOK;     // show main programm
                break;                          // that we're off-hook
            }
        }

        break;
    }
}// end watchdog_timer


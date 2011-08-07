//////////////////////////////////////////////////////////////////////////////////////////
//
// file        : softmodem.c
// author      : Wolfgang Lutsch
//               Texas Instruments
// last update : 06/15/2004
// description : demonstrates V.21 modem communication
//               Built with IAR Embedded Workbench Version: 2.21B
//
//////////////////////////////////////////////////////////////////////////////////////////

#ifdef __MSP430__
#include <legacymsp430.h>
#endif
#include  "msp430x14x.h"               // MSP430 specific header file
#include  "softmodem.h"                // header for Soft-Modem Demo
#include  "v_21.h"                     // header for V.21 Datapump

unsigned char uSTATUS = 0;             // application status register
char RcvdChar;                         // global variable to store a received byte
unsigned int ADCresult;                // global variable to store an ADC conversion result
char Temperature[4];
static const char HexTable[16] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8',
    '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

//-----------------------------------------------------------------------------------
// main programm to demonstrate the usage of the V.21 software module
#ifdef __ICC430__
void main(void)
#endif
#ifdef __MSP430__
int main(void)
#endif
{
    HardwareInit();                      // setup basic clock system
    ModemInit();                         // setup modem functions
    EnableADC12();                       // enable ADC12 for single conversion

    while (1) {
        modem();                           // handle modem state machine
        TriggerADC12();                    // do a single AD conversion of internal temperature diode

        if (ModemStateMachine == DataMode) { // do this when a connection is established
            if (!(uSTATUS & WelcomeMessage)) { // do this once right after
                // a successful connection establishment
                prtstr("\033[2JMSP430 Soft-Modem Demo\n\r");  // send clear screen sequence and a welcome message
                uSTATUS |= WelcomeMessage;     // set status bit to signal that the message has been sent
            }

            ProcessRcvdChr();                // evaluate and process a received character
        } else {
            uSTATUS &= ~(WelcomeMessage + CHAR_RCVD); // reset status bits when no connection is available
        }
    }

#ifdef __MSP430__
    return 0;
#endif
}

//-----------------------------------------------------------------------------------
// R c v d C h r H a n d l e r
// function: moves a received char from the UART receive buffer to the global variable
//           RcvdChar. This function is called by the UART interrupt routine
//           within the V.21 module sourcefile
void RcvdChrHandler(unsigned char chr)
{
    if (!(uSTATUS & CHAR_RCVD)) {        // do this if last received character has been
        // already processesed
        RcvdChar = chr;                    // copy received char to global variable
        uSTATUS |= CHAR_RCVD;              // set status bit to signal char reception
    }
}

//-----------------------------------------------------------------------------------
// P r o c e s s R c v d C h r
// function: handles a simple user protocol
//           processes a received char stored in the global variable RcvdChar
//           and echoes corresponding message strings
void ProcessRcvdChr(void)
{
    if (uSTATUS & CHAR_RCVD) {           // do this if character was received
        putchr(RcvdChar);                  // echo received char

        switch (RcvdChar) {                // switch hotkeys
        case '0':
            P1OUT ^= BIT0;                 // toggle output

            if (P1OUT & BIT0) {            // test output level
                prtstr("->LED off\n\r");    // echo message string: LED off
            } else {
                prtstr("->LED on\n\r");    // echo message string: LED off
            }

            break;
        case '6':

            if (P6IN & BIT6) {             // get input level on P6.6
                prtstr("->P6.6 Input: HIGH\n\r");
            } else {                       // and send string message
                prtstr("->P6.6 Input: LOW\n\r");
            }

            break;
        case '7':

            if (P6IN & BIT7) {             // get input level on P6.7
                prtstr("->P6.7 Input: HIGH\n\r");
            } else {                       // and send string message
                prtstr("->P6.7 Input: LOW\n\r");
            }

            break;
        case 't':
            prtstr("->MCU Temperature: ");
            prtstr("0x0");
            prtstr(Temperature);            // send current ADC value
            prtstr("\n\r");
            break;
        case 'x':
            prtstr("->Good Bye!");         // send Good Bye string
            uSTATUS = 0;                   // reset all user status bits
            delay(500);                    // delay
            Disconnect();                  // terminate the connection
            break;                         // and initialize Soft-Modem
        default:
            prtstr("\n\r");
        }

        uSTATUS &= ~CHAR_RCVD;   // reset status bit to signal rcvd char has been processed
    }
}

//-----------------------------------------------------------------------------------
// p r t s t r
// function: outputs a string by using function putchr
void prtstr(char* txt)
{
    while (*txt != 0) {                 // see if string ends
        putchr(*txt++);			// and print
    }
}

//-----------------------------------------------------------------------------------
// ADC12 routines
// E n a b l e A D C 1 2
// function: initilizes the ADC12 module
void EnableADC12(void)
{
    volatile unsigned int i;
    ADC12CTL0 = ADC12ON + SHT0_7 + REFON;     // Turn on ADC12, set sampling time

    for (i = 0; i < 0x97D; i++);              // Delay for reference start-up

    ADC12CTL1 = CSTARTADD_0 + SHP;            // use ADC12MEM0, use sampling timer
    ADC12MCTL0 = INCH_10 + SREF_1;            // select A10, Vref=1.5V
    ADC12IE = 0x01;                           // Enable ADC12IFG.0
}

//-----------------------------------------------------------------------------------
// T r i g g e r A D C 1 2
// function: triggers a sinlge AD conversion on channel 10
//           and scales the converted sample value to degrees Celsius
//           result is stored in global variable Temperature
void TriggerADC12(void)
{
    if (uSTATUS & ADC_INIT) {                 // is ADC initialized to perform a conversion?
        if (uSTATUS & ADC_vAVIAL) {           // is a conversion result available?
            // format ADC result for transmission
            // value will be tranmitted as ASCII coded hex
            int2Hex((unsigned long)ADCresult, 3, Temperature);
            uSTATUS &= ~ADC_vAVIAL;               // reset status flags
            uSTATUS &= ~ADC_INIT;
        }
    } else {
        ADC12CTL0 |= ENC;                       // enable ADC conversion
        ADC12CTL0 |= ADC12SC;                   // start conversion
        uSTATUS |= ADC_INIT;                    // set status bit that ADC is initialized for conversion
    }
}

//-----------------------------------------------------------------------------------
#ifdef __ICC430__
#pragma vector=ADC_VECTOR
__interrupt void ADC12ISR(void)
#endif
#ifdef __MSP430__
interrupt(ADC_VECTOR) ADC12ISR(void)
#endif
{
    ADCresult = (ADC12MEM0 - 2692);           // calculation of temperature(1/10 deg C)
    uSTATUS |= ADC_vAVIAL;                    // set status bit that ADC result is available
    ADC12CTL0 &= ~ENC;                        // disable conversion
}

//-----------------------------------------------------------------------------------
void int2Hex(unsigned long v, unsigned char Len, char* ps)
{
    *(ps + Len) = '\0';

    while (Len--) {
        *(ps + Len) = HexTable[v & 0x0f];
        v >>= 4;
    }
}
//-----------------------------------------------------------------------------------


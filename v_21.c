//////////////////////////////////////////////////////////////////////////////////////////
//
// file        : v_21.c
// author      : Wolfgang Lutsch
//               Texas Instruments
// last update : 06/15/2004
// description : implements the V.21 data pump
//               Built with IAR Embedded Workbench Version: 2.21B
//
//////////////////////////////////////////////////////////////////////////////////////////

#include  "msp430x14x.h"               // MSP430 specific header file
#include  "v_21.h"                     // header for V.21 Datapump
#include  "ring_detection.h"           // header for ring detection algorithms

unsigned char ModemStateMachine;
unsigned char HandshakeStateMachine;
unsigned char ModemStatusReg = POSITIVE_EDGE;
unsigned char BitCounter = 0;
unsigned char ones = 0;
unsigned char zeros = 0;
unsigned int  BitFreq;

//////////////////////////////////////////////////////////////////////////////////////////
// m o d e m
// function  : implements the modem state machine
//             must be called by user programm periodically
//             handles on-hook and off-hook communication states
// arguments : none
// returns   : none
// globals   : ModemStateMachine, ModemStatusReg, HandshakeStateMachine
void modem(void) {
  static unsigned int Off_Hook_TimeOut_cntr;

   switch(ModemStateMachine) {
      case RingDetection:
         if(ModemStatusReg & STABILIZE_DCO) {
            Set_DCO();
            ModemStatusReg &= ~STABILIZE_DCO;
         }
         if(ModemStatusReg & OFF_HOOK) {                    // off-hook ?
            HandshakeStateMachine = Silence150ms;           // initialize
            ModemStateMachine = AnsweringHandshake;         // answer handshake algorithm
            SetupFSKModem();                                // setup datapump
	    }
         break;
      case DataMode:
         if(ModemStatusReg & OFF_HOOK) {                    // routine to check
            ModemStatusReg &= ~OFF_HOOK;                    // if carrier signal is still available
            Off_Hook_TimeOut_cntr = 0;
         } else {
            Off_Hook_TimeOut_cntr++;
            if(Off_Hook_TimeOut_cntr >= OFF_HOOK_TIMEOUT) {
               Disconnect();                                // carrier signal is no longer available
                                                            // terminate connection
             }
         }
         break;
   }
}// end void modem(void)

//////////////////////////////////////////////////////////////////////////////////////////
// H a r d w a r e I n i t
// function  : setup basic clock system
//             stop watchdog timer
//             enable global interrupt
// arguments : none
// returns   : none
void HardwareInit(void) {
   volatile unsigned int i;              // volatile to prevent optimization

   WDTCTL = WDTPW + WDTHOLD;             // stop WDT
   for (i = 0xFFFF; i > 0; i--);         // Delay for LFXT1 to settle
   Set_DCO();                            // set DCO frequency
   _EINT();                              // Enable interrupts
}// end void HardwareInit(void)

//////////////////////////////////////////////////////////////////////////////////////////
// S e t u p L E D s
// function  : setup LED port pins
// arguments : none
void SetupLEDs(void) {   
   LED_PORT_OUT &= ~(DATA_LED+STATUS_LED);      // LEDs on
   LED_PORT_DIR |= DATA_LED+STATUS_LED;         // set pins to output direction
   delay(50);
   LED_PORT_OUT |= DATA_LED+STATUS_LED;         // LEDs off
   delay(50);
   LED_PORT_OUT &= ~(DATA_LED+STATUS_LED);      // LEDs on
   delay(50);
   LED_PORT_OUT |= DATA_LED+STATUS_LED;         // LEDs off
   delay(50);
   LED_PORT_OUT &= ~(DATA_LED+STATUS_LED);      // LEDs on
   delay(50);
   LED_PORT_OUT |= DATA_LED+STATUS_LED;         // LEDs off
}// end void SetupLEDs(unsigned char* seq)

//////////////////////////////////////////////////////////////////////////////////////////
// M o d e m I n i t
// function  : setup modem state machine,
//             LITELINK and ring detection routines
void ModemInit(void) {
   Setup_LITELINK();
   SetupLEDs();
   Setup_RingDetection();
   ModemStateMachine = CommandMode;
}// end void ModemInit(void)

//////////////////////////////////////////////////////////////////////////////////////////
// S e t u p _ L I T E L I N K
// function  : setup control pins connected to LITELINK DAA
void Setup_LITELINK(void) {
   CTL_PORT_OUT |= OH + CID;                    // de-assert LITELINK inputs
   CTL_PORT_DIR |= OH + CID;                    // P2.1 as output to LITELINK OH
   delay(50);                                   // LITLINK needs 50ms to setup
}// end void Setup_LITELINK(void)

//////////////////////////////////////////////////////////////////////////////////////////
// S e t u p F S K M o d e m
// function  : setup hardware modules for modulation
//             and demodulation
void SetupFSKModem(void) {
   TACTL = TIMER_A_INIT;                        // clock source, clear TAR
// Initialize_Demodulator
   TACCTL1 = CM_3+CCIS_1+SCS+CAP;               // Capture mode: 3 - both edges
                                                // Capture input select: 1 - CCIxB from Comp.A
                                                // Capture sychronize
                                                // Capture Mode
// Setup_ComparatorA
   CACTL1 = CARSEL+CAREF_1;                     // Comp. A Int. Ref. enable on CA1	
									            // Comp. A Int. Ref. Select 2 : 0.5*Vcc
   CACTL2 = P2CA0+CAF;                          // Comp. A Connect External Signal to CA0
					                            // Comp. A Enable Output Filter
// Setup_UART0
   UCTL0 = CHAR;                                // 8bit character
   UTCTL0 = SSEL1;                              // UCLK = SMCLK
   UBR00 = (unsigned char)BAUDRATE;             // 300bps @ 32768 Hz
   UBR10 = (unsigned char)(BAUDRATE >> 8);
   UMCTL1 = 0;                                  // no modulation
   ME1 |= UTXE0+URXE0;                          // enable RX, TX
   IE1 |= URXIE0;                               // enable RX interrupt
// setup communication port pins
   UART_COM_PORT_OUT |= DEMODULATOR_DIGITAL_OUT;// set input level HIGH for URXD1
   UART_COM_PORT_DIR |= DEMODULATOR_DIGITAL_OUT;// P1.2 output
   UART_COM_PORT_IE |= MODULATOR_DIGITAL_IN;    // P1.3 interrupt enable
   UART_COM_PORT_IES |= MODULATOR_DIGITAL_IN;   // interrupt flag is set with a high-to-low transition
   MODEM_PORT1_SEL |= MODULATOR_OUT;            // select TA2 instead of P1.7
   MODEM_PORT1_DIR |= MODULATOR_OUT;            // TA2 as output
// setup Port 2
   MODEM_PORT2_SEL |= DEMODULATOR_IN;           // select CA0 instead of P2.3
// setup ports for UART0
   UART_PORT_SEL |= UART_RxD+UART_TxD;          // select P3.4, P3.5 for USART0 function
   UART_PORT_DIR |= UART_TxD;                   // P3.4 as output UTXD0
// Initialize_Negotation_Handshake_Trigger
   TACCTL0 = CCIE;                              // CCR0 interrupt enable
   TACCR0 = BAUDRATE; 
// Start_TimerA_Enable_Comp_A
   TACTL |= MC_2;                               // Timer A mode control: 2 - Continous up
   CACTL1 |= CAON;                              // Comp. A enable
}// end void SetupFSKModem(void)

//////////////////////////////////////////////////////////////////////////////////////////
// S t a r t A n s w e r S e q u e n c e
// function  : start answer sequence tone generation
// globals   : BitFreq
void StartAnswerSequence(void) {
   BitFreq = _2100HzAnsFreq;                     // store answer sequence frequency value
   TACCR2 = BitFreq;                             // move answer sequence frequency value to TACCR2
   TACCTL2 = OUTMOD_4+CCIE;                      // CCR2 toggle mode, interrupt enable
}// end void StartAnswerSequence(void)

//////////////////////////////////////////////////////////////////////////////////////////
// S t o p A n s w e r S e q u e n c e
// function  : stop answer sequence tone generation
void StopAnswerSequence(void) {
   TACCTL2 = 0;
}// end void StopAnswerSequence(void)

//////////////////////////////////////////////////////////////////////////////////////////
// S t a r t F S K M o d u l a t o r
// function  : start FSK modulator
// globals   : BitFreq
void StartFSKModulator(void) {
   BitFreq = _1650HzBitFreq;                     // save logic 1 frequency value 
   TACCR2 = BitFreq;                             // move logic 1 frequency value to TACCR2
   TACCTL2 = OUTMOD_4+CCIE;                      // CCR2 toggle mode, interrupt enable
}// end void StartFSKModulator(void)

//////////////////////////////////////////////////////////////////////////////////////////
// S t a r t F S K D e m o d u l a t o r
// function  : start FSK Demodulator
// globals   : BitFreq
void StartFSKDemodulator(void) {
   TACCTL1 |= CCIE;                              // CCR1 Capture interrupt enable
}// end void StartFSKDemodulator(void)

//////////////////////////////////////////////////////////////////////////////////////////
// S t o p F S K M o d e m
// function  : disable modulator/demodulator hardware modules
void StopFSKModem(void) {
   TACTL = 0;                                    // stop Timer A
   CACTL1 = 0;                                   // shut down Comparator A
   TACCTL1 = 0;                                  // shut down demodulator
   TACCTL2 = 0;                                  // shut down modulator
}// end StopFSKModem(void)
//////////////////////////////////////////////////////////////////////////////////////////
// D i s c o n n e c t
// function  : terminates an established connection
void Disconnect(void) {
  StopFSKModem();                              // stop modem
  go_on_hook();                                // change communication state
  ModemInit();                                 // init softmodem
}
//////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINE : P 1 _ I S R
// function  : generated by UART tx signal
//             init frequency generator routines with new
//             bit frequency value
#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR(void) {
   if(UART_COM_PORT_IFG & MODULATOR_DIGITAL_IN) {   // isr caused by MODULATOR_DIGITAL_IN ?
      if(UART_COM_PORT_IES & MODULATOR_DIGITAL_IN) {// caused by high-low transmission ?
         BitFreq = _1850HzBitFreq;                  // load logical 0 value into modulation register
         LED_PORT_OUT &= ~DATA_LED;                 // LED on
         UART_COM_PORT_IES &= ~MODULATOR_DIGITAL_IN;// change pin isr edge sensivity
      } else {
         BitFreq = _1650HzBitFreq;                  // load logical 1 value into modulation register
         LED_PORT_OUT |= DATA_LED;                  // LED off
         UART_COM_PORT_IES |= MODULATOR_DIGITAL_IN; // change pin isr edge sensivity
      }
      UART_COM_PORT_IFG &= ~MODULATOR_DIGITAL_IN;   // clear interrupt flag
   } else {
      UART_COM_PORT_IFG = 0;
   }
}// end P1_ISR

//////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINE : T A _ C C R 0 _ I S R
// function  : handle handshake algorithm during answer-handshake state
//             generate time discrete signal of demodulated 0s and 1s
//             to be recognized by the UART rx pin during datamode state
//             if enabled, isr is generated every 1/300s (=baudrate)
#pragma vector=TIMERA0_VECTOR
__interrupt void TA_CCR0_ISR(void) {

  static unsigned int HandshakeCounter = 0;

   // DataMode
   if((ModemStatusReg & STARTBIT) && ModemStateMachine == DataMode) {
      BitCounter++;
      if(BitCounter == 9) {                               // processed one byte ?
         TACCTL0 = 0;                                     // stop bit detection trigger
         ModemStatusReg &= ~STARTBIT;
         UART_COM_PORT_OUT |= DEMODULATOR_DIGITAL_OUT;    // show stopbit to UART
         LED_PORT_OUT |= DATA_LED;                        // LED off
      } else {
         if(ones >= zeros) {
            UART_COM_PORT_OUT |= DEMODULATOR_DIGITAL_OUT; // show 1 to UART
            LED_PORT_OUT |= DATA_LED;                     // LED off
         } else {
            UART_COM_PORT_OUT &= ~DEMODULATOR_DIGITAL_OUT;// show 0 to UART
            LED_PORT_OUT &= ~DATA_LED;                    // LED on
         }
      }
      zeros = 0;
      ones = 0;
   }

   // Handshake
   if(ModemStateMachine == AnsweringHandshake) {
       switch(HandshakeStateMachine) {
          case Silence150ms:
             HandshakeCounter++;
             if(HandshakeCounter >= NH_SILENCE) {
                HandshakeCounter = 0;
                StartAnswerSequence();
                HandshakeStateMachine = Answer3300ms;
             }
             break;
          case Answer3300ms:
             HandshakeCounter++;
             if(HandshakeCounter >= NH_ANSWER) {
                HandshakeCounter = 0;
                StopAnswerSequence();
                HandshakeStateMachine = Wait75ms;
             }
             break;
          case Wait75ms:
             HandshakeCounter++;
             if(HandshakeCounter >= NH_WAIT) {
                HandshakeCounter = 0;
                StartFSKModulator();
                StartFSKDemodulator();
                HandshakeStateMachine = DetectOnes;
             }
             break;
          case OnesDetected:
             HandshakeCounter++;
             if(HandshakeCounter >= NH_ONES) {
                HandshakeCounter = 0;
                ModemStateMachine = DataMode;
                StartWDT_IntervalTimer();
                zeros = 0;
                ones = 0;
                LED_PORT_OUT &= ~STATUS_LED;        // LED on
                TACCTL0 = 0;                        // disable handshake trigger
             }
             break;
       }
   }
   TACCR0 += BAUDRATE;
}// end TA_CCR0_ISR

//////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINE : T i m e r _ A 1
// function  : measures time between two corresponding trigger edges
//             and decides if measured value matches
//             the space or mark frequency value
//             startbit detection
// globals   : zeros, ones
#pragma vector=TIMERA1_VECTOR
__interrupt void Timer_A1(void) {

static signed int Prev_negEgdeCapture = 0;
static signed int Prev_posEgdeCapture = 0;
static unsigned char EdgeCounter = 0;
signed int CCR1_Capture;
signed int Capture_Diff;

   switch(TAIV) {
   // CCR1 interrupt handler
      case  2:
         CCR1_Capture = TACCR1;
         if(ModemStatusReg & POSITIVE_EDGE) {
            Capture_Diff = CCR1_Capture - Prev_posEgdeCapture;
            Prev_posEgdeCapture = CCR1_Capture;
            ModemStatusReg &= ~POSITIVE_EDGE;
         } else {
            Capture_Diff = CCR1_Capture - Prev_negEgdeCapture;
            Prev_negEgdeCapture = CCR1_Capture;
            ModemStatusReg |= POSITIVE_EDGE;
         }
         
         if(Capture_Diff > RX_FREQ_CHN1_SPACE-CHN1_MARGIN && Capture_Diff < RX_FREQ_CHN1_SPACE+CHN1_MARGIN) {
            if(HandshakeStateMachine == DetectOnes)
               EdgeCounter = 0;
            // 0 detected
            zeros++;
            ModemStatusReg |= OFF_HOOK;
         } else if(Capture_Diff > RX_FREQ_CHN1_MARK-CHN1_MARGIN && Capture_Diff < RX_FREQ_CHN1_MARK+CHN1_MARGIN) {
            // handle detection of binary ones during negotiation handshake
            if(HandshakeStateMachine == DetectOnes) {    // are we performing the negotiation handshake?
               EdgeCounter++;                            // increment counter
               if(EdgeCounter == DetectTime_Chn1) {      // sufficient edges detected?
                  HandshakeStateMachine = OnesDetected;  // change to next negotiation handshake state
               }
            }
            // 1 detected
            ones++;
            ModemStatusReg |= OFF_HOOK;
         }
         
         if(zeros>=6 && ModemStateMachine == DataMode && !(ModemStatusReg & STARTBIT)) {
            ModemStatusReg |= STARTBIT;       // startbit detected
            BitCounter = 0;
            zeros = 0;                        // reset counters
            ones = 0;
            UART_COM_PORT_OUT &= ~DEMODULATOR_DIGITAL_OUT;  // show startbit to UART
            LED_PORT_OUT &= ~DATA_LED;
            TACCTL0 = CCIE;
            TACCR0 = CCR1_Capture + BAUDRATE;
         }

         break;
   // CCR2 interrupt handler
      case  4: 
         TACCR2 += BitFreq;                   // Add Offset to CCR2
         break;
   // TA overflow interrupt handler
      case 10: _NOP();
         break;
   }
}// end Timer_A1

//////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINE : u s a r t 0 _ r x
#pragma vector=UART0RX_VECTOR
__interrupt void usart0_rx (void)
{
  RcvdChrHandler(U0RXBUF);                   // handle received char
//  putchr(U0RXBUF);                         // performs a simple echo of the recieved char
}// end usart0_rx

//////////////////////////////////////////////////////////////////////////////////////////
// d e l a y (resolution is 1ms)
// function  : Delay function 
//             User knows target frequency, instruction cycles, C implementation.
// arguments : word millisec (number of ms, max number is 0xFFFF) 	
void delay(unsigned int msec) {
  unsigned int i;

  for (i = msec; i > 0; i--)
    delay_cycles(D_LOOPCOUNT1ms);                   
}// end void delay(unsigned int msec)

//////////////////////////////////////////////////////////////////////////////////////////
// d e l a y _ c y c l e s (resolution is 3 MCLK cycles)
// function  : Delay function, works C optimizer independent
// arguments : Program execution is delayed by (Num * 3) MCLK cycles
void delay_cycles(unsigned int Num)
{
  asm("   dec.w   R12");                    // decrement 'Num'
  asm("   jnz     $-2");                    // jump back if not zero
}

//////////////////////////////////////////////////////////////////////////////////////////
// p u t c h r
// function  : writes one byte into the UART tx buffer register
// arguments : c, the byte to be sent
void putchr(unsigned char c) {
   while ((IFG1 & UTXIFG0)==0);               // wait until register is released
   TXBUF0 = c;                                // write byte into register
}// end void putchr(unsigned char c)

//////////////////////////////////////////////////////////////////////////////////////////
// S e t _ D C O
// function  : Subroutine to set DCO to selected frequency
//             DCO frequency will be a multiple of 4096 Hz
// arguments : desired DCO frequency
// returns   : frequency to which the DCO was set by the routine
//             (a multiple of 4096 Hz)
//long Set_DCO (long tmpDCO_CLK)
void Set_DCO (void)
{
  unsigned int Compare, Oldcapture = 0;
  
  BCSCTL1 |= DIVA1 + DIVA0;             // ACLK = LFXT1CLK/8
  
  CCTL2 = CCIS0 + CM0 + CAP;            // Define CCR2, CAP, ACLK
  TACTL = TASSEL1 + TACLR + MC1;        // SMCLK, continous mode
  while (1)
  {
    while ((CCTL2 & CCIFG) != CCIFG);   // Wait until capture occured!
    CCTL2 &= ~CCIFG;                    // Capture occured, clear flag
    Compare = CCR2;                     // Get current captured SMCLK
    Compare = Compare - Oldcapture;     // SMCLK difference
    Oldcapture = CCR2;                  // Save current captured SMCLK
    if (DELTA == Compare)
    {
      break;                            // if equal, leave "while(1)"
    }
    else if (DELTA < Compare)           // DCO is too fast, slow it down
    {
      DCOCTL--;
      if (DCOCTL == 0xFF)               // Did DCO role under?
      {
        BCSCTL1--;                      // Select next lower RSEL
      }
    }
    else
    {                      
      DCOCTL++;
      if (DCOCTL == 0x00)               // Did DCO role over?
      {
        BCSCTL1++;                      // Select next higher RSEL
      }
    }
  }  
  CCTL2 = 0;                              // Stop CCR2 function
  TACTL = 0;                              // Stop Timer_A
  BCSCTL1 &= ~(DIVA1 + DIVA0);            // undivided ACLK
}

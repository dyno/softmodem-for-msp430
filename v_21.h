//////////////////////////////////////////////////////////////////////////////////////////
//
// file        : v_21.h
// author      : Wolfgang Lutsch
//               Texas Instruments
// last update : 06/15/2004
// description : header-file for v_21.c
//               Built with IAR Embedded Workbench Version: 2.21B
//
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef   _v_21_H
#define   _v_21_H

#define   MODULATOR_OUT            BIT7
#define   MODULATOR_DIGITAL_IN     BIT3
#define   DEMODULATOR_IN           BIT3
#define   DEMODULATOR_DIGITAL_OUT  BIT2

#define   MODEM_PORT1_DIR          P1DIR
#define   MODEM_PORT1_SEL          P1SEL

#define   MODEM_PORT2_SEL          P2SEL

#define   UART_RxD                 BIT5
#define   UART_TxD                 BIT4

#define   UART_PORT_DIR            P3DIR
#define   UART_PORT_SEL            P3SEL
#define   UART_COM_PORT_OUT        P1OUT
#define   UART_COM_PORT_DIR        P1DIR
#define   UART_COM_PORT_IE         P1IE
#define   UART_COM_PORT_IES        P1IES
#define   UART_COM_PORT_IFG        P1IFG

#define   STATUS_LED               BIT0
#define   DATA_LED                 BIT1
#define   LED_PORT_DIR             P1DIR
#define   LED_PORT_OUT             P1OUT

#define   OH                       BIT1
#define   CID                      BIT0
#define   RING                     BIT5
#define   _911_DET                 BIT6

#define   CTL_PORT_DIR             P2DIR
#define   CTL_PORT_OUT             P2OUT
#define   CTL_PORT_IE              P2IE
#define   CTL_PORT_IES             P2IES
#define   CTL_PORT_IFG             P2IFG

//////////////////////////////////////////////////////////////////////////////////////////
// modem states
#define	  CommandMode              0x01
#define	  RingDetection            0x02
#define	  AnsweringHandshake       0x03
#define	  DataMode                 0x04

// handshake states
#define   DetectOnes               0x01
#define   OnesDetected             0x02
#define   Silence150ms             0x03
#define   Answer3300ms             0x04
#define   Wait75ms                 0x05

//////////////////////////////////////////////////////////////////////////////////////////
// modem status flags
#define   ValidRingFG              BIT0
#define   OFF_HOOK                 BIT1
#define   STARTBIT                 BIT2
#define   POSITIVE_EDGE            BIT3
#define   STABILIZE_DCO            BIT4

//////////////////////////////////////////////////////////////////////////////////////////
// clock timing definitions
#define   DELTA                    250                           // defines MCLK : default: 1024000 Hz
#define   MCLK_DCO                 ((unsigned long)DELTA * 4096) // main clock is a multiple of 4096 Hz
#define   TA_CLK                   MCLK_DCO                      // MCLK is used to clock Timer_A

// negotiation handshake and other timing constants
#define   OFF_HOOK_TIMEOUT         3000
#define   NH_SILENCE               45                            //  45 * 1/300Hz =  150 ms
#define   NH_ANSWER                990                           // 990 * 1/300Hz = 3300 ms
#define   NH_WAIT                  23                            //  23 * 1/300Hz =   76 ms
#define   NH_ONES                  180                           // 180 * 1/300Hz =  600 ms
#define   DetectTime_Chn1          152                           // defines time to detect binary ones (980 Hz)
// transmitted by the caller
// DetectTime_Chn1 * 1/980Hz ~ 155ms
// delay routine constants
#define   D_LOOPBODY               3                             // MSP430: 3 cycles for inner loop
#define   D_LOOPCOUNT1ms           (unsigned int)((MCLK_DCO / (D_LOOPBODY*1000)))

#define   F_CENTER_CHN1            1080                          // center frequency for channel 1
#define   F_CENTER_CHN2            1750                          // center frequency for channel 2
#define   F_ANSWER                 2100                          // answer tone frequency
#define   F_DELTA                  100                           // delta for mark and space frequencies

#define   F_CENTER_CHN1_SPACE      (F_CENTER_CHN1 + F_DELTA)     // channel 1 SPACE frequency (0) 1180 Hz
#define   F_CENTER_CHN1_MARK       (F_CENTER_CHN1 - F_DELTA)     // channel 1 MARK  frequency (1)  980 Hz
#define   F_CENTER_CHN2_SPACE      (F_CENTER_CHN2 + F_DELTA)     // channel 2 SPACE frequency (0) 1850 Hz
#define   F_CENTER_CHN2_MARK       (F_CENTER_CHN2 - F_DELTA)     // channel 2 MARK  frequency (1) 1650 Hz

#define   RX_FREQ_CHN1_SPACE       (unsigned int)(TA_CLK / F_CENTER_CHN1_SPACE) // theoretical period value for 1180 Hz
#define   RX_FREQ_CHN1_MARK        (unsigned int)(TA_CLK / F_CENTER_CHN1_MARK)  // theoretical period value for  980 Hz
#define   CHN1_MARGIN              (unsigned int)((RX_FREQ_CHN1_MARK - RX_FREQ_CHN1_SPACE)/2) // tolerance window
#define   BAUDRATE                 (unsigned int)(TA_CLK / 300)                 // period value for baudrate
#define   _1650HzBitFreq           (unsigned int)(TA_CLK / ( 2 * F_CENTER_CHN2_MARK )) // period value for 1650 Hz frequency generation
#define   _1850HzBitFreq           (unsigned int)(TA_CLK / ( 2 * F_CENTER_CHN2_SPACE)) // 1850 Hz frequency generation
#define   _2100HzAnsFreq           (unsigned int)(TA_CLK / ( 2 * F_ANSWER))            // 2100 Hz frequency generation

//////////////////////////////////////////////////////////////////////////////////////////
// function prototypes
void modem(void);
void HardwareInit(void);
void SetupLEDs(void);
void ModemInit(void);
void Setup_LITELINK(void);
void SetupFSKModem(void);
void StartFSKModulator(void);
void StartFSKDemodulator(void);
void StartAnswerSequence(void);
void StopAnswerSequence(void);
void StopFSKModem(void);
void Disconnect(void);
void delay(unsigned int msec);
void delay_cycles(unsigned int Num);
void putchr(unsigned char c);
void Set_DCO(void);               // Subroutine to set DCO to selected frequency

// extern user application function to be called by the UART receive interrupt
extern void RcvdChrHandler(unsigned char chr);

// make ModemStateMachine and ModemStatusReg accessible for user application
extern unsigned char ModemStateMachine;
extern unsigned char ModemStatusReg;

#endif 	// _v_21_H

//////////////////////////////////////////////////////////////////////////////////////////
//
// file        : softmodem.h
// author      : Wolfgang Lutsch
//               Texas Instruments
// last update : 06/15/2004
// description : header-file for softmodem.c
//               Built with IAR Embedded Workbench Version: 2.21B
//
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef   _softmodem_H
#define   _softmodem_H

#define  WelcomeMessage    BIT0
#define  CHAR_RCVD         BIT1
#define  ADC_INIT          BIT2
#define  ADC_vAVIAL        BIT3

// function prototypes
void RcvdChrHandler(unsigned char chr);
void ProcessRcvdChr(void);
void prtstr(char* txt);
void EnableADC12(void);
void TriggerADC12(void);
void int2Hex(unsigned long v, unsigned char Len, char* ps);

#endif 	// _softmodem_H

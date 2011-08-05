//////////////////////////////////////////////////////////////////////////////////////////
//
// file        : ring_detection.h
// author      : Wolfgang Lutsch
//               Texas Instruments
// last update : 06/15/2004
// description : header file for ring_detection.c Soft-Modem Demo
//               Built with IAR Embedded Workbench Version: 2.21B
//
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef   _ring_detection_H
#define   _ring_detection_H

#define   _250ms        1
#define   _750ms        3
#define   _2000ms       8
#define   _2250ms       9
#define   _2500ms       10
#define   _2750ms       11
#define   _3000ms       12
#define   _3250ms       13
#define   _3500ms       14
#define   _3750ms       15
#define   _4000ms       16
#define   _10000ms      40
#define   TIMER_A_INIT  (TASSEL_2+TACLR)
#define   WDT_INIT      WDT_ADLY_250

//////////////////////////////////////////////////////////////////////////////////////////
#define   MAX_RINGS     2           // after MAX_RINGS valid ring signals 
                                    // we will take the call
//////////////////////////////////////////////////////////////////////////////////////////
// function prototypes
void Setup_RingDetection(void);
void Init_RingDetection(void);
void StartWDT_IntervalTimer(void);
void StopWDT(void);
void go_off_hook(void);
void go_on_hook(void);

#endif 	// _ring_detection_H

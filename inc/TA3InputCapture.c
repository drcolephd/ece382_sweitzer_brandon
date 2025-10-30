// TA3InputCapture.c
// Runs on MSP432
// Use Timer A3 in capture mode to request interrupts on rising
// edges of P10.4 (TA3CCP0) and P8.2 (TA3CCP2) and call user
// functions.
// Daniel Valvano
// May 30, 2017

// Comments revised by Stan Baek
// October 21, 2024

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/


// External signal input on P10.4 (TA3CCP0) triggers interrupt on rising edge
// External signal input on P10.5 (TA3CCP1) triggers interrupt on rising edge

#include <stdint.h>
#include "msp.h"

static void (*CaptureTask0)(uint16_t time); // User-defined function to handle interrupt on P10.4 (TA3CCP0)
static void (*CaptureTask1)(uint16_t time); // User-defined function to handle interrupt on P10.5 (TA3CCP1)

//------------TimerA3Capture_Init------------
// Initializes Timer A3 in capture mode to generate interrupts on rising edges
// of P10.4 (TA3CCP0) and P10.5 (TA3CCP1). When an interrupt occurs, the
// corresponding user-defined function is called.
// Input:
//   task0 - Pointer to a function that handles P10.4 rising edge interrupt
//           The function parameter is the 16-bit timer value at the time of the interrupt.
//   task1 - Pointer to a function that handles P10.5 rising edge interrupt
// Output: none
// Assumptions: SMCLK is 12 MHz
void TimerA3Capture_Init(void(*task0)(uint16_t time), void(*task1)(uint16_t time)){
	// write this as part of lab 16


    CaptureTask0 = task0;          // user function
    CaptureTask1 = task1;          // user function
    P10->SEL0 |= 0x30;
    P10->SEL1 &= ~0x30;
    // halt Timer A3
    TIMER_A3->CTL &= ~0x0030;
    // or TIMER_A3->CTL = 0; //
    // SMCLK, input divider /1, TACLR = 0, stop mode,
    TIMER_A3->CTL = 0x0200; // no friendly
    TIMER_A3->EX0 = 0x7;
    TIMER_A3->CCTL[0] = 0x4910;
    TIMER_A3->CCTL[1] = 0x4910;
    NVIC->IP[14] = 2 << 5;
    NVIC->IP[15] = 2 << 5;
    NVIC->ISER[0] = 0b11 << 14; // or 0x0000C000;
    TIMER_A3->CTL |= 0x0024; // must be friendly
    // or TIMER_A3->CTL = 0x0224;


}


//------------TA3_0_IRQHandler------------
// Interrupt handler for Timer A3 CCR0 (rising edge on P10.4).
// It clears the interrupt and calls the user function.
// Input: none
// Output: none
void TA3_0_IRQHandler(void){
	// write this as part of lab 16

    // Acknowledge the interrupt and clear the flag
	TIMER_A3->CCTL[0] &= ~0x0001;

    // Call the user function with the timer value
	(*CaptureTask0)(TIMER_A3->CCR[0]);

}


//------------TA3_N_IRQHandler------------
// Interrupt handler for Timer A3 CCR1 (rising edge on P10.5).
// It clears the interrupt and calls the user function.
// Input: none
// Output: none
void TA3_N_IRQHandler(void){
    // write this as part of lab 16

    // Acknowledge the interrupt and clear the flag
	TIMER_A3->CCTL[1] &= ~0x0001;
								 
    // Call the user function with the timer value
	(*CaptureTask1)(TIMER_A3->CCR[1]);

}

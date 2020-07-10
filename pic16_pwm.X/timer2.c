/*
 * File:   timer2.c
 * Author: Haesh_MikoToniks
 *
 * Created on June 15, 2020, 4:33 PM
 */


#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include "define.h"
#include "timer2.h"

void timer2_init(void)
{
    // Set TMR2 to the options selected in the User Interface

    // T2CS FOSC/4; 
    T2CLKCON = 0x01;

    // T2PSYNC Not Synchronized; T2MODE Software control; T2CKPOL Rising Edge; T2CKSYNC Not Synchronized; 
    T2HLT = 0x00;

    // T2RSEL T2INPPS pin; 
    T2RST = 0x00;

    // PR2 124; 
    T2PR = 0x7C;

    // TMR2 0; 
    T2TMR = 0x00;

    // Clearing IF flag.
    PIR4bits.TMR2IF = 0;

    // T2CKPS 1:16; T2OUTPS 1:1; TMR2ON on; 
    T2CON = 0xC0;
}

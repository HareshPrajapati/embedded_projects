/*
 * File:   TMR2.c
 * Author: Haesh_MikoToniks
 *
 * Created on June 16, 2020, 10:50 AM
 */


#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include "DEFINE.h"
#include "TMR2.h"

void timer2_init(void)
{    
    //set  pins of  TIMERS  with PPS
    
    
    LATC = 0x00;
    TRISC = 0xDE;   //1101 1110
    ANSELC = 0x1C;  //0001 1100
    SLRCONC = 0xDF; 
    INLVLC = 0xDF;
    RC0PPS = 0x09;   //RC0->CCP1:CCP1; 
    
    T2INPPS = 0x17;   //RC7->TMR2:T2IN;
    
    
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

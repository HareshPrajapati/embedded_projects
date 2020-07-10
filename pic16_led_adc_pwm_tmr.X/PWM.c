/*
 * File:   PWM.c
 * Author: Haesh_MikoToniks
 *
 * Created on June 16, 2020, 10:50 AM
 */


#include <xc.h>
#include "DEFINE.h"
#include <stdio.h>
#include <stdint.h>
#include "PWM.h"

void PWM1_init(void)
{   
     //set Output pins of  PWM  with PPS
    
    
    LATC = 0x00;
    TRISC = 0xDE;   //1101 1110
    ANSELC = 0x1C;  //0001 1100
    SLRCONC = 0xDF; 
    INLVLC = 0xDF;
    
    RC0PPS = 0x09;   //PWM1 : RC0->CCP1:CCP1; 
    
    
    
    
    
    // NOSC HFINTOSC; NDIV 4; 
    OSCCON1 = 0x62;
    // CSWHOLD may proceed; SOSCPWR Low power; 
    OSCCON3 = 0x00;
    // MFOEN disabled; LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled; 
    OSCEN = 0x00;
    // HFFRQ 32_MHz; 
    OSCFRQ = 0x06;
    // MFOR not ready; 
    OSCSTAT = 0x00;
    // TUN 0; 
    OSCTUNE = 0x00;
    // ACTUD enabled; ACTEN disabled; 
    ACTCON = 0x00;
    
    // Set the PWM1 to the options selected in the User Interface
	
	// MODE PWM; EN enabled; CCP1FMT right_aligned; 
	CCP1CON = 0x8C;    
	
	// RH 1; 
	CCPR1H = 0x01;    
	
	// RL 243; 
	CCPR1L = 0xF3;    

	// Selecting Timer 2
	CCPTMRS0bits.C1TSEL = 0x1;
    
}

void PWM1_Duty(uint16_t dutyValue)
{
    dutyValue &= 0x03FF;
    
    // Load duty cycle value
    if(CCP1CONbits.CCP1FMT)
    {
        dutyValue <<= 6;
        CCPR1H = dutyValue >> 8;
        CCPR1L = dutyValue;
    }
    else
    {
        CCPR1H = dutyValue >> 8;
        CCPR1L = dutyValue;
    }
}


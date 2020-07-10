/*
 * File:   newmain.c
 * Author: Haesh_MikoToniks
 *
 * Created on June 15, 2020, 4:31 PM
 */


#define FCY 8000000L
#define _XTAL_FREQ 8000000
#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include "define.h"
#include "timer2.h"
#include "pwm.h"

void main(void) {
    
    TRISC = 0xDE;
    ANSELC = 0xDC;
    RC0PPS = 0x09;   //RC0->CCP1:CCP1;
    uint8_t i;
    timer2_init();
    PWM1_init();


    while (1) {

        for (i = 0; i < 255; i++) {
            PWM1_Duty(i);
            __delay_ms(10);

        }
        __delay_ms(2000);
        for (i = 255; i > 0; i--) {
            PWM1_Duty(i);
            __delay_ms(10);

        }
        __delay_ms(2000);
    }

}

    


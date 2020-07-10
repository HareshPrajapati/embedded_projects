/*
 * File:   newmain.c
 * Author: Haesh_MikoToniks
 *
 * Created on June 16, 2020, 10:49 AM
 */


#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "DEFINE.h"
#include "PWM.h"
#include "TMR2.h"
#include "ADC.h"

void main(void) {
    timer2_init();
    ADC_Init();
    PWM1_init();


    while (1) {
        adc_result_t PotVal = ADC_GetSingleValue(channel_RC4);
        PWM1_Duty(PotVal);
    }
}

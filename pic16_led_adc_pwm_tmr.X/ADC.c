/*
 * File:   ADC.c
 * Author: Haesh_MikoToniks
 *
 * Created on June 16, 2020, 10:50 AM
 */


#include <xc.h>
#include "DEFINE.h"
#include "ADC.h"

void ADC_Init(void)
{   
    
    //set input pins of ADC  with PPS
    LATC = 0x00;
    TRISC = 0xDE;   //1101 1110
    ANSELC = 0x1C;  //0001 1100
    SLRCONC = 0xDF; 
    INLVLC = 0xDF;
    ADCACTPPS = 0x16;   //RC6->ADCC:ADCACT;    
    
    
    // set the ADCC to the options selected in the User Interface
    // ADLTH 0; 
    ADLTHL = 0x00;
    // ADLTH 0; 
    ADLTHH = 0x00;
    // ADUTH 0; 
    ADUTHL = 0x00;
    // ADUTH 0; 
    ADUTHH = 0x00;
    // ADSTPT 0; 
    ADSTPTL = 0x00;
    // ADSTPT 0; 
    ADSTPTH = 0x00;
    // ADACC 0; 
    ADACCU = 0x00;
    // ADRPT 0; 
    ADRPT = 0x00;
    // ADPCH ANA0; 
    ADPCH = 0x00;
    // ADACQ 160; 
    ADACQL = 0xA0;
    // ADACQ 15; 
    ADACQH = 0x0F;
    // ADCAP Additional uC disabled; 
    ADCAP = 0x00;
    // ADPRE 0; 
    ADPREL = 0x00;
    // ADPRE 0; 
    ADPREH = 0x00;
    // ADDSEN disabled; ADGPOL digital_low; ADIPEN disabled; ADPPOL Vss; 
    ADCON1 = 0x00;
    // ADCRS 0; ADMD Basic_mode; ADACLR disabled; ADPSIS RES; 
    ADCON2 = 0x00;
    // ADCALC First derivative of Single measurement; ADTMD disabled; ADSOI ADGO not cleared; 
    ADCON3 = 0x00;
    // ADMATH registers not updated; 
    ADSTAT = 0x00;
    // ADPREF VDD; 
    ADREF = 0x00;
    // ADACT disabled; 
    ADACT = 0x00;
    // ADCS FOSC/2; 
    ADCLK = 0x00;
    // ADGO stop; ADFM right; ADON enabled; ADCS Frc; ADCONT disabled; 
    ADCON0 = 0x94;
    

}



void ADC_Start(adc_channel_t channel)
{
    // select the A/D channel
    ADPCH = channel;      
  
    // Turn on the ADC module
    ADCON0bits.ADON = 1;

    // Start the conversion
    ADCON0bits.ADGO = 1;
}

adc_result_t ADC_GetSingleValue(adc_channel_t channel)
{
    // select the A/D channel
    ADPCH = channel;  

    // Turn on the ADC module
    ADCON0bits.ADON = 1;
	
    //Disable the continuous mode.
    ADCON0bits.ADCONT = 0;    

    // Start the conversion
    ADCON0bits.ADGO = 1;


    // Wait for the conversion to finish
    while (ADCON0bits.ADGO)
    {
    }
    
    
    // Conversion finished, return the result
    return ((adc_result_t)((ADRESH << 8) + ADRESL));
}

void ADC_Stop(void)
{
    //Reset the ADGO bit.
    ADCON0bits.ADGO = 0;
}



uint16_t ADC_GetFilterValue(void)
{
    //Return the contents of ADFLTRH and ADFLTRL registers
    return ((uint16_t)((ADFLTRH << 8) + ADFLTRL));
}

uint16_t ADC_GetPreviousValue(void)
{
    //Return the contents of ADPREVH and ADPREVL registers
    return ((uint16_t)((ADPREVH << 8) + ADPREVL));
}

void ADC_DefineSetPoint(uint16_t setPoint)
{
    //Sets the ADSTPTH and ADSTPTL registers
    ADSTPTH = setPoint >> 8;
    ADSTPTL = setPoint;
}



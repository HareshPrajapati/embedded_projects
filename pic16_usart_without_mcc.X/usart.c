/*
 * File:   usart.c
 * Author: Haesh_MikoToniks
 *
 * Created on June 15, 2020, 10:04 AM
 */


#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include "define.h"
#include "usart.h"


void USART_Init()
{
   
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
    
    
    
    BAUD1CON = 0x08;

    // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled; 
    RC1STA = 0x90;

    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave; 
    TX1STA = 0x24;

    // SP1BRGL 207; boudrate 9600 freq 8000000
   // SP1BRGL = 0xCF;
    SP1BRGL = 207;

    // SP1BRGH 0; 
    SP1BRGH = 0x00;
 
}


uint8_t EUSART1_Read(void)
{
    while(!PIR3bits.RC1IF)
    {
    }

    
    
    if(1 == RC1STAbits.OERR)
    {
        // EUSART1 error - restart

        RC1STAbits.CREN = 0; 
        RC1STAbits.CREN = 1; 
    }

    return RC1REG;
}

void EUSART1_Write(uint8_t txData)
{
    while(0 == PIR3bits.TX1IF)
    {
    }

    TX1REG = txData;    // Write the data byte to the USART.
}

char getch(void)
{
    return EUSART1_Read();
}

void putch(char txData)
{
    EUSART1_Write(txData);
}
 
void USART_Print(const char *data)
{
  uint8_t i = 0;
  while (data[i] != '\0')
    putch(data[i++]);
}



/*
 * File:   newmain.c
 * Author: Haesh_MikoToniks
 *
 * Created on June 15, 2020, 10:02 AM
 */






#include <xc.h>
#include <stdio.h>
#include "define.h"
#include "usart.h"

void main(void) {
    
    LATC = 0x00;
    TRISC = 0x9F;
    ANSELC = 0x5C;
    RX1PPS = 0x17;   //RC7->EUSART1:RX1;    
    RC6PPS = 0x0D;   //RC6->EUSART1:TX1;
    USART_Init();
   
    
    while (1) {
      
        USART_Print("harry \r\n");

    }

   
     
    
}

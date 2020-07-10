/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.3
        Device            :  PIC16LF19156
        Driver Version    :  2.00
 */

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
 */

#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <stdint.h>
#include <pic16lf19156.h>
#define RW_LAT PORTCbits.RC6
#define RE     PORTCbits.RC1
#define RST_LAT PORTCbits.RC7
#define ENABLE_LAT PORTCbits.RC4
#define CS1_LAT PORTCbits.RC3
#define CS2_LAT PORTBbits.RB3

void lcdcmd(char value) {
    PORTA = value;
    RW_LAT = 0;
    RST_LAT = 0;
    ENABLE_LAT = 0;
    __delay_ms(30);
    ENABLE_LAT = 1;
    __delay_ms(2);
    ENABLE_LAT = 0;

}

void lcddata(char data1) {
    PORTA = data1;
    RW_LAT = 0;
    RST_LAT = 1;
    ENABLE_LAT = 0;
    __delay_ms(30);
    ENABLE_LAT = 1;
    __delay_ms(2);
    ENABLE_LAT = 0;
}

void clearlcd() {
    unsigned int i = 0, j, k;
    int page[] = {0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF};
    CS1_LAT = 1; //First Half is selected
    CS2_LAT = 0; //Second Half unselected
    lcdcmd(0xC0); //Start line, (64 rows) selects from where to start(1 line) 

    while (i != 8) {
        CS2_LAT = 0; //Second Half unselected		
        CS1_LAT = 1; //First Half is selected
        lcdcmd(page[i]); //Setting x-address, page is selected
        lcdcmd(0x40); //Setting y-address, coulomb of page is selected

        for (j = 1; j <= 65; j++) {
            if (j <= 64) {
                lcddata(0xF0);
            }
            else if (j == 65) {
                CS2_LAT = 1; //Second Half Selected
                CS1_LAT = 0; //First Half unselected
                lcdcmd(page[i]); //Setting x-address, page is selected
                lcdcmd(0x40); //Setting y-address, coulomb of page is selected

                for (k = 0; k <= 63; k++)
                    lcddata(0x0F);
            }
        }
        i = i + 1;
    }
}
void glcd_SelectPage0(){
     CS1_LAT = 1;
}

void glcd_SelectPage1(){
    CS2_LAT = 1;
}
void GLCD_Init()
{
	
    /* Select the Page0/Page1 and Enable the GLCD */
    glcd_SelectPage0();
    lcdcmd(0x3f);
    glcd_SelectPage1();
    lcdcmd(0x3f);
    __delay_ms(10);

    /* Select the Page0/Page1 and Enable the GLCD */
    glcd_SelectPage0();
    lcdcmd(0xc0);
    glcd_SelectPage1();
    lcdcmd(0xc0);

   /* Clear the complete LCD and move the cursor to beginning of page0*/
//    clearlcd();
}

/*
                         Main application
 */
void main(void) {
    // initialize the device
    SYSTEM_Initialize();
    TRISC = 0;
    TRISA = 0;
    TRISB = 0;
    

    

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    GLCD_Init();


    

    lcddata(0x7F); //M
    lcddata(0x02);
    lcddata(0x0C);
    lcddata(0x02);
    lcddata(0x7f);
    lcddata(0x00);
    lcddata(0xFF); //M
//
//    lcddata(0x0D); //i
//    lcddata(0xFF);
//
//    lcddata(0xC7); //c
//    lcddata(0xBB);
//    lcddata(0x7D);
//    lcddata(0x7D); //c
//    lcddata(0xFF);
//
//    lcddata(0x01); //r
//    lcddata(0xF7);
//    lcddata(0xFB);
//    lcddata(0xFD); //r
//    lcddata(0xFF);
//
//    lcddata(0xC3); //o
//    lcddata(0x3D);
//    lcddata(0x3D);
//    lcddata(0xC3); //o
//    lcddata(0xFF);
//
//    lcddata(0xC7); //c
//    lcddata(0xBB);
//    lcddata(0x7D);
//    lcddata(0x7D); //c
//    lcddata(0xFF);
//
//    lcddata(0xC3); //o
//    lcddata(0x3D);
//    lcddata(0x3D);
//    lcddata(0xC3); //o
//    lcddata(0xFF);
//
//    lcddata(0x01); //n
//    lcddata(0xF7);
//    lcddata(0xF7);
//    lcddata(0x07); //n
//    lcddata(0xFF);
//
//
//    lcddata(0xF7); //t
//    lcddata(0x01);
//    lcddata(0x67);
//    lcddata(0x7F); //t
//    lcddata(0xFF);
//
//    lcddata(0x01); //r
//    lcddata(0xF7);
//    lcddata(0xFB);
//    lcddata(0xFD); //r
//    lcddata(0xFF);
//
//    lcddata(0xC3); //o
//    lcddata(0x3D);
//    lcddata(0x3D);
//    lcddata(0xC3); //o
//    lcddata(0xFF);
//
//    lcddata(0x01); //l
//    lcddata(0x01);
//    lcddata(0xFF); //l
//
//    lcddata(0x01); //l
//    lcddata(0x01);
//    lcddata(0xFF); //l
//
//    lcddata(0xC7); //e
//    lcddata(0xAB);
//    lcddata(0x75);
//
//    CS1_LAT = 0; //Switch off First Half
//    CS2_LAT = 1; //Selecting 2nd Half
//    lcdcmd(0x3F); //Display on
//    lcdcmd(0x40); //Setting y-address
//    lcdcmd(0xBB); //Setting x-address page 3 is selected
//    //lcdcmd(0xC0);    //start line
//
//    lcddata(0x79); //e
//    lcddata(0xFF);
//
//    lcddata(0x01); //r
//    lcddata(0xF7);
//    lcddata(0xFB);
//    lcddata(0xFD); //r
//    lcddata(0xFF);
//
//
//    lcddata(0xF7); //-
//    lcddata(0xF7);
//    lcddata(0xF7); //-
//    lcddata(0xFF);
//
//    lcddata(0x01); //p
//    lcddata(0xED);
//    lcddata(0xED);
//    lcddata(0xF3); //p
//    lcddata(0xFF);
//
//
//    lcddata(0x01); //r
//    lcddata(0xF7);
//    lcddata(0xFB);
//    lcddata(0xFD); //r
//    lcddata(0xFF);
//
//
//    lcddata(0xC3); //o
//    lcddata(0x3D);
//    lcddata(0x3D);
//    lcddata(0xC3); //o
//    lcddata(0xFF);
//
//    lcddata(0xBF); //j
//    lcddata(0x7F);
//    lcddata(0x05); //j
//    lcddata(0xFF);
//
//    lcddata(0xC7); //e
//    lcddata(0xAB);
//    lcddata(0x75);
//    lcddata(0x79); //e
//    lcddata(0xFF);
//
//    lcddata(0xC7); //c
//    lcddata(0xBB);
//    lcddata(0x7D);
//    lcddata(0x7D); //c
//    lcddata(0xFF);
//
//    lcddata(0xF7); //t
//    lcddata(0x01);
//    lcddata(0x67);
//    lcddata(0x7F); //t
//    lcddata(0xFF);
//
//    lcddata(0x9F); //.
//    lcddata(0x9F); //.
//    lcddata(0xFF);
//
//    lcddata(0xC7); //c
//    lcddata(0xBB);
//    lcddata(0x7D);
//    lcddata(0x7D); //c
//    lcddata(0xFF);
//
//    lcddata(0xC3); //o
//    lcddata(0x3D);
//    lcddata(0x3D);
//    lcddata(0xC3); //o
//    lcddata(0xFF);
//
//    lcddata(0x01); //m
//    lcddata(0xFD);
//    lcddata(0x01);
//    lcddata(0xFD);
//    lcddata(0x01); //m
//    lcddata(0xFF);



    while (1) {
        printf("this is simple interface \r\n");
    }
}
/**
 End of File
 */
;******************************************************************************
;   This file is a basic code template for code generation on the             *
;   PIC16F1937. This file contains the basic code building blocks to build    *
;   upon.                                                                     *
;                                                                             *
;   Refer to the MPASM User's Guide for additional information on             *
;   features of the assembler.                                                *
;                                                                             *
;   Refer to the respective data sheet for additional                         *
;   information on the instruction set.                                       *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Filename:        pic16lf19156.asm                                                 *
;    Date:                                                                    *
;    File Version:                                                            *
;                                                                             *
;    Author:                                                                  *
;    Company:                                                                 *
;                                                                             *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Files Required: pic16lf19156.INC                                             *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Notes:                                                                   *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Revision History:                                                        *
;                                                                             *
;******************************************************************************


	list		p=16f19156      ; list directive to define processor
	#include	<p16lf19156.inc> ; processor specific variable definitions

;------------------------------------------------------------------------------
;
; CONFIGURATION WORD SETUP
;
; The 'CONFIG' directive is used to embed the configuration word within the 
; .asm file. The lables following the directive are located in the respective 
; .inc file.  See the data sheet for additional information on configuration 
; word settings.
;
;------------------------------------------------------------------------------    

; CONFIG1
; __config 0xFEE8
 __CONFIG _CONFIG1, _FEXTOSC_OFF & _RSTOSC_HFINT1 & _CLKOUTEN_ON & _VBATEN_OFF & _LCDPEN_ON & _CSWEN_ON & _FCMEN_ON
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _MCLRE_ON & _PWRTE_OFF & _LPBOREN_OFF & _BOREN_ON & _BORV_LO & _ZCD_OFF & _PPS1WAY_ON & _STVREN_ON
; CONFIG3
; __config 0xFF9F
 __CONFIG _CONFIG3, _WDTCPS_WDTCPS_31 & _WDTE_OFF & _WDTCWS_WDTCWS_7 & _WDTCCS_SC
; CONFIG4
; __config 0xFFFF
 __CONFIG _CONFIG4, _BBSIZE_512 & _BBEN_OFF & _SAFEN_OFF & _WRTAPP_OFF & _WRTB_OFF & _WRTC_OFF & _WRTD_OFF & _WRTSAF_OFF & _LVP_ON
; CONFIG5
; __config 0xFFFF
 __CONFIG _CONFIG5, _CP_OFF
	


;------------------------------------------------------------------------------
; VARIABLE DEFINITIONS
;
; Available Data Memory divided into Bank 0-15.  Each Bank may contain 
; Special Function Registers, General Purpose Registers, and Access RAM 
;
;------------------------------------------------------------------------------

reg UDATA
REGA  res 1
REGB  res 1
REGC  res 1

 

;------------------------------------------------------------------------------
; RESET VECTOR
;------------------------------------------------------------------------------

    ORG     0x0000            ; processor reset vector
    PAGESEL START
    GOTO    START             ; When using debug header, first inst.
    

 

Main Code
;org 0x04

;--------------------------Routines-------------------------------------

Delay:
			;249993 cycles
	movlw	0x4E
	movwf	REGA
	movlw	0xc4
	movwf	REGB
oneSecondDelay_0
	decfsz	REGA, f
	goto	$+2
	decfsz	REGB, f
	goto	oneSecondDelay_0
			;3 cycles
	goto	$+1
	nop

			;4 cycles (including call)
	retlw 0    

;Delay2
;movlw D'4'
;movwf REGC
;loop3
;movlw D'50'
;movwf REGB
;loop2
;movlw D'200'
;movwf REGA
;loop 
;nop  ;1
;nop ;1
;decfsz REGA,F ;1
;goto loop  ;2
;decfsz REGB,F
;goto loop2
;decfsz REGC,F
;goto loop3
;return

	
 ;------------------------------------------- software delay-------------------
Delaycounter:
   BANKSEL	PORTC		 ;Select bank 0, i.e. where PORTD is	
   btfsc	PORTC,0x04	 ;Check switch status, is it high or low ?
   goto 	startBlink	 ;If high execute startBlinking
   goto         stopBlink
 



;------------------------------------------------------------------------------
; MAIN PROGRAM
;------------------------------------------------------------------------------

START
    
 

    BANKSEL  TRISC
    MOVLW    0x10     ;RC3 output ,RC4 input
    MOVWF    TRISC
    
    banksel  INLVLC
    movlw    0xDf
    movwf    INLVLC
    
    BANKSEL  ANSELC
    MOVLW    0x00
    MOVWF    ANSELC
    
    

    
    
              


     
    

    
    
   
;--------------------Starting Main Program---------------------------
     
checkSwitch:	
			        BANKSEL	PORTC		 ;Select bank 0, i.e. where PORTD is	
				btfss	PORTC,0x04	 ;Check switch status, is it high or low ? 
				goto    Delaycounter
;				goto    startBlink
				goto 	stopBlink	 ;If low execute stopBlinking
			
startBlink		        
                                
				
 
				BANKSEL	LATC             ;Select bank again, just to be sure
				bsf LATC,0x03
				PAGESEL Delay
				call	Delay	;Keep lit for 1s
		                BANKSEL	LATC             ;Select bank again, just to be sure
				bcf LATC,0x03
			
				PAGESEL Delay
				call	Delay	;Keep off for 1s
                                goto startBlink
		

stopBlink                      
			       
				 
				goto	checkSwitch	;Goto to routine to determine status of switch


				END         
 


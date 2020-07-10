; TODO INSERT CONFIG CODE HERE USING CONFIG BITS GENERATOR
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
	
    
    
    ; TODO INSERT CONFIG CODE HERE USING CONFIG BITS GENERATOR

RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    START                   ; go to beginning of program

; TODO ADD INTERRUPTS HERE IF USED

;MAIN_PROG CODE                      ; let linker place main program

    
    
    
START
     
    banksel TRISC
    bcf TRISC,RC6
    banksel ANSELC
    bsf ANSELC,ANSC3
    
    banksel RC6PPS 
    movlw 0x0D
    movwf RC6PPS
    
    ; NOSC HFINTOSC; NDIV 4;
    banksel OSCCON1
    movlw 0x62
    movwf OSCCON1
   
    ;CSWHOLD may proceed; SOSCPWR Low power; 
    banksel OSCCON3
    movlw 0x00
    movwf  OSCCON3
  
    ; MFOEN disabled; LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled; 
    banksel OSCEN
    movlw 0x00
    movwf OSCEN
  
    ; HFFRQ 4_MHz; 
     banksel  OSCFRQ
    movlw  0x02
    movwf OSCFRQ

    ; MFOR not ready; 
    banksel  OSCSTAT
    movlw 0x00
    movwf  OSCSTAT

    ; TUN 0; 
      banksel  OSCTUNE
    movlw 0x00
    movwf  OSCTUNE
  
  
    ; ACTUD enabled; ACTEN disabled; 
    banksel ACTCON
    movlw 0x00
    movwf ACTCON
   
    
    
    banksel BAUD1CON
    movlw 0x08
    movwf BAUD1CON
   

    ; SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled;
    banksel  RC1STA
    movlw 0x90
    movwf  RC1STA
   

    ; TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave; 
    banksel TX1STA
    movlw 0x24
    movwf TX1STA
    

    ; SP1BRGL 25;; boudrate 9600 freq 1000000
  
    banksel SP1BRGL
    movlw 0x19
    movwf SP1BRGL
  

    ; SP1BRGH 0; 
    banksel SP1BRGH
    movlw 0x00
    movwf SP1BRGH
    
    
trigger_loop
    
    
;    btfss PORTC, RC6
;    goto trigger_loop
    
    ; start transmitting
    movlw 0x48	; 'H'
    call transmit
    
    movlw 0x65	; 'e'
    call transmit
    
    movlw 0x6c	; 'l'
    call transmit
    
    movlw 0x6c	; 'l'
    call transmit
    
    movlw 0x6f	; 'o'
    call transmit
    
    movlw 0x20	; SPACE
    call transmit
    
    movlw 0x77	; 'w'
    call transmit
    
    movlw 0x6f	; 'o'
    call transmit
    
    movlw 0x72	; 'r'
    call transmit
    
    movlw 0x6c	; 'l'
    call transmit
    
    movlw 0x64	; 'd'
    call transmit
    
    movlw 0x21	; '!'
    call transmit
    
    GOTO trigger_loop   

transmit
    banksel TX1REG
    movwf TX1REG
txreg_wait_loop
    ; check that TXREG is empty
    banksel PIR3
    btfss PIR3, TX1IF
    goto txreg_wait_loop
    

    END
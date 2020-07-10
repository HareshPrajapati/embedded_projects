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
	


;------------------------------------------------------------------------------
; VARIABLE DEFINITIONS
;
; Available Data Memory divided into Bank 0-15.  Each Bank may contain 
; Special Function Registers, General Purpose Registers, and Access RAM 
;
;------------------------------------------------------------------------------
cblock 0x20	; Begin General Purpose-Register
count1
count2
count3
count4
CNT
endc
 
	;***** VARIABLE DEFINITIONS
w_temp        EQU     0x71        ; variable used for context saving 
status_temp   EQU     0x72        ; variable used for context saving

;;--------------------------Routines-------------------------------------
;
;oneSecondDelay
;			;249993 cycles
;	movlw	0x4E
;	movwf	REGA
;	movlw	0xc4
;	movwf	REGB
;oneSecondDelay_0
;	decfsz	REGA, f
;	goto	$+2
;	decfsz	REGB, f
;	goto	oneSecondDelay_0
;			;3 cycles
;	goto	$+1
;	nop
;
;			;4 cycles (including call)
;	retlw 0    


 
 
 
RES_VECT  CODE    0x0000            ; processor reset vector
    PAGESEL setup
    GOTO    setup                    ; go to beginning of program


ORG     0x004             ; interrupt vector location
	movwf   w_temp            ; save off current W register contents
	movf	STATUS,w          ; move status register into W register
	movwf	status_temp       ; save off contents of STATUS register
	
	
; isr code can go here or be located as a call subroutine elsewhere
        movlw d'61' 
	; approx 61 interrupts = 1 sec. at 1 mHz crystal
	subwf CNT, w
	banksel STATUS
	btfss STATUS, Z ; i = 10
;        goto $+4
	call toggle               ;state = !state PORTA, 0
	clrf CNT
	goto $+2
  	incf CNT
	banksel PIR0
	bcf PIR0, TMR0IF ; clr TMR0 interrupt flag
	movf    status_temp,w     ; retrieve copy of STATUS register
	banksel STATUS
	movwf	STATUS            ; restore pre-isr STATUS register contents
	swapf   w_temp,f
	swapf   w_temp,w          ; restore pre-isr W register contents
	RETFIE                   ; return from interrupt     
  

;**********************************************************************

	
;*************************************************************************	
	
MAIN_PROG CODE                      ; let linker place main program
 
 
 
 
setup ; init PIC16LF19156
	
	

	banksel TRISC    ; BSF	STATUS,RP0 Jump to bank 1 use BANKSEL instead
	bcf TRISC, RC3
	bcf TRISC, RC6

        
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
  
        ;MFOEN disabled; LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled; 
        banksel OSCEN
        movlw 0x00
        movwf OSCEN
  
        ;HFFRQ 4_MHz; 
        banksel  OSCFRQ
        movlw  0x02
        movwf OSCFRQ

        ;MFOR not ready; 
        banksel  OSCSTAT
        movlw 0x00
        movwf  OSCSTAT

        ;TUN 0; 
        banksel  OSCTUNE
        movlw 0x00
        movwf  OSCTUNE
  
  
        ;ACTUD enabled; ACTEN disabled; 
        banksel ACTCON
        movlw 0x00
        movwf ACTCON
   
        ;Usart inisilize
    
        banksel BAUD1CON
        movlw 0x08
        movwf BAUD1CON
   

        ;SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled;
        banksel  RC1STA
        movlw 0x90
        movwf  RC1STA
   

        ;TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave; 
        banksel TX1STA
        movlw 0x24
        movwf TX1STA
    

        ;SP1BRGL 25;; boudrate 9600 freq 1000000
  
        banksel SP1BRGL
        movlw 0x19
        movwf SP1BRGL
  

        ;SP1BRGH 0; 
        banksel SP1BRGH
        movlw 0x00
        movwf SP1BRGH
	
	
	
	;setp TMR0 interrupts
	banksel T0CON1 ; Reg. 0x81 BANK1
	movlw 0x40 ; F0sc/4, T0ASYNC synchronised, prescale 1:1 0x40
	movwf T0CON1
	banksel  TMR0H 
        movlw 0xF9   ;0xF9
	movwf TMR0H
        ;TMR0L 0; 
        banksel TMR0L 
	movlw 0x00
	movwf TMR0L
	banksel T0CON0
	movlw 0x80
	movwf T0CON0
	banksel INTCON ; 
	bsf INTCON, GIE ; enable global interrupt
	bsf INTCON, PEIE ; enable all unmasked interrupts
	banksel PIE0
	bsf PIE0, TMR0IE ; enable TMR0 interrupt
	banksel PIR0
	bcf PIR0, TMR0IF ; clr TMR0 interrupt flag
	
	;to turn on, must be cleared after interrupt
	clrf	CNT ; RAM location 0x23

goto main

main

    movlw count1
    call transmit    
    goto	main 
	
;***********************************************************************
	
toggle  
    BANKSEL LATC
    btfss LATC, RC3
    goto $+3
    bcf LATC, RC3
    return	
    bsf LATC, RC3	
    return

transmit
    banksel TX1REG
    movwf TX1REG
txreg_wait_loop
    ; check that TXREG is empty
    banksel PIR3
    btfss PIR3, TX1IF
    goto txreg_wait_loop
    
    
END  ; directive 'end of program'
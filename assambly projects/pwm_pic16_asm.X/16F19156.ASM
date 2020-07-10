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
;    Filename:        pic16lf19156_pem.asm                                                 *
;    Date:            15/06/2020                                                          *
;    File Version:                                                            *
;                                                                             *
;    Author:          haresh mokrotroniks                                                        *
;    Company:         mikrotroniks                                                        *
;                                                                             *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Files Required: P16F19156.INC                                             *
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
	#include	<p16f19156.inc> ; processor specific variable definitions

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


CBLOCK 0x20
FADE_STATE ;IF = 0x00 INCREMENT CCPR1L ELSE DECREMENT CCPR1L
ENDC

;------------------------------------------------------------------------------
; RESET VECTOR
;------------------------------------------------------------------------------
    Mode	EQU		0x30 ;Intensity counter
    ORG     0x0000            ; processor reset vecto
    nop
    GOTO    START             ; When using debug header, first inst.
                              ; may be passed over by ICD2.  
   
   
;------------------------------------------------------------------------------
; MAIN PROGRAM
;------------------------------------------------------------------------------

START
     
;   MOVLW 0x00
;   IORWF FADE_STATE, W
;   BTFSS STATUS, Z ;IF FADE_STATE == 0 GOTO INC_CCPR1L
;   GOTO DEC_CCPR1L ;ELSE GOTO DEC_CCPR1L
;   
;   
;INC_CCPR1L:
;	INCFSZ CCPR1L ;INCREMENT CCPR1L
;	GOTO START
;	GOTO CHANGE_STATE_0 ;IF WE HAVE AN OVERFLOW GOTO CHANGE_STATE
;	
;DEC_CCPR1L:
;	DECFSZ CCPR1L ;DECREMENT CCPR1L
;	GOTO START
;	;IF WE HAVE AN OVERFLOW GOTO CHANGE_STATE
;	
;CHANGE_STATE:
;	COMF FADE_STATE, F ;TOGLE FADE_STATE BITS
;	INCFSZ CCPR1L
;	GOTO START
;	
;CHANGE_STATE_0:
;	COMF FADE_STATE, F ;TOGLE FADE_STATE BITS
;	DECFSZ CCPR1L
;	GOTO START
     BANKSEL   TRISC
     MOVLW     0x8C
     MOVWF     TRISC
     
     BANKSEL   ANSELC
     MOVLW     0xDC
     MOVWF     ANSELC
     
     BANKSEL   RC0PPS
     MOVLW     0x09
     MOVWF     RC0PPS
     
                            ;OSCILLATOR initialize
     BANKSEL   OSCCON1   
     MOVLW     0x62
     MOVWF     OSCCON1      ; NOSC HFINTOSC; NDIV 4; 
     
     
     BANKSEL   OSCCON3
     MOVLW     0x00  
     MOVWF     OSCCON3      ; CSWHOLD may proceed; SOSCPWR Low power; 
     
     BANKSEL   OSCEN
     MOVLW     0x00
     MOVWF     OSCEN        ;  MFOEN disabled; LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled;
     
     BANKSEL   OSCFRQ  
     MOVLW     0x06
     MOVWF     OSCFRQ       ;  HFFRQ 32_MHz; 
     
     
     BANKSEL   OSCSTAT
     MOVLW     0x00
     MOVWF     OSCSTAT      ; MFOR not ready; 
     
     BANKSEL   OSCTUNE 
     MOVLW     0x00
     MOVWF     OSCTUNE      ; TUN 0;
     
     
     BANKSEL   ACTCON 
     MOVLW     0x00
     MOVWF     ACTCON       ; ACTUD enabled; ACTEN disabled;
     
     
     
     
     
        BANKSEL STATUS
	BANKSEL T2CON
	CLRF	T2CON
        BSF     T2CON,0x0
	BANKSEL CCP1CON
	CLRF	CCP1CON		;CCP module OFF
	CLRF	TMR2		;Clear Timer2
	clrf	Mode;  		Intensity Counter
                            ;TIMER2 Initialize
;     BANKSEL   T2CLKCON 
;     MOVLW     0x01
;     MOVWF     T2CLKCON     ; T2CS Fosc = Fosc/4
;     
;     
;     BANKSEL   T2HLT 
;     MOVLW     0x00
;     MOVWF     T2HLT         ;T2PSYNC Not Synchronized; T2MODE Software control; T2CKPOL Rising Edge; T2CKSYNC Not Synchronized;
;     
;     BANKSEL   T2RST   
;     MOVLW     0x00
;     MOVWF     T2RST         ;T2RSEL T2INPPS pin; 
;     
;     BANKSEL   T2PR   
;     MOVLW     0x7C
;     MOVWF     T2PR          ;T2PR = 124
;     
;     
;     BANKSEL   T2TMR 
;     MOVLW     0x00
;     MOVWF     T2TMR         ;TMR2 0
;     
;     
;     BANKSEL   PIR4
;     BCF  PIR4,TMR2IF         ; Clearing IF flag.
;     
;     BANKSEL   T2CON   
;     MOVLW     0xC0
;     MOVWF     T2CON         ; (prescaller)T2CKPS 1:16;  (postscaller)T2OUTPS 1:1;  TMR2ON on;
;     
;     
;     
;                             ;PWM Initialize.
;     BANKSEL   CCP1CON
;     MOVLW     0x8C
;     MOVWF     CCP1CON        ;MODE PWM; EN enabled; CCP1FMT right_aligned;
;     
;     
;     
;     
;     BANKSEL   CCPR1H  
;     MOVLW     0x01
;     MOVWF     CCPR1H         ;RH 1;
;     
;     
;     
;     
;     
;     BANKSEL   CCPR1L  
;     MOVLW     0xF3
;     MOVWF     CCPR1L        ;RL 243; 
;     
;     
;     
;     BANKSEL   CCPTMRS0
;     BSF CCPTMRS0,C1TSEL0
     
  


next:
	
;   call PWM
   call UP
   call DOWN

   GOTO next     

    
PWM:
;0x30 - Duty 97%; 0x01 - Duty < 2%
;-------------------------------------------------------
	clrf	Mode
;-----------------------UP Count--------
UP:	
	incf	Mode,f
	movf	Mode,w
	call	Update
	movlw   0x30
	subwf	Mode,w
	btfss	STATUS,Z
	goto	UP

;;-----------------------DOWN Count--------
DOWN:
	decf	Mode,f
	movf	Mode,w
	call	Update
	movlw   0x00
	subwf	Mode,w
	btfss	STATUS,Z
	goto	DOWN	
	goto    PWM
	
;;------------------------------------------------------------	
Update: 
        
        BANKSEL   TRISC
        MOVLW     0x8C
        MOVWF     TRISC
	BCF INTCON, GIE		;Disable interrupts, clear T0IF
	
	
	BANKSEL   CCPR1H  
        MOVLW     0x01
        MOVWF     CCPR1H         ;RH 1;
     
     
	
        BANKSEL CCPR1L
	MOVLW     0xF3
	MOVWF	CCPR1L		;Duty Cycle  
	
	BANKSEL CCP1CON
	MOVLW	0x8C		
	MOVWF	CCP1CON		
	
	
	BANKSEL	T2PR		;Bank1
	MOVLW	0x7C     ;0x09	;T2Pr 124
	MOVWF	T2PR
	
	
	BANKSEL PIE4
	CLRF	PIE4		;Disable peripheral interrupts
	
	
	BANKSEL PIR3
	CLRF	PIR3
	
	
	
	BANKSEL   CCPTMRS0
        BSF CCPTMRS0,C1TSEL0
     
     
	BSF	T2CON,TMR2ON	;Timer2 starts to increment
        return


	
    

    END
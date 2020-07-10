

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






;------------------------------------------------------------------------------
; RESET VECTOR
;------------------------------------------------------------------------------

RESET_VECT  CODE    0x0000      ; processor reset vector
    GOTO    START             ; When using debug header, first inst.
                              ; may be passed over by ICD2.  

		      
transmit
WaitTx:
    BANKSEL PIR3
    btfss PIR3,TX1IF 
    GOTO WaitTx
    
    BANKSEL TX1REG
    movwf TX1REG
   
    RETURN
;------------------------------------------------------------------------------
; MAIN PROGRAM
;------------------------------------------------------------------------------
START
   
    
    BANKSEL LATC
    MOVLW 0x00
    MOVWF LATC   ;port c pin rc6 input and rc7 output
    
    BANKSEL TRISC
    MOVLW 0x9F
    MOVWF TRISC  ;port c pin rc6 input and rc7 output
    
    BANKSEL ANSELC
    MOVLW 0x5C
    MOVWF ANSELC   ;analoh and digital pins configuer
    
    
    BANKSEL RX1PPS   
    MOVLW 0x17
    MOVWF RX1PPS     ;RC7PPS SET
    
    BANKSEL RC6PPS
    MOVLW 0x0D
    MOVWF RC6PPS     ;RC6PPS SET
       
    BANKSEL OSCCON1
    MOVLW 0x62
    MOVWF OSCCON1     ;occilator init..
    
    BANKSEL  OSCCON3
    MOVLW    0x00
    MOVWF    OSCCON3
    
    BANKSEL  OSCEN    
    MOVLW    0x00
    MOVWF    OSCEN    ; LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled;
     
    BANKSEL  OSCFRQ
    MOVLW    0x06
    MOVWF    OSCFRQ   ; HFFRQ 32_MHz;
    
    BANKSEL  OSCSTAT
    MOVLW    0x00
    MOVWF    OSCSTAT  ; MFOR not ready; 
    
    BANKSEL  OSCTUNE
    MOVLW    0x00
    MOVWF    OSCTUNE   ; TUN 0; 
    
    BANKSEL  ACTCON
    MOVLW    0x00      
    MOVWF    ACTCON   ;  ACTUD enabled; ACTEN disabled;
     
    BANKSEL  BAUD1CON
    MOVLW    0x08      
    MOVWF    BAUD1CON  ;boud1con reg.
    
    BANKSEL  RC1STA  
    MOVLW    0x90           
    MOVWF    RC1STA   ;SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled;
    
    BANKSEL  TX1STA 
    MOVLW    0x24          
    MOVWF    TX1STA   ;TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave;
    
    BANKSEL  SP1BRGL  
    MOVLW    0xCF         ; 0xCF          
    MOVWF    SP1BRGL  ; SP1BRGL 207; boudrate 9600 freq 8000000
    
    BANKSEL  SP1BRGH
    MOVLW    0x00      
    MOVWF    SP1BRGH  ;SP1BRGH 0;

 

trigger_loop:
  
  
   
   
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
    
    movlw 0x0d  ; carriage return
    call transmit
    
    movlw 0x0a  ; line break
    call transmit
   
    GOTO trigger_loop   
 
    
    END
    
    
    

    
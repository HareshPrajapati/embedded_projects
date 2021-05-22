/* ===========================================================================
 * Project  :   BTZGDRtx
 * Version  :   1.0
  
 * File     :   BTZGDRtx.h
 * Author   :   Ido
 * Created on July 28, 2020, 3:32 PM
   
    Global R&D ltd. 04_9592201    ;  www.global_rd.com
    Eli Jacob 054_8010330         ;  eli@global_rd.com
 ===========================================================================*/

#ifndef _BTZGDRtx_H_
#define	_BTZGDRtx_H_

// <editor-fold defaultstate="collapsed" desc="General">

#define _XTAL_FREQ                  16000000
#define RF_TRANSMITTER_MASTER_MODE  0

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Standard includes">

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Keys defination">

#define BUTTON_UPDATE_RATE_MS               (50)
#define BUTTON_COUNT                        (4)

#define BTN1_GetValue()                     (PORTAbits.RA2) 
#define BTN2_GetValue()                     (PORTAbits.RA5)
#define BTN3_GetValue()                     (PORTAbits.RA1)
#define BTN4_GetValue()                     (PORTAbits.RA0)  

#define BUTTON12_PRESSED_SYMBOL                 (0xAA)
#define BUTTON34_PRESSED_SYMBOL                 (0xAB)

typedef enum {
    BUTTON_1 = 0xA1,
    BUTTON_2,
    BUTTON_3,
    BUTTON_4
} button_id_t;

typedef struct __attribute__((packed)){
    bool event;
    button_id_t id;
    uint32_t pressed_time;
    uint32_t duration;
} button_t;

void button_init(void);
void button_task(void);

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Milli-second tick(millis)">

uint32_t millis(void);
void delay_ms(uint32_t ms);

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="EUSART">

void usart_write(uint8_t data);

// </editor-fold>   

// <editor-fold defaultstate="collapsed" desc="Encryption">

static const uint8_t encrypt_key = 10;
void rf_encrypt_and_send(uint8_t data);
// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="RF-443Mhz readio">

// Used same baud on USART-2
#define RF_TRANSMITTER_BAUD              4800

// RF packet buffer defines 
#define RF_PACKET_BUFFER_SIZE           (59)
#define RF_PACKET_IDENTITY_SIZE         (05)
#define RF_PACKET_TRANSMITTER_ID_SIZE   (04)
#define RF_PACKET_TRANSMITTER_CRC_SIZE  (01)
#define RF_PACKET_COUNTER_SIZE          (16)
#define RF_PACKET_PAYLOAD_COUNT         ((RF_PACKET_BUFFER_SIZE-(RF_PACKET_IDENTITY_SIZE+RF_PACKET_TRANSMITTER_ID_SIZE+RF_PACKET_TRANSMITTER_CRC_SIZE+1)))

// Change your transmitter ID (4 byte) here...  
#if (RF_TRANSMITTER_MASTER_MODE==1)
#define RF_TRANSMITTER_ID               0x55665566
#if(RF_TRANSMITTER_ID!=0x55665566)
#error "Invalid master transmitter id... it should be :0x55665566."
#else
#warning "Transmitter mode is: Master" 
#endif
#else
#define RF_TRANSMITTER_ID               0x22554411
#warning "Transmitter mode is: Normal"
#endif



int rf_send_packet(uint8_t *data, uint8_t len);

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Counter">

/**
  Section: Macro Declarations
 */

#define WRITE_FLASH_BLOCKSIZE       16
#define ERASE_FLASH_BLOCKSIZE       16
#define END_FLASH                   0x800

// 16 Byte counter value...
typedef struct __attribute__((packed)){
    uint16_t word0;
    uint16_t word1;
    uint16_t word2;
    uint16_t word3;
    uint16_t word4;
    uint16_t word5;
    uint16_t word6;
    uint16_t word7;
} counter_t;

#define CONSTANT_INC_BY_UINT16_VAL  5
#define COUNTER_LOCATION_IN_FLASH   (0x7E0)

void counter_inc_by_n(counter_t *ctx, uint16_t val);
void counter_read(counter_t *ctx);
void counter_save(counter_t *ctx);

// </editor-fold> 

#endif
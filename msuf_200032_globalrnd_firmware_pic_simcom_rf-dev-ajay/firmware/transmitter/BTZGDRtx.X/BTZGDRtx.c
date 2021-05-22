#include <xc.h>
#include "BTZGDRtx.h"

const uint8_t __attribute__((address(COUNTER_LOCATION_IN_FLASH))) reserve_location[32] = {0};

static counter_t counter;
static volatile uint8_t timer0ReloadVal;
static volatile uint32_t tick = 0;
static button_t buttons[BUTTON_COUNT];
static uint8_t button_packet[5] = {0};
static uint32_t last_tick = 0;
static uint32_t combo0_pressed = 0;
static uint32_t combo1_pressed = 0;


// <editor-fold defaultstate="collapsed" desc="Configuration bits and Initialization">

// Configuration bits: selected in the GUI

// CONFIG1
#pragma config FOSC = INTOSC    // ->INTOSC oscillator; I/O function on CLKIN pin
#pragma config WDTE = OFF    // Watchdog Timer Enable->WDT disabled
#pragma config PWRTE = OFF    // Power-up Timer Enable->PWRT disabled
#pragma config MCLRE = ON    // MCLR Pin Function Select->MCLR/VPP pin function is MCLR
#pragma config CP = OFF    // Flash Program Memory Code Protection->Program memory code protection is disabled
#pragma config BOREN = ON    // Brown-out Reset Enable->Brown-out Reset enabled
#pragma config CLKOUTEN = OFF    // Clock Out Enable->CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin

// CONFIG2
#pragma config WRT = OFF    // Flash Memory Self-Write Protection->Write protection off
#pragma config PLLEN = OFF    // PLL Enable->4x PLL disabled
#pragma config STVREN = ON    // Stack Overflow/Underflow Reset Enable->Stack Overflow or Underflow will cause a Reset
#pragma config BORV = LO    // Brown-out Reset Voltage Selection->Brown-out Reset Voltage (Vbor), low trip point selected.
#pragma config LPBOREN = OFF    // Low Power Brown-out Reset enable bit->LPBOR is disabled
#pragma config LVP = ON    // Low-Voltage Programming Enable->Low-voltage programming enabled

void system_initialize(void) {

    // SCS INTOSC; SPLLEN disabled; IRCF 16MHz_HF; 
    OSCCON = 0x7A;
    // TUN 0; 
    OSCTUNE = 0x00;
    // SBOREN disabled; BORFS disabled; 
    BORCON = 0x00;

    // WDTPS 1:65536; SWDTEN OFF; 
    WDTCON = 0x16;

    // Port A----
    LATA = 0b00000000; // Clear all pins.
    TRISA = 0b00101111; // PA0, PA1, PA2, PA5 input for keys.  RA4 - UART0 Tx
    ANSELA = 0b00000000; // Set all pin as digital.

    WPUA = 0x00;
    OPTION_REGbits.nWPUEN = 1; // Disabled

    // ODx registers
    ODCONA = 0x00;

    // SLRCONx registers
    SLRCONA = 0x37;

    // INLVLx registers
    INLVLA = 0x3F;

    // APFCONx registers
    APFCON = 0x84;

    // TMR0 @1ms ------
    // PSA assigned; PS 1:32; TMRSE Increment_hi_lo; mask the nWPUEN and INTEDG bits
    OPTION_REG = (uint8_t) ((OPTION_REG & 0xC0) | (0xD4 & 0x3F));

    // TMR0 131; 
    TMR0 = 0x83;

    // Load the TMR value to reload variable
    timer0ReloadVal = 131;

    // Clear Interrupt flag before enabling the interrupt
    INTCONbits.TMR0IF = 0;

    // Enabling TMR0 interrupt
    INTCONbits.TMR0IE = 1;

    // EUSART @4800 -----
    // ABDOVF no_overflow; SCKP Non-Inverted; BRG16 16bit_generator; WUE disabled; ABDEN disabled; 
    BAUDCON = 0x08;

    // SPEN enabled; RX9 8-bit; CREN disabled; ADDEN disabled; SREN disabled; 
    RCSTA = 0x80;

    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave; 
    TXSTA = 0x24;

    // SPBRGL 64; 
    SPBRGL = 0x40;

    // SPBRGH 3; 
    SPBRGH = 0x03;
}

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Milli-second tick(millis)">

/**
 * Tick updater on interrupt.
 */
static void timer_interrupt_handler(void) {
    tick++;
}

/**
 * Get total ticks from system wake in millis.
 * @return total ticks.
 */
uint32_t millis() {
    return tick;
}

/**
 * Generate delay.
 * @param ms time in milli-seconds to generate delay.
 */
void delay_ms(uint32_t ms) {
    uint32_t wait = millis() + ms;
    while (millis() < wait);
}

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Keys defination">

/**
 * Get button state pressed or not.
 * @param btn index of button.
 * @return true if pressed.
 */
bool button_get_state(uint8_t btn) {
    bool state = 0;
    switch (btn) {
        case 0:
        {
            state = BTN1_GetValue();
        }
            break;
        case 1:
        {
            state = BTN2_GetValue();
        }
            break;
        case 2:
        {
            state = BTN3_GetValue();
        }
            break;
        case 3:
        {
            state = BTN4_GetValue();
        }
            break;
        default:break;
    }
    return state;
}

/**
 * Initialize button with default IDs.
 */
void button_init(void) {

    memset(&buttons[0], 0, sizeof (button_t));
    memset(&buttons[1], 0, sizeof (button_t));
    memset(&buttons[2], 0, sizeof (button_t));
    memset(&buttons[3], 0, sizeof (button_t));

    buttons[0].id = BUTTON_1;
    buttons[1].id = BUTTON_2;
    buttons[2].id = BUTTON_3;
    buttons[3].id = BUTTON_4;

    last_tick = 0;
}

/**
 * Keep check for button event and send RF data.
 */
void button_task(void) {

    if ((uint32_t) (millis() - last_tick) >= 50) {
        for (uint8_t i = 0; i < 4; i++) {
            
            if(i == 1 ){
                continue;
            }

            if (button_get_state(i) == 1) {

                button_packet[0] = buttons[i].id;

                if (buttons[i].pressed_time == 0) {
                    buttons[i].pressed_time = millis();
                }

            } else {
                buttons[i].pressed_time = 0;
                buttons[i].duration = 0;
            }
        }
        if (buttons[0].pressed_time > 0 && buttons[1].pressed_time > 0) {
            if (combo0_pressed == 0) {
                combo0_pressed = millis();
            }
            uint32_t combo0_pressed_now = millis() - combo0_pressed;
            button_packet[0] = BUTTON12_PRESSED_SYMBOL;
            memcpy(&button_packet[1], &combo0_pressed_now, sizeof (combo0_pressed_now));
            rf_send_packet(button_packet, 5);
        } else if (buttons[0].pressed_time > 0) {
            combo0_pressed = 0;
            button_packet[0] = buttons[0].id;
            buttons[0].duration = millis() - buttons[0].pressed_time;
            memcpy(&button_packet[1], &buttons[0].duration, sizeof (buttons[0].duration));
            rf_send_packet(button_packet, 5);
        } else if (buttons[1].pressed_time > 0) {
            combo0_pressed = 0;
            button_packet[0] = buttons[1].id;
            buttons[1].duration = millis() - buttons[1].pressed_time;
            memcpy(&button_packet[1], &buttons[1].duration, sizeof (buttons[1].duration));
            rf_send_packet(button_packet, 5);
        }

        if (buttons[2].pressed_time > 0 && buttons[3].pressed_time > 0) {
            if (combo1_pressed == 0) {
                combo1_pressed = millis();
            }
            uint32_t combo1_pressed_now = millis() - combo1_pressed;
            button_packet[0] = BUTTON34_PRESSED_SYMBOL;
            memcpy(&button_packet[1], &combo1_pressed_now, sizeof (combo1_pressed_now));
            rf_send_packet(button_packet, 5);
        } else if (buttons[2].pressed_time > 0) {
            combo1_pressed = 0;
            button_packet[0] = buttons[2].id;
            buttons[2].duration = millis() - buttons[2].pressed_time;
            memcpy(&button_packet[1], &buttons[2].duration, sizeof (buttons[2].duration));
            rf_send_packet(button_packet, 5);
        } else if (buttons[3].pressed_time > 0) {
            combo1_pressed = 0;
            button_packet[0] = buttons[3].id;
            buttons[3].duration = millis() - buttons[3].pressed_time;
            memcpy(&button_packet[1], &buttons[3].duration, sizeof (buttons[3].duration));
            rf_send_packet(button_packet, 5);
        }
        last_tick = millis();
    }

}
// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="EUSART">

void usart_write(uint8_t data) {
    while (0 == PIR1bits.TXIF) {
    }

    TXREG = data; // Write the data byte to the USART.
}

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Encryption">

const uint8_t bit_convt_4to6[] = {
    0x2d, 0x2e, 0x33, 0x35, 0x36, 0x29, 0x2a, 0x3c,
    0x23, 0x25, 0x26, 0x39, 0x3a, 0x32, 0x3e, 0x3f
};

void rf_encrypt_and_send(uint8_t data) {
    uint8_t enc[2];
    uint8_t mbyte = data^encrypt_key;
    enc[0] = bit_convt_4to6[mbyte >> 4];
    enc[1] = bit_convt_4to6[mbyte & 0x0f];
    usart_write(enc[0]);
    usart_write(enc[1]);
}

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="RF-443Mhz readio">

// send buffer.

/**
 * To calculate packet checksum.
 * @param buffer data pointer to calculate checksum.
 * @param len total bytes to compute checksum.
 * @return  calculated checksum
 */

uint8_t crc8(uint8_t *buffer, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc += buffer[i];
    }
    crc = (~crc) + 1;
    return crc;
}

/**
 * Used to transmit data via RF
 * @param data pointer to the buffer to send.
 * @param len total bytes to send. Must be <=RF_PACKET_PAYLOAD_COUNT.
 * @return true if provided len is in range else -1.
 */
int rf_send_packet(uint8_t *data, uint8_t len) {
    uint8_t index = 0;
    uint32_t tx_id = RF_TRANSMITTER_ID;

    // Fill buffer with 0xf0
    uint8_t plain_msg[31];
    memset(plain_msg, 0xf0, sizeof (plain_msg));

    index = 0;

    memcpy(&plain_msg[index], &counter, sizeof (counter_t));
    index += sizeof (counter);

    plain_msg[index++] = (uint8_t) ((tx_id >> 0)&0xFF);
    plain_msg[index++] = (uint8_t) rand();
    plain_msg[index++] = (uint8_t) ((tx_id >> 8)&0xFF);
    plain_msg[index++] = (uint8_t) rand();
    plain_msg[index++] = (uint8_t) ((tx_id >> 16)&0xFF);
    plain_msg[index++] = (uint8_t) rand();
    plain_msg[index++] = (uint8_t) ((tx_id >> 24)&0xFF);
    plain_msg[index++] = (uint8_t) rand();

    plain_msg[index++] = len;

    memcpy(&plain_msg[index], data, len);
    index += len;

    plain_msg[sizeof (plain_msg) - 1] = crc8(plain_msg, sizeof (plain_msg) - 1);

    uint32_t random_nm0 = rand();
    uint32_t random_nm1 = rand();
    uint32_t random_nm2 = rand();

    usart_write(0xF0);
    usart_write(random_nm0 >> 0);
    usart_write(random_nm0 >> 8);
    usart_write(random_nm0 >> 16);

    usart_write(0xFE);
    usart_write(random_nm1 >> 0);
    usart_write(random_nm1 >> 8);
    usart_write(random_nm1 >> 16);

    usart_write(0x5A);
    usart_write(random_nm2 >> 0);
    usart_write(random_nm2 >> 8);
    usart_write((random_nm0 >> 0)+(random_nm0 >> 8)+(random_nm0 >> 16)+
            (random_nm1 >> 0)+(random_nm1 >> 8)+(random_nm1 >> 16)+
            (random_nm2 >> 0)+(random_nm2 >> 8));


    // Send RF-packet
    for (uint8_t i = 0; i < sizeof (plain_msg); i++) {
        rf_encrypt_and_send(plain_msg[i]);
    }
    return 1;

}

// </editor-fold>   

// <editor-fold defaultstate="collapsed" desc="Interrupt">

void __interrupt() INTERRUPT_InterruptManager(void) {
    // interrupt handler
    if (INTCONbits.TMR0IE == 1 && INTCONbits.TMR0IF == 1) {

        // Clear the TMR0 interrupt flag
        INTCONbits.TMR0IF = 0;

        TMR0 = timer0ReloadVal;

        timer_interrupt_handler();

    } else if (INTCONbits.IOCIE == 1 && INTCONbits.IOCIF == 1) {
        // Pin change interrupt
    } else {
        //Unhandled Interrupt
    }
}

// </editor-fold>  

// <editor-fold defaultstate="collapsed" desc="Counter">

/**
 * Increment counter by 1.
 * @param ctx counter storage structure.
 * @return 
 */
static void counter_inc(counter_t *ctx) {
    ctx->word0++;
    if (ctx->word0 == 0) {
        ctx->word1++;
        if (ctx->word1 == 0) {
            ctx->word2++;
            if (ctx->word2 == 0) {
                ctx->word3++;
                if (ctx->word3 == 0) {
                    ctx->word4++;
                    if (ctx->word4 == 0) {
                        ctx->word5++;
                        if (ctx->word5 == 0) {
                            ctx->word6++;
                            if (ctx->word6 == 0) {
                                ctx->word7++;
                                if (ctx->word7 == 0) {
                                    ctx->word0 = 0;
                                    ctx->word1 = 0;
                                    ctx->word2 = 0;
                                    ctx->word3 = 0;
                                    ctx->word4 = 0;
                                    ctx->word5 = 0;
                                    ctx->word6 = 0;
                                    ctx->word7 = 0;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/**
 * Increment counter value by val.
 * @param ctx   pointer to counter.
 * @param val   by val.
 */
void counter_inc_by_n(counter_t *ctx, uint16_t val) {
    for (uint16_t i = 0; i < val; i++)
        counter_inc(ctx);
}

/**
  @Summary
    Reads a word from Flash

  @Description
    This routine reads a word from given Flash address

  @Preconditions
    None

  @Param
    flashAddr - Flash program memory location from which data has to be read

  @Returns
    Data word read from given Flash address

  @Example
    <code>
    uint16_t    readWord;
    uint16_t    flashAddr = 0x01C0;

    readWord = FLASH_ReadWord(flashAddr);
    </code>
 */
static uint16_t FLASH_ReadWord(uint16_t flashAddr) {
    uint8_t GIEBitValue = INTCONbits.GIE; // Save interrupt enable

    INTCONbits.GIE = 0; // Disable interrupts
    PMADRL = (flashAddr & 0x00FF);
    PMADRH = ((flashAddr & 0xFF00) >> 8);

    PMCON1bits.CFGS = 0; // Deselect Configuration space
    PMCON1bits.RD = 1; // Initiate Read
    NOP();
    NOP();
    INTCONbits.GIE = GIEBitValue; // Restore interrupt enable

    return ((uint16_t) ((PMDATH << 8) | PMDATL));
}

/**
  @Summary
    Erases complete Flash program memory block

  @Description
    This routine erases complete Flash program memory block

  @Preconditions
    None

  @Param
    startAddr - A valid block starting address in Flash program memory

  @Returns
    None

  @Example
    <code>
    uint16_t    flashBlockStartAddr = 0x01C0;

    FLASH_EraseBlock(flashBlockStartAddr);
    </code>
 */
void FLASH_EraseBlock(uint16_t startAddr) {
    uint8_t GIEBitValue = INTCONbits.GIE; // Save interrupt enable


    INTCONbits.GIE = 0; // Disable interrupts
    // Load lower 8 bits of erase address boundary
    PMADRL = (startAddr & 0xFF);
    // Load upper 6 bits of erase address boundary
    PMADRH = ((startAddr & 0xFF00) >> 8);

    // Block erase sequence
    PMCON1bits.CFGS = 0; // Deselect Configuration space
    PMCON1bits.FREE = 1; // Specify an erase operation
    PMCON1bits.WREN = 1; // Allows erase cycles

    // Start of required sequence to initiate erase
    PMCON2 = 0x55;
    PMCON2 = 0xAA;
    PMCON1bits.WR = 1; // Set WR bit to begin erase
    NOP();
    NOP();

    PMCON1bits.WREN = 0; // Disable writes
    INTCONbits.GIE = GIEBitValue; // Restore interrupt enable
}

/**
  @Summary
    Writes data to complete block of Flash

  @Description
    This routine writes data words to complete block in Flash program memory

  @Preconditions
    None

  @Param
    writeAddr         - A valid block starting address in Flash
 *flashWordArray   - Pointer to an array of size 'WRITE_FLASH_BLOCKSIZE' at least

  @Returns
    -1, if the given address is not a valid block starting address of Flash
    0, in case of valid block starting address

  @Example
    <code>
    #define FLASH_ROW_ADDRESS     0x01C0

    uint16_t wrBlockData[] =
    {
        0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007,
        0x0008, 0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x000D, 0x000F,
        0x0010, 0x0011, 0x0012, 0x0013, 0x0014, 0x0015, 0x0016, 0x0017,
        0x0018, 0x0019, 0x001A, 0x001B, 0x001C, 0x001D, 0x001E, 0x001F
    }

    // write to Flash memory block
    FLASH_WriteBlock((uint16_t)FLASH_ROW_ADDRESS, (uint16_t*)wrBlockData);
    </code>
 */
int8_t FLASH_WriteBlock(uint16_t writeAddr, uint16_t *flashWordArray) {
    uint16_t blockStartAddr = (uint16_t) (writeAddr & ((END_FLASH - 1) ^ (ERASE_FLASH_BLOCKSIZE - 1)));
    uint8_t GIEBitValue = INTCONbits.GIE; // Save interrupt enable
    uint8_t i;

    // Flash write must start at the beginning of a row
    if (writeAddr != blockStartAddr) {
        return -1;
    }

    INTCONbits.GIE = 0; // Disable interrupts

    // Block erase sequence
    FLASH_EraseBlock(writeAddr);

    // Block write sequence
    PMCON1bits.CFGS = 0; // Deselect Configuration space
    PMCON1bits.WREN = 1; // Enable wrties
    PMCON1bits.LWLO = 1; // Only load write latches

    for (i = 0; i < WRITE_FLASH_BLOCKSIZE; i++) {
        // Load lower 8 bits of write address
        PMADRL = (writeAddr & 0xFF);
        // Load upper 6 bits of write address
        PMADRH = ((writeAddr & 0xFF00) >> 8);

        // Load data in current address
        PMDATL = flashWordArray[i];
        PMDATH = ((flashWordArray[i] & 0xFF00) >> 8);

        if (i == (WRITE_FLASH_BLOCKSIZE - 1)) {
            // Start Flash program memory write
            PMCON1bits.LWLO = 0;
        }

        PMCON2 = 0x55;
        PMCON2 = 0xAA;
        PMCON1bits.WR = 1;
        NOP();
        NOP();

        writeAddr++;
    }

    PMCON1bits.WREN = 0; // Disable writes
    INTCONbits.GIE = GIEBitValue; // Restore interrupt enable

    return 0;
}

void counter_read(counter_t *ctx) {
    uint16_t temp[8];
    memset(temp, 0x00, sizeof (temp));

    for (uint8_t i = 0; i < 8; i++) {
        temp[i] = FLASH_ReadWord(COUNTER_LOCATION_IN_FLASH + (2 * i) + 1) & 0xFF;
        temp[i] = temp[i] << 8;
        temp[i] |= FLASH_ReadWord(COUNTER_LOCATION_IN_FLASH + (2 * i)) & 0xFF;
    }

    memcpy(ctx, temp, sizeof (counter_t));
}

void counter_save(counter_t *ctx) {
    uint16_t block[32];
    uint16_t *temp = (uint16_t*) ctx;
    uint8_t i = 0;
    memset(block, 0x00, sizeof (block));

    for (i = 0; i < 8; i++) {
        block[2 * i] = (temp[i] >> 0)&0xFF;
        block[(2 * i) + 1] = (temp[i] >> 8)&0xFF;
    }

    FLASH_WriteBlock(COUNTER_LOCATION_IN_FLASH, block);
}

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Main">

void main() {
    uint32_t seed = 0;

    system_initialize();
    
    memset(&counter, 0, sizeof (counter_t));
    counter_read(&counter);
    counter_inc_by_n(&counter, CONSTANT_INC_BY_UINT16_VAL);
    counter_save(&counter);

    button_init();
    // Enable interrupt
    INTCONbits.GIE = 1;

    seed = counter.word0 + counter.word1 + counter.word2 + counter.word3 +
            counter.word4 + counter.word5 + counter.word6 + counter.word7 +
            millis();
    srand((unsigned) seed);

    last_tick = 0;

    while (1) {
        button_task();
    }
}

// </editor-fold>  

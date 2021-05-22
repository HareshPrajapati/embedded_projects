#include "BTZGDRrv.h"

static uint8_t recv_counter_value[RF_PACKET_COUNTER_SIZE];

// <editor-fold defaultstate="collapsed" desc="Counter">

/**
 * Increment counter by 1.
 * @param ctx counter storage structure.
 * @return 
 */
void counter_inc(counter_t *ctx) {
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
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="System config">
static system_config_t system_cfg;
static bool system_cfg_update = 0;
static uint32_t system_cfg_last_update = 0;

static uint32_t user_blocked_timer = 0;

uint32_t last_blink_red = 0;
uint32_t last_blink_green = 0;
uint32_t last_blink_yellow = 0;

/**
 * Load system config data from eeprom and validate if needed.
 */
void system_config_begin(void) {


    eeprom_read_nbytes(0, (uint8_t*) & system_cfg, sizeof (system_cfg));

    DEBUG_MSG("> Config New vs Old %d %d\r\n", sizeof (system_cfg), system_cfg.cfg_size);

    // Reset if size issue.
    if ((int32_t) system_cfg.cfg_size != sizeof (system_cfg)) {
        DEBUG_MSG("> Config mis-match...\r\n");
        DEBUG_MSG("> Resetting config..\r\n");
        memset(&system_cfg, 0xFF, sizeof (system_cfg));

        system_cfg.cfg_size = sizeof (system_cfg);
        eeprom_write_nbytes(0, (uint8_t*) & system_cfg, sizeof (system_cfg));
        delay_ms(10);
        eeprom_read_nbytes(0, (uint8_t*) & system_cfg, sizeof (system_cfg));
        DEBUG_MSG("> Resetting Done..\r\n");
    }

    // Verify pulse config.
    uint8_t n = 0;
    for (n = 0; n < CONFIG_RELAY_MAX; n++) {
#if (CONFIG_USE_LIMITED_FUNC==1)
        if (system_cfg.pulse_width[n].pulse_width.val != 3000) {
            system_cfg.pulse_width[n].pulse_width.val = CONFIG_RELAY_PULSE_DEFAULT_VAL;
            system_cfg_update = true;
        }
#else
#endif
        if (system_cfg.pulse_width[n].pulse_width.val == CONFIG_PARAM_INVALID) {
            system_cfg.pulse_width[n].pulse_width.val = CONFIG_RELAY_PULSE_DEFAULT_VAL;
            system_cfg_update = true;
        }
    }

    // Verify relay trigger mode and update if required..
    for (n = 0; n < CONFIG_RELAY_MAX; n++) {

#if (CONFIG_USE_LIMITED_FUNC==1)
        if ((system_cfg.relay_mode.mode[n] != RLY_PULSE_MODE)) {
            system_cfg.relay_mode.mode[n] = RLY_PULSE_MODE;
            system_cfg_update = true;
        }
#else
        if ((system_cfg.relay_mode.mode[n] < RLY_MOMENTARY_MODE) || (system_cfg.relay_mode.mode[n] > RLY_DISABLED)) {
            system_cfg.relay_mode.mode[n] = RLY_PULSE_MODE;
            system_cfg_update = true;
        }
#endif

    }
    if (system_cfg.transmitter[4].id != RF_MASTER_TRANSMITTER_ID) {
        system_cfg.transmitter[4].id = RF_MASTER_TRANSMITTER_ID;
        system_cfg_update = true;
    }

    if (system_cfg.system_block_timeout_ms > CONFIG_LOGIN_BLOCK_TIMEOUT) {
        system_cfg.system_block_timeout_ms = 0;
        system_cfg_update = true;
    }

    // Update if invalid
    if (system_cfg_update) {
        eeprom_write_nbytes(0, (uint8_t*) & system_cfg, sizeof (system_cfg));
        system_cfg_update = false;
    }
}

/**
 * Update system config if pending after timeout of 2 sec.
 */
void system_config_update_task(void) {
    if (system_cfg_update) {
        if (millis() - system_cfg_last_update > 2000) {
            eeprom_write_nbytes(0, (uint8_t*) & system_cfg, sizeof (system_cfg));
            system_cfg_update = false;
        }
    }
}

uint32_t system_get_block_timeout(void) {
    return system_cfg.system_block_timeout_ms;
}

void system_set_block_timeout(uint32_t tout) {
    system_cfg.system_block_timeout_ms = tout;
    eeprom_write_nbytes(0, (uint8_t*) & system_cfg, sizeof (system_cfg));
}

/**
 * Set pin/password to access relay.
 * @param pin 4 byte code.
 */
void system_set_user_pin(uint32_t pin) {
    system_cfg.user_pin = pin;
    system_cfg_last_update = millis();
    system_cfg_update = true;
}

/**
 * Get system user pin.
 * If value is 0xFFFFFFFF than system not registered any pin.
 * @return pin.
 */
uint32_t system_get_user_pin() {
    return system_cfg.user_pin;
}

/**
 * Set pin/password to limit access 
 * @param pin 4 byte code.
 */
void system_set_technician_pin(uint32_t pin) {
    system_cfg.technician_pin = pin;
    system_cfg_last_update = millis();
    system_cfg_update = true;
}

/**
 * Get system technician pin.
 * If value is 0xFFFFFFFF than system not registered any pin.
 * @return pin.
 */
uint32_t system_get_technician_pin() {
    return system_cfg.technician_pin;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Decryption">
const uint8_t bit_convt_4to6[] = {
    0x2d, 0x2e, 0x33, 0x35, 0x36, 0x29, 0x2a, 0x3c,
    0x23, 0x25, 0x26, 0x39, 0x3a, 0x32, 0x3e, 0x3f
};

/**
 * To decrypt message with key and table.
 * 
 * @param plain_msg pointer to store decrypted message.
 * @param cypher_msg pointer to encrypted message to decrypt.
 * @param plain_len decrypt message buffer size.
 * @param cypher_len encrypted message length.
 * @return true if buffer enough to hold decrypted message.
 */
bool decrypt(uint8_t *plain_msg, uint8_t *cypher_msg, uint8_t plain_len, uint8_t cypher_len) {
    if (cypher_len >= (plain_len * 2)) {
        for (uint8_t i = 0; i < plain_len; i++) {
            *(plain_msg + i) = (uint8_t) bit_convt_6to4(*(cypher_msg + ((2 * i))));
            *(plain_msg + i) = (uint8_t) ((*(plain_msg + i)) << 4);
            *(plain_msg + i) |= (uint8_t) bit_convt_6to4(*(cypher_msg + ((2 * i) + 1)));
            *(plain_msg + i) ^= decrypt_key;
        }
        return true;
    }
    return false;
}

/**
 * Convert 6bit to 4 bit data.
 * @param symbol 6bit symbol to convert.
 * @return 4 bit symbol or nibble.
 */
int bit_convt_6to4(uint8_t symbol) {
    uint8_t i;

    // Linear search :
    for (i = 0; i < 16; i++)
        if (symbol == bit_convt_4to6[i]) return i;
    return -1; // Not found
}

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Device config and intialization">
// Configuration bits: selected in the GUI

// CONFIG1L
#pragma config FEXTOSC = OFF    // External Oscillator Selection->Oscillator not enabled
#pragma config RSTOSC = HFINTOSC_1MHZ    // Reset Oscillator Selection->HFINTOSC with HFFRQ = 4 MHz and CDIV = 4:1

// CONFIG1H
#pragma config CLKOUTEN = OFF    // Clock out Enable bit->CLKOUT function is disabled
#pragma config PR1WAY = ON    // PRLOCKED One-Way Set Enable bit->PRLOCK bit can be cleared and set only once
#pragma config CSWEN = ON    // Clock Switch Enable bit->Writing to NOSC and NDIV is allowed
#pragma config FCMEN = ON    // Fail-Safe Clock Monitor Enable bit->Fail-Safe Clock Monitor enabled

// CONFIG2L
#pragma config MCLRE = EXTMCLR    // MCLR Enable bit->If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR 
#pragma config PWRTS = PWRT_OFF    // Power-up timer selection bits->PWRT is disabled
#pragma config MVECEN = OFF    // Multi-vector enable bit->Interrupt contoller does not use vector table to prioritze interrupts
#pragma config IVT1WAY = ON    // IVTLOCK bit One-way set enable bit->IVTLOCK bit can be cleared and set only once
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit->ULPBOR disabled
#pragma config BOREN = SBORDIS    // Brown-out Reset Enable bits->Brown-out Reset enabled , SBOREN bit is ignored

// CONFIG2H
#pragma config BORV = VBOR_2P45    // Brown-out Reset Voltage Selection bits->Brown-out Reset Voltage (VBOR) set to 2.45V
#pragma config ZCD = OFF    // ZCD Disable bit->ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON
#pragma config PPS1WAY = ON    // PPSLOCK bit One-Way Set Enable bit->PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle
#pragma config STVREN = ON    // Stack Full/Underflow Reset Enable bit->Stack full/underflow will cause Reset
#pragma config DEBUG = OFF    // Debugger Enable bit->Background debugger disabled
#pragma config XINST = OFF    // Extended Instruction Set Enable bit->Extended Instruction Set and Indexed Addressing Mode disabled

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31    // WDT Period selection bits->Divider ratio 1:65536; software control of WDTPS
#pragma config WDTE = OFF    // WDT operating mode->WDT Disabled; SWDTEN is ignored

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7    // WDT Window Select bits->window always open (100%); software control; keyed access not required
#pragma config WDTCCS = SC    // WDT input clock selector->Software Control

// CONFIG4L
#pragma config BBSIZE = BBSIZE_512    // Boot Block Size selection bits->Boot Block size is 512 words
#pragma config BBEN = OFF    // Boot Block enable bit->Boot block disabled
#pragma config SAFEN = OFF    // Storage Area Flash enable bit->SAF disabled
#pragma config WRTAPP = OFF    // Application Block write protection bit->Application Block not write protected

// CONFIG4H
#pragma config WRTB = OFF    // Configuration Register Write Protection bit->Configuration registers (300000-30000Bh) not write-protected
#pragma config WRTC = OFF    // Boot Block Write Protection bit->Boot Block (000000-0007FFh) not write-protected
#pragma config WRTD = OFF    // Data EEPROM Write Protection bit->Data EEPROM not write-protected
#pragma config WRTSAF = OFF    // SAF Write protection bit->SAF not Write Protected
#pragma config LVP = ON    // Low Voltage Programming Enable bit->Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored

// CONFIG5L
#pragma config CP = OFF    // PFM and Data EEPROM Code Protection bit->PFM and Data EEPROM code protection disabled

/**
 * Initialize system clock, timer, port, uart modules, IOC.
 */
void system_initialize(void) {

    // Disable Interrupt Priority Vectors (16CXXX Compatibility Mode)
    INTCON0bits.IPEN = 0;

    // PMD--------------------------
    // CLKRMD CLKR enabled; SYSCMD SYSCLK enabled; SCANMD SCANNER enabled; FVRMD FVR enabled; IOCMD IOC enabled; CRCMD CRC enabled; HLVDMD HLVD enabled; NVMMD NVM enabled; 
    PMD0 = 0x00;
    // NCO1MD DDS(NCO1) enabled; TMR0MD TMR0 enabled; TMR1MD TMR1 enabled; TMR4MD TMR4 enabled; TMR5MD TMR5 enabled; TMR2MD TMR2 enabled; TMR3MD TMR3 enabled; TMR6MD TMR6 enabled; 
    PMD1 = 0x00;
    // ZCDMD ZCD enabled; DACMD DAC enabled; CMP1MD CMP1 enabled; ADCMD ADC enabled; CMP2MD CMP2 enabled; 
    PMD2 = 0x00;
    // CCP2MD CCP2 enabled; CCP1MD CCP1 enabled; CCP4MD CCP4 enabled; CCP3MD CCP3 enabled; PWM6MD PWM6 enabled; PWM5MD PWM5 enabled; PWM8MD PWM8 enabled; PWM7MD PWM7 enabled; 
    PMD3 = 0x00;
    // CWG3MD CWG3 enabled; CWG2MD CWG2 enabled; CWG1MD CWG1 enabled; 
    PMD4 = 0x00;
    // U2MD UART2 enabled; U1MD UART1 enabled; SPI1MD SPI1 enabled; I2C2MD I2C2 enabled; I2C1MD I2C1 enabled; 
    PMD5 = 0x00;
    // DSMMD DSM1 enabled; CLC3MD CLC3 enabled; CLC4MD CLC4 enabled; SMT1MD SMT1 enabled; CLC1MD CLC1 enabled; CLC2MD CLC2 enabled; 
    PMD6 = 0x00;
    // DMA1MD DMA1 enabled; DMA2MD DMA2 enabled; 
    PMD7 = 0x00;

    // Port and GPIO--------------------------
    /**
    LATx registers
     */
    LATA = 0x00;
    LATB = 0x80;
    LATC = 0x00;

    /**
    TRISx registers
     */
    TRISA = 0x0F;
    TRISB = 0x3F;
    TRISC = 0xC7;

    /**
    ANSELx registers
     */
    ANSELC = 0x07;
    ANSELB = 0x70;
    ANSELA = 0xF0;

    /**
    WPUx registers
     */
    WPUE = 0x00;
    WPUB = 0x80;
    WPUA = 0x01;
    WPUC = 0x40; //< Trigger pin pull-up

    /**
    RxyI2C registers
     */
    RB1I2C = 0x00;
    RB2I2C = 0x00;
    RC3I2C = 0x00;
    RC4I2C = 0x00;

    /**
    ODx registers
     */
    ODCONA = 0x00;
    ODCONB = 0x00;
    ODCONC = 0x18;

    /**
    SLRCONx registers
     */
    SLRCONA = 0xFF;
    SLRCONB = 0xFF;
    SLRCONC = 0xFF;

    /**
    INLVLx registers
     */
    INLVLA = 0xFF;
    INLVLB = 0xFF;
    INLVLC = 0xFF;
    INLVLE = 0x08;


    /**
    IOCx registers 
     */
    //interrupt on change for group IOCAF - flag
    IOCAFbits.IOCAF0 = 0;
    //interrupt on change for group IOCAN - negative
    IOCANbits.IOCAN0 = 1;
    //interrupt on change for group IOCAP - positive
    IOCAPbits.IOCAP0 = 1;
    //interrupt on change for group IOCBF - flag
    IOCBFbits.IOCBF0 = 0;
    //interrupt on change for group IOCBF - flag
    IOCBFbits.IOCBF1 = 0;
    //interrupt on change for group IOCBF - flag
    IOCBFbits.IOCBF2 = 0;
    //interrupt on change for group IOCBF - flag
    IOCBFbits.IOCBF3 = 0;
    //interrupt on change for group IOCBN - negative
    IOCBNbits.IOCBN0 = 1;
    //interrupt on change for group IOCBN - negative
    IOCBNbits.IOCBN1 = 1;
    //interrupt on change for group IOCBN - negative
    IOCBNbits.IOCBN2 = 1;
    //interrupt on change for group IOCBN - negative
    IOCBNbits.IOCBN3 = 1;
    //interrupt on change for group IOCBP - positive
    IOCBPbits.IOCBP0 = 1;
    //interrupt on change for group IOCBP - positive
    IOCBPbits.IOCBP1 = 1;
    //interrupt on change for group IOCBP - positive
    IOCBPbits.IOCBP2 = 1;
    //interrupt on change for group IOCBP - positive
    IOCBPbits.IOCBP3 = 1;
    //interrupt on change for group IOCCF - flag
    IOCCFbits.IOCCF6 = 0;
    //interrupt on change for group IOCCN - negative
    IOCCNbits.IOCCN6 = 1;
    //interrupt on change for group IOCCP - positive
    IOCCPbits.IOCCP6 = 1;


    // Enable IOCI interrupt 
    PIE0bits.IOCIE = 1;

#if (DEBUG_ENABLE==1)
    RB6PPS = 0x16; //RB6->UART2:TX2;    
#endif
    U2RXPPS = 0x17; //RC7->UART2:RX2;    
    I2C1SDAPPS = 0x14; //RC4->I2C1:SDA1;    
    RC3PPS = 0x21; //RC3->I2C1:SCL1;    
    RC4PPS = 0x22; //RC4->I2C1:SDA1;    
    I2C1SCLPPS = 0x13; //RC3->I2C1:SCL1;  

    // Clock---------------------------
    // NOSC HFINTOSC; NDIV 1; 
    OSCCON1 = 0x60;
    // CSWHOLD may proceed; SOSCPWR Low power; 
    OSCCON3 = 0x00;
    // MFOEN disabled; LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled; 
    OSCEN = 0x00;
    // HFFRQ 16_MHz; 
    OSCFRQ = 0x05;
    // TUN 0; 
    OSCTUNE = 0x00;

    // Timer0-------------------------
    // T0CS FOSC/4; T0CKPS 1:16; T0ASYNC synchronised; 
    T0CON1 = 0x44;

    // TMR0H 249; 
    TMR0H = 0xF9;

    // TMR0L 0; 
    TMR0L = 0x00;

    // Clear Interrupt flag before enabling the interrupt
    PIR3bits.TMR0IF = 0;

    // Enabling TMR0 interrupt.
    PIE3bits.TMR0IE = 1;

    // T0OUTPS 1:1; T0EN enabled; T016BIT 8-bit; 
    T0CON0 = 0x80;

    // UART2------------------------------------------
    // Disable interrupts before changing states
    PIE6bits.U2RXIE = 0;
    PIE6bits.U2TXIE = 0;

    // Set the UART2 module to the options selected in the user interface.
    // P1L 0; 
    U2P1L = 0x00;

    // P2L 0; 
    U2P2L = 0x00;

    // P3L 0; 
    U2P3L = 0x00;

#if (DEBUG_ENABLE==1)
    // BRGS high speed; MODE Asynchronous 8-bit mode; RXEN enabled; TXEN enabled; ABDEN disabled; 
    U2CON0 = 0xB0;
#else
    // BRGS high speed; MODE Asynchronous 8-bit mode; RXEN enabled; TXEN disabled; ABDEN disabled; 
    U2CON0 = 0x90;
#endif


    // RXBIMD Set RXBKIF on rising RX input; BRKOVR disabled; WUE disabled; SENDB disabled; ON enabled; 
    U2CON1 = 0x80;

    // TXPOL not inverted; FLO off; RXPOL not inverted; RUNOVF RX input shifter stops all activity; STP Transmit 1Stop bit, receiver verifies first Stop bit; 
    U2CON2 = 0x00;

    // BRGL 64; 
    U2BRGL = 0x40;

    // BRGH 3; 
    U2BRGH = 0x03;

    // STPMD in middle of first Stop bit; TXWRE No error; 
    U2FIFO = 0x00;

    // ABDIF Auto-baud not enabled or not complete; WUIF WUE not enabled by software; ABDIE disabled; 
    U2UIR = 0x00;

    // ABDOVF Not overflowed; TXCIF 0; RXBKIF No Break detected; RXFOIF not overflowed; CERIF No Checksum error; 
    U2ERRIR = 0x00;

    // TXCIE disabled; FERIE disabled; TXMTIE disabled; ABDOVE disabled; CERIE disabled; RXFOIE disabled; PERIE disabled; RXBKIE disabled; 
    U2ERRIE = 0x00;

    // enable receive interrupt
    PIE6bits.U2RXIE = 1;

    // I2C1----------------------------------
    //EN disabled; RSEN disabled; S Cleared by hardware after Start; CSTR Enable clocking; MODE 7-bit address; 
    I2C1CON0 = 0x04;
    //ACKCNT Acknowledge; ACKDT Acknowledge; ACKSTAT ACK received; ACKT 0; RXO 0; TXU 0; CSD Clock Stretching enabled; 
    I2C1CON1 = 0x80;
    //ACNT disabled; GCEN disabled; FME disabled; ABD enabled; SDAHT 300 ns hold time; BFRET 8 I2C Clock pulses; 
    I2C1CON2 = 0x18;
    //CLK MFINTOSC; 
    I2C1CLK = 0x03;
    //CNTIF 0; ACKTIF 0; WRIF 0; ADRIF 0; PCIF 0; RSCIF 0; SCIF 0; 
    I2C1PIR = 0x00;
    //CNTIE disabled; ACKTIE disabled; WRIE disabled; ADRIE disabled; PCIE disabled; RSCIE disabled; SCIE disabled; 
    I2C1PIE = 0x00;
    //BTOIF No bus timout; BCLIF No bus collision detected; NACKIF No NACK/Error detected; BTOIE disabled; BCLIE disabled; NACKIE disabled; 
    I2C1ERR = 0x00;
    //Count register 
    I2C1CNT = 0xFF;
}

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Milli-seconds functions">
static volatile uint32_t tick = 0;
static external_tick_cb_t external_callback = NULL;

/**
 * Increment tick on Timer 0 1ms interrupt.
 * Call external_callback @10ms to handle async led blink.
 */
void timer0_isr_handler() {
    tick++;
    if (external_callback && (tick % 10 == 0)) {
        external_callback(tick);
        relay_task();
    }
}

/**
 * Get system time in milli seconds.
 * @return Total wake/running time in milli seconds.
 */
uint32_t millis() {
    return tick;
}

/**
 * Generate delay using timer0.
 * @param ms time to generate delay in milli seconds.
 */
void delay_ms(uint32_t ms) {
    uint32_t wait = millis() + ms;
    while (millis() < wait);
}

/**
 * Attach additional function to tick @10ms interval
 * @param cb callback function.
 * @return false if already attached one callback.
 */
bool millis_attach_external_callback(external_tick_cb_t cb) {
    if (external_callback) return 0;

    external_callback = cb;
    return true;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C">
// I2C1 STATES

typedef enum {
    I2C1_IDLE = 0,
    I2C1_SEND_ADR_READ,
    I2C1_SEND_ADR_WRITE,
    I2C1_TX,
    I2C1_RX,
    I2C1_TX_EMPTY,
    I2C1_RX_EMPTY,
    I2C1_SEND_RESTART_READ,
    I2C1_SEND_RESTART_WRITE,
    I2C1_SEND_RESTART,
    I2C1_SEND_STOP,
    I2C1_RX_ACK,
    I2C1_TX_ACK,
    I2C1_RX_NACK_STOP,
    I2C1_RX_NACK_RESTART,
    I2C1_RESET,
    I2C1_ADDRESS_NACK,
    I2C1_BUS_COLLISION,
    I2C1_BUS_ERROR
} i2c1_fsm_states_t;

// I2C1 Event callBack List

typedef enum {
    I2C1_DATA_COMPLETE = 0,
    I2C1_WRITE_COLLISION,
    I2C1_ADDR_NACK,
    I2C1_DATA_NACK,
    I2C1_TIMEOUT,
    I2C1_NULL
} i2c1_callbackIndex_t;

// I2C1 Status Structure

typedef struct {
    i2c1_callback_t callbackTable[6];
    void *callbackPayload[6]; //  each callBack can have a payload
    uint16_t time_out; // I2C1 Timeout Counter between I2C1 Events.
    uint16_t time_out_value; // Reload value for the timeouts
    i2c1_address_t address; // The I2C1 Address
    uint8_t *data_ptr; // pointer to a data buffer
    size_t data_length; // Bytes in the data buffer
    i2c1_fsm_states_t state; // Driver State
    i2c1_error_t error;
    unsigned addressNackCheck : 2;
    unsigned busy : 1;
    unsigned inUse : 1;
    unsigned bufferFree : 1;

} i2c1_status_t;

static void I2C1_SetCallback(i2c1_callbackIndex_t idx, i2c1_callback_t cb, void *ptr);
static void I2C1_Poller(void);
static inline void I2C1_ClearInterruptFlags(void);
static inline void I2C1_MasterFsm(void);

/* I2C1 interfaces */
static inline bool I2C1_MasterOpen(void);
static inline void I2C1_MasterClose(void);
static inline uint8_t I2C1_MasterGetRxData(void);
static inline void I2C1_MasterSendTxData(uint8_t data);
static inline void I2C1_MasterSetCounter(uint8_t counter);
static inline uint8_t I2C1_MasterGetCounter();
static inline void I2C1_MasterResetBus(void);
static inline void I2C1_MasterEnableRestart(void);
static inline void I2C1_MasterDisableRestart(void);
static inline void I2C1_MasterStop(void);
static inline bool I2C1_MasterIsNack(void);
static inline void I2C1_MasterSendAck(void);
static inline void I2C1_MasterSendNack(void);
static inline void I2C1_MasterClearBusCollision(void);
static inline bool I2C1_MasterIsRxBufFull(void);
static inline bool I2C1_MasterIsTxBufEmpty(void);
static inline bool I2C1_MasterIsStopFlagSet(void);
static inline bool I2C1_MasterIsCountFlagSet(void);
static inline bool I2C1_MasterIsNackFlagSet(void);
static inline void I2C1_MasterClearStopFlag(void);
static inline void I2C1_MasterClearCountFlag(void);
static inline void I2C1_MasterClearNackFlag(void);

/* Interrupt interfaces */
static inline void I2C1_MasterEnableIrq(void);
static inline bool I2C1_MasterIsIrqEnabled(void);
static inline void I2C1_MasterDisableIrq(void);
static inline void I2C1_MasterClearIrq(void);
static inline void I2C1_MasterWaitForEvent(void);

static i2c1_fsm_states_t I2C1_DO_IDLE(void);
static i2c1_fsm_states_t I2C1_DO_SEND_ADR_READ(void);
static i2c1_fsm_states_t I2C1_DO_SEND_ADR_WRITE(void);
static i2c1_fsm_states_t I2C1_DO_TX(void);
static i2c1_fsm_states_t I2C1_DO_RX(void);
static i2c1_fsm_states_t I2C1_DO_TX_EMPTY(void);
static i2c1_fsm_states_t I2C1_DO_RX_EMPTY(void);
static i2c1_fsm_states_t I2C1_DO_SEND_RESTART_READ(void);
static i2c1_fsm_states_t I2C1_DO_SEND_RESTART_WRITE(void);
static i2c1_fsm_states_t I2C1_DO_SEND_RESTART(void);
static i2c1_fsm_states_t I2C1_DO_SEND_STOP(void);
static i2c1_fsm_states_t I2C1_DO_RX_ACK(void);
static i2c1_fsm_states_t I2C1_DO_TX_ACK(void);
static i2c1_fsm_states_t I2C1_DO_RX_NACK_STOP(void);
static i2c1_fsm_states_t I2C1_DO_RX_NACK_RESTART(void);
static i2c1_fsm_states_t I2C1_DO_RESET(void);
static i2c1_fsm_states_t I2C1_DO_ADDRESS_NACK(void);
static i2c1_fsm_states_t I2C1_DO_BUS_COLLISION(void);
static i2c1_fsm_states_t I2C1_DO_BUS_ERROR(void);

typedef i2c1_fsm_states_t(*i2c1FsmHandler)(void);
const i2c1FsmHandler i2c1_fsmStateTable[] = {
    I2C1_DO_IDLE,
    I2C1_DO_SEND_ADR_READ,
    I2C1_DO_SEND_ADR_WRITE,
    I2C1_DO_TX,
    I2C1_DO_RX,
    I2C1_DO_TX_EMPTY,
    I2C1_DO_RX_EMPTY,
    I2C1_DO_SEND_RESTART_READ,
    I2C1_DO_SEND_RESTART_WRITE,
    I2C1_DO_SEND_RESTART,
    I2C1_DO_SEND_STOP,
    I2C1_DO_RX_ACK,
    I2C1_DO_TX_ACK,
    I2C1_DO_RX_NACK_STOP,
    I2C1_DO_RX_NACK_RESTART,
    I2C1_DO_RESET,
    I2C1_DO_ADDRESS_NACK,
    I2C1_DO_BUS_COLLISION,
    I2C1_DO_BUS_ERROR
};

i2c1_status_t I2C1_Status = {0};

i2c1_error_t I2C1_Open(i2c1_address_t address) {
    i2c1_error_t returnValue = I2C1_BUSY;

    if (!I2C1_Status.inUse) {
        I2C1_Status.address = address;
        I2C1_Status.busy = 0;
        I2C1_Status.inUse = 1;
        I2C1_Status.addressNackCheck = 0;
        I2C1_Status.state = I2C1_RESET;
        I2C1_Status.time_out_value = 500; // MCC should determine a reasonable starting value here.
        I2C1_Status.bufferFree = 1;

        // set all the call backs to a default of sending stop
        I2C1_Status.callbackTable[I2C1_DATA_COMPLETE] = I2C1_CallbackReturnStop;
        I2C1_Status.callbackPayload[I2C1_DATA_COMPLETE] = NULL;
        I2C1_Status.callbackTable[I2C1_WRITE_COLLISION] = I2C1_CallbackReturnStop;
        I2C1_Status.callbackPayload[I2C1_WRITE_COLLISION] = NULL;
        I2C1_Status.callbackTable[I2C1_ADDR_NACK] = I2C1_CallbackReturnStop;
        I2C1_Status.callbackPayload[I2C1_ADDR_NACK] = NULL;
        I2C1_Status.callbackTable[I2C1_DATA_NACK] = I2C1_CallbackReturnStop;
        I2C1_Status.callbackPayload[I2C1_DATA_NACK] = NULL;
        I2C1_Status.callbackTable[I2C1_TIMEOUT] = I2C1_CallbackReturnReset;
        I2C1_Status.callbackPayload[I2C1_TIMEOUT] = NULL;

        I2C1_MasterClearIrq();
        I2C1_MasterOpen();
        returnValue = I2C1_NOERR;
    }
    return returnValue;
}

i2c1_error_t I2C1_Close(void) {
    i2c1_error_t returnValue = I2C1_BUSY;
    if (!I2C1_Status.busy) {
        I2C1_Status.inUse = 0;
        I2C1_Status.address = 0xff;
        I2C1_MasterClearIrq();
        I2C1_MasterDisableIrq();
        I2C1_MasterClose();
        returnValue = I2C1_Status.error;
    }
    return returnValue;
}

i2c1_error_t I2C1_MasterOperation(bool read) {
    i2c1_error_t returnValue = I2C1_BUSY;
    if (!I2C1_Status.busy) {
        I2C1_Status.busy = true;
        returnValue = I2C1_NOERR;
        I2C1_MasterSetCounter((uint8_t) I2C1_Status.data_length);

        if (read) {
            I2C1_Status.state = I2C1_RX;
            I2C1_DO_SEND_ADR_READ();
        } else {
            I2C1_Status.state = I2C1_TX;
            I2C1_DO_SEND_ADR_WRITE();
        }
        I2C1_Poller();
    }
    return returnValue;
}

i2c1_error_t I2C1_MasterRead(void) {
    return I2C1_MasterOperation(true);
}

i2c1_error_t I2C1_MasterWrite(void) {
    return I2C1_MasterOperation(false);
}

void I2C1_SetTimeOut(uint8_t timeOutValue) {
    I2C1_MasterDisableIrq();
    I2C1_Status.time_out_value = timeOutValue;
    I2C1_MasterEnableIrq();
}

void I2C1_SetBuffer(void *buffer, size_t bufferSize) {
    if (I2C1_Status.bufferFree) {
        I2C1_Status.data_ptr = buffer;
        I2C1_Status.data_length = bufferSize;
        I2C1_Status.bufferFree = false;
    }
}

void I2C1_SetDataCompleteCallback(i2c1_callback_t cb, void *ptr) {
    I2C1_SetCallback(I2C1_DATA_COMPLETE, cb, ptr);
}

void I2C1_SetWriteCollisionCallback(i2c1_callback_t cb, void *ptr) {
    I2C1_SetCallback(I2C1_WRITE_COLLISION, cb, ptr);
}

void I2C1_SetAddressNackCallback(i2c1_callback_t cb, void *ptr) {
    I2C1_SetCallback(I2C1_ADDR_NACK, cb, ptr);
}

void I2C1_SetDataNackCallback(i2c1_callback_t cb, void *ptr) {
    I2C1_SetCallback(I2C1_DATA_NACK, cb, ptr);
}

void I2C1_SetTimeoutCallback(i2c1_callback_t cb, void *ptr) {
    I2C1_SetCallback(I2C1_TIMEOUT, cb, ptr);
}

static void I2C1_SetCallback(i2c1_callbackIndex_t idx, i2c1_callback_t cb, void *ptr) {
    if (cb) {
        I2C1_Status.callbackTable[idx] = cb;
        I2C1_Status.callbackPayload[idx] = ptr;
    } else {
        I2C1_Status.callbackTable[idx] = I2C1_CallbackReturnStop;
        I2C1_Status.callbackPayload[idx] = NULL;
    }
}

static void I2C1_Poller(void) {
    while (I2C1_Status.busy) {
        I2C1_MasterWaitForEvent();
        I2C1_MasterFsm();
    }
}

static inline void I2C1_MasterFsm(void) {
    I2C1_ClearInterruptFlags();

    if (I2C1_Status.addressNackCheck && I2C1_MasterIsNack()) {
        I2C1_Status.state = I2C1_ADDRESS_NACK;
    }
    I2C1_Status.state = i2c1_fsmStateTable[I2C1_Status.state]();
}

static inline void I2C1_ClearInterruptFlags(void) {
    if (I2C1_MasterIsCountFlagSet()) {
        I2C1_MasterClearCountFlag();
    } else if (I2C1_MasterIsStopFlagSet()) {
        I2C1_MasterClearStopFlag();
    } else if (I2C1_MasterIsNackFlagSet()) {
        I2C1_MasterClearNackFlag();
    }
}

static i2c1_fsm_states_t I2C1_DO_IDLE(void) {
    I2C1_Status.busy = false;
    I2C1_Status.error = I2C1_NOERR;
    return I2C1_RESET;
}

static i2c1_fsm_states_t I2C1_DO_SEND_ADR_READ(void) {
    I2C1_Status.addressNackCheck = 2;
    if (I2C1_Status.data_length == 1) {
        I2C1_DO_RX_EMPTY();
    }
    I2C1_MasterSendTxData((uint8_t) (I2C1_Status.address << 1 | 1));
    return I2C1_RX;
}

static i2c1_fsm_states_t I2C1_DO_SEND_ADR_WRITE(void) {
    I2C1_Status.addressNackCheck = 2;
    I2C1_MasterSendTxData((uint8_t) (I2C1_Status.address << 1));
    return I2C1_TX;
}

static i2c1_fsm_states_t I2C1_DO_TX(void) {
    if (I2C1_MasterIsNack()) {
        switch (I2C1_Status.callbackTable[I2C1_DATA_NACK](I2C1_Status.callbackPayload[I2C1_DATA_NACK])) {
            case I2C1_RESTART_READ:
                return I2C1_DO_SEND_RESTART_READ();
            case I2C1_RESTART_WRITE:
                return I2C1_DO_SEND_RESTART_WRITE();
            default:
            case I2C1_CONTINUE:
            case I2C1_STOP:
                return I2C1_IDLE;
        }
    } else if (I2C1_MasterIsTxBufEmpty()) {
        if (I2C1_Status.addressNackCheck) {
            I2C1_Status.addressNackCheck--;
        }
        uint8_t dataTx = *I2C1_Status.data_ptr++;
        i2c1_fsm_states_t retFsmState = (--I2C1_Status.data_length) ? I2C1_TX : I2C1_DO_TX_EMPTY();
        I2C1_MasterSendTxData(dataTx);
        return retFsmState;
    } else {
        return I2C1_TX;
    }
}

static i2c1_fsm_states_t I2C1_DO_RX(void) {
    if (!I2C1_MasterIsRxBufFull()) {
        return I2C1_RX;
    }
    if (I2C1_Status.addressNackCheck) {
        I2C1_Status.addressNackCheck--;
    }

    if (--I2C1_Status.data_length) {
        *I2C1_Status.data_ptr++ = I2C1_MasterGetRxData();
        return I2C1_RX;
    } else {
        i2c1_fsm_states_t retFsmState = I2C1_DO_RX_EMPTY();
        *I2C1_Status.data_ptr++ = I2C1_MasterGetRxData();
        return retFsmState;
    }
}

static i2c1_fsm_states_t I2C1_DO_TX_EMPTY(void) {
    I2C1_Status.bufferFree = true;
    switch (I2C1_Status.callbackTable[I2C1_DATA_COMPLETE](I2C1_Status.callbackPayload[I2C1_DATA_COMPLETE])) {
        case I2C1_RESTART_READ:
            I2C1_MasterEnableRestart();
            return I2C1_SEND_RESTART_READ;
        case I2C1_CONTINUE:
            // Avoid the counter stop condition , Counter is incremented by 1
            I2C1_MasterSetCounter((uint8_t) I2C1_Status.data_length + 1);
            return I2C1_TX;
        default:
        case I2C1_STOP:
            I2C1_MasterDisableRestart();
            return I2C1_SEND_STOP;
    }
}

static i2c1_fsm_states_t I2C1_DO_RX_EMPTY(void) {
    I2C1_Status.bufferFree = true;
    switch (I2C1_Status.callbackTable[I2C1_DATA_COMPLETE](I2C1_Status.callbackPayload[I2C1_DATA_COMPLETE])) {
        case I2C1_RESTART_WRITE:
            I2C1_MasterEnableRestart();
            return I2C1_SEND_RESTART_WRITE;
        case I2C1_RESTART_READ:
            I2C1_MasterEnableRestart();
            return I2C1_SEND_RESTART_READ;
        case I2C1_CONTINUE:
            // Avoid the counter stop condition , Counter is incremented by 1
            I2C1_MasterSetCounter((uint8_t) (I2C1_Status.data_length + 1));
            return I2C1_RX;
        default:
        case I2C1_STOP:
            if (I2C1_Status.state != I2C1_SEND_RESTART_READ) {
                I2C1_MasterDisableRestart();
            }
            return I2C1_RESET;
    }
}

static i2c1_fsm_states_t I2C1_DO_SEND_RESTART_READ(void) {
    I2C1_MasterSetCounter((uint8_t) I2C1_Status.data_length);
    return I2C1_DO_SEND_ADR_READ();
}

static i2c1_fsm_states_t I2C1_DO_SEND_RESTART_WRITE(void) {
    return I2C1_SEND_ADR_WRITE;
}

static i2c1_fsm_states_t I2C1_DO_SEND_RESTART(void) {
    return I2C1_SEND_ADR_READ;
}

static i2c1_fsm_states_t I2C1_DO_SEND_STOP(void) {
    I2C1_MasterStop();
    if (I2C1_MasterGetCounter()) {
        I2C1_MasterSetCounter(0);
        I2C1_MasterSendTxData(0);
    }
    return I2C1_IDLE;
}

static i2c1_fsm_states_t I2C1_DO_RX_ACK(void) {
    I2C1_MasterSendAck();
    return I2C1_RX;
}

static i2c1_fsm_states_t I2C1_DO_TX_ACK(void) {
    I2C1_MasterSendAck();
    return I2C1_TX;
}

static i2c1_fsm_states_t I2C1_DO_RX_NACK_STOP(void) {
    I2C1_MasterSendNack();
    I2C1_MasterStop();
    return I2C1_DO_IDLE();
}

static i2c1_fsm_states_t I2C1_DO_RX_NACK_RESTART(void) {
    I2C1_MasterSendNack();
    return I2C1_SEND_RESTART;
}

static i2c1_fsm_states_t I2C1_DO_RESET(void) {
    I2C1_MasterResetBus();
    I2C1_Status.busy = false;
    I2C1_Status.error = I2C1_NOERR;
    return I2C1_RESET;
}

static i2c1_fsm_states_t I2C1_DO_ADDRESS_NACK(void) {
    I2C1_Status.addressNackCheck = 0;
    I2C1_Status.error = I2C1_FAIL;
    I2C1_Status.busy = false;
    switch (I2C1_Status.callbackTable[I2C1_ADDR_NACK](I2C1_Status.callbackPayload[I2C1_ADDR_NACK])) {
        case I2C1_RESTART_READ:
        case I2C1_RESTART_WRITE:
            return I2C1_DO_SEND_RESTART();
        default:
            return I2C1_RESET;
    }
}

static i2c1_fsm_states_t I2C1_DO_BUS_COLLISION(void) {
    // Clear bus collision status flag
    I2C1_MasterClearIrq();

    I2C1_Status.error = I2C1_FAIL;
    switch (I2C1_Status.callbackTable[I2C1_WRITE_COLLISION](I2C1_Status.callbackPayload[I2C1_WRITE_COLLISION])) {
        case I2C1_RESTART_READ:
            return I2C1_DO_SEND_RESTART_READ();
        case I2C1_RESTART_WRITE:
            return I2C1_DO_SEND_RESTART_WRITE();
        default:
            return I2C1_DO_RESET();
    }
}

static i2c1_fsm_states_t I2C1_DO_BUS_ERROR(void) {
    I2C1_MasterResetBus();
    I2C1_Status.busy = false;
    I2C1_Status.error = I2C1_FAIL;
    return I2C1_RESET;
}

void I2C1_BusCollisionIsr(void) {
    I2C1_MasterClearBusCollision();
    I2C1_Status.state = I2C1_RESET;
}

i2c1_operations_t I2C1_CallbackReturnStop(void *funPtr) {
    return I2C1_STOP;
}

i2c1_operations_t I2C1_CallbackReturnReset(void *funPtr) {
    return I2C1_RESET_LINK;
}

i2c1_operations_t I2C1_CallbackRestartWrite(void *funPtr) {
    return I2C1_RESTART_WRITE;
}

i2c1_operations_t I2C1_CallbackRestartRead(void *funPtr) {
    return I2C1_RESTART_READ;
}

/* I2C1 Register Level interfaces */
static inline bool I2C1_MasterOpen(void) {
    if (!I2C1CON0bits.EN) {
        //CNTIF 0; ACKTIF 0; WRIF 0; ADRIF 0; PCIF 0; RSCIF 0; SCIF 0; 
        I2C1PIR = 0x00;
        //CNTIE disabled; ACKTIE disabled; WRIE disabled; ADRIE disabled; PCIE disabled; RSCIE disabled; SCIE disabled; 
        I2C1PIE = 0x00;
        //BTOIF No bus timout; BCLIF No bus collision detected; NACKIF No NACK/Error detected; BTOIE disabled; BCLIE disabled; NACKIE disabled; 
        I2C1ERR = 0x00;
        //Count register 
        I2C1CNT = 0xFF;
        //Clock PadReg Configuration
        RC3I2C = 0x51;
        //Data PadReg Configuration
        RC4I2C = 0x51;
        //Enable I2C1
        I2C1CON0bits.EN = 1;
        return true;
    }
    return false;
}

static inline void I2C1_MasterClose(void) {
    //Disable I2C1
    I2C1CON0bits.EN = 0;
    //CNTIF 0; ACKTIF 0; WRIF 0; ADRIF 0; PCIF 0; RSCIF 0; SCIF 0; 
    I2C1PIR = 0x00;
    //Set Clear Buffer Flag
    I2C1STAT1bits.CLRBF = 1;
}

static inline uint8_t I2C1_MasterGetRxData(void) {
    return I2C1RXB;
}

static inline void I2C1_MasterSendTxData(uint8_t data) {
    I2C1TXB = data;
}

static inline uint8_t I2C1_MasterGetCounter() {
    return I2C1CNT;
}

static inline void I2C1_MasterSetCounter(uint8_t counter) {
    I2C1CNT = counter;
}

static inline void I2C1_MasterResetBus(void) {
    //Disable I2C1
    I2C1CON0bits.EN = 0;
    //Set Clear Buffer Flag
    I2C1STAT1bits.CLRBF = 1;
    //Enable I2C1
    I2C1CON0bits.EN = 1;
}

static inline void I2C1_MasterEnableRestart(void) {
    //Enable I2C1 Restart
    I2C1CON0bits.RSEN = 1;
}

static inline void I2C1_MasterDisableRestart(void) {
    //Disable I2C1 Restart
    I2C1CON0bits.RSEN = 0;
}

static inline void I2C1_MasterStop(void) {
    //Clear Start Bit
    I2C1CON0bits.S = 0;
}

static inline bool I2C1_MasterIsNack(void) {
    return I2C1CON1bits.ACKSTAT;
}

static inline void I2C1_MasterSendAck(void) {
    I2C1CON1bits.ACKDT = 0;
}

static inline void I2C1_MasterSendNack(void) {
    I2C1CON1bits.ACKDT = 1;
}

static inline void I2C1_MasterClearBusCollision(void) {
    I2C1ERRbits.BCLIF = 0;
    I2C1ERRbits.BTOIF = 0;
    I2C1ERRbits.NACKIF = 0;
}

static inline bool I2C1_MasterIsRxBufFull(void) {
    return I2C1STAT1bits.RXBF;
}

static inline bool I2C1_MasterIsTxBufEmpty(void) {
    return I2C1STAT1bits.TXBE;
}

static inline bool I2C1_MasterIsStopFlagSet(void) {
    return I2C1PIRbits.PCIF;
}

static inline bool I2C1_MasterIsCountFlagSet(void) {
    return I2C1PIRbits.CNTIF;
}

static inline bool I2C1_MasterIsNackFlagSet(void) {
    return I2C1ERRbits.NACKIF;
}

static inline void I2C1_MasterClearStopFlag(void) {
    I2C1PIRbits.PCIF = 0;
}

static inline void I2C1_MasterClearCountFlag(void) {
    I2C1PIRbits.CNTIF = 0;
}

static inline void I2C1_MasterClearNackFlag(void) {
    I2C1ERRbits.NACKIF = 0;
}

static inline void I2C1_MasterEnableIrq(void) {
    PIE3bits.I2C1IE = 1;
    PIE3bits.I2C1EIE = 1;
    PIE2bits.I2C1RXIE = 1;
    PIE3bits.I2C1TXIE = 1;

    I2C1PIEbits.PCIE = 1;
    I2C1PIEbits.CNTIE = 1;
    I2C1ERRbits.NACKIE = 1;
}

static inline bool I2C1_MasterIsIrqEnabled(void) {
    return (PIE2bits.I2C1RXIE && PIE3bits.I2C1TXIE && PIE3bits.I2C1IE);
}

static inline void I2C1_MasterDisableIrq(void) {
    PIE3bits.I2C1IE = 0;
    PIE3bits.I2C1EIE = 0;
    PIE2bits.I2C1RXIE = 0;
    PIE3bits.I2C1TXIE = 0;
    I2C1PIEbits.SCIE = 0;
    I2C1PIEbits.PCIE = 0;
    I2C1PIEbits.CNTIE = 0;
    I2C1PIEbits.ACKTIE = 0;
    I2C1PIEbits.RSCIE = 0;
    I2C1ERRbits.BCLIE = 0;
    I2C1ERRbits.BTOIE = 0;
    I2C1ERRbits.NACKIE = 0;
}

static inline void I2C1_MasterClearIrq(void) {
    I2C1PIR = 0x00;
}

static inline void I2C1_MasterWaitForEvent(void) {
    while (1) {
        if (PIR3bits.I2C1TXIF) {
            break;
        }
        if (PIR2bits.I2C1RXIF) {
            break;
        }
        if (I2C1PIRbits.PCIF) {
            break;
        }
        if (I2C1PIRbits.CNTIF) {
            break;
        }
        if (I2C1ERRbits.NACKIF) {
            break;
        }
    }
}
// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="UART2">

/**
 * Read UART2 data.
 * @return Receive data.
 */
uint8_t uart2_read(void) {
    while (!PIR6bits.U2RXIF) {
    }


    if (U2ERRIRbits.FERIF) {
        U2ERRIRbits.FERIF = 0;
    }

    if (U2ERRIRbits.RXFOIF) {
        U2ERRIRbits.RXFOIF = 0;
    }

    return U2RXB;
}

/**
 * Redirect UART2 read to getch().
 * @return 
 */
char getch(void) {
    return uart2_read();
}

#if (DEBUG_ENABLE==1)

/**
 * Write data to UART2.
 * @param data to write.
 */
void uart2_write(uint8_t data) {
    while (0 == PIR6bits.U2TXIF) {
    }

    U2TXB = data; // Write the data byte to the USART.
}

/**
 * Redirect UART2 write to DEBUG_MSG()/putch() func.
 * @param data data to send.
 */
void putch(char data) {
    uart2_write(data);
}
#else

/**
 * Write data to UART2.
 * @param data to write.
 */
void uart2_write(uint8_t data) {

}

/**
 * Redirect UART2 write to DEBUG_MSG()/putch() func.
 * @param data data to send.
 */
void putch(char data) {

}
#endif

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Led">
static volatile led_async_t async_red;
static volatile led_async_t async_green;

/**
 * Blink led async handler.
 * Must Attach to milli external callback.
 * @param tick current system time in milli second from time 0 callback.
 */
static void blink_async_handler(uint32_t tick) {

    if (async_red.is_enabled) {
        if (async_red.timeout != -1) {
            if (tick < (async_red.timeout + async_red.start_time)) {
                // blink...
                if (async_red.rate_ms == -1) {
                    RED_LED_SetLow();
                } else if (((uint32_t) (tick - async_red.last_tick) >= async_red.rate_ms) &&
                        (tick > async_red.start_time)) {
                    async_red.last_tick = tick;
                    RED_LED_Toggle();
                }
            } else {
                RED_LED_SetHigh();
                async_red.is_enabled = false;
            }
        } else {
            // blink
            if (((uint32_t) (tick - async_red.last_tick) >= async_red.rate_ms)&&
                    (tick > async_red.start_time)) {
                async_red.last_tick = tick;
                RED_LED_Toggle();
            }
        }
    }

    if (async_green.is_enabled) {
        if (async_green.timeout != -1) {
            if (tick < (async_green.timeout + async_green.start_time)) {
                // blink...
                if (async_green.rate_ms == -1) {
                    GREEN_LED_SetLow();
                } else if (((uint32_t) (tick - async_green.last_tick) >= async_green.rate_ms) &&
                        (tick > async_green.start_time)) {
                    async_green.last_tick = tick;
                    GREEN_LED_Toggle();
                }
            } else {
                GREEN_LED_SetHigh();
                async_green.is_enabled = false;
            }
        } else {
            // blink
            if (((uint32_t) (tick - async_green.last_tick) >= async_green.rate_ms) &&
                    (tick > async_green.start_time)) {

                async_green.last_tick = tick;
                GREEN_LED_Toggle();
            }
        }
    }
}

/**
 * Initialize led structure.
 * Add Led task to milli callback.
 * Can blink minimum rate @10ms.
 */
void blink_init(void) {

    memset(&async_red, 0, sizeof (async_red));
    memset(&async_green, 0, sizeof (async_green));
    millis_attach_external_callback(blink_async_handler);
    blink_led_async_stop(LED_GREEN);
    blink_led_async_stop(LED_RED);
}

/**
 * Blink given led in blocking manner.
 * @param led  led to blink.
 * @param ms   duration/timeout to blink.
 */
void blink_led(led_t led, uint32_t ms) {
    uint32_t tout = millis() + ms;

    while (millis() < tout) {

        switch (led) {
            case LED_GREEN:
            {
                //Toggle
                GREEN_LED_Toggle();
            }
                break;
            case LED_RED:
            {
                //Toggle
                RED_LED_Toggle();

            }
                break;
            case LED_YELLOW:
            {
                //Toggle

                RED_LED_Toggle();
                GREEN_LED_Toggle();

            }
                break;
            default:return;
        }

        delay_ms(LED_BLINKING_RATE_MS);
    }

    // off both led.
    RED_LED_SetHigh();
    GREEN_LED_SetHigh();
}

/**
 * Blink led without blocking routine/ Async manner.
 * @param led led to blink.
 * @param blink_rate toggle time in ms. -1 for on continue till timeout.
 * @param timeout duration/timeout to toggle. -1 for forever toggle.
 */
void blink_led_async(led_t led, int blink_rate, int timeout) {
    switch (led) {
        case LED_GREEN:
        {
            async_green.is_enabled = false;
            async_green.rate_ms = blink_rate;
            async_green.timeout = timeout;
            async_green.start_time = millis();
            async_green.last_tick = 0;
            async_green.is_enabled = true;
        }
            break;
        case LED_RED:
        {
            async_red.is_enabled = false;
            async_red.rate_ms = blink_rate;
            async_red.timeout = timeout;
            async_red.start_time = millis();
            async_red.last_tick = 0;
            async_red.is_enabled = true;

        }
            break;

        case LED_YELLOW:
        {

            async_red.is_enabled = false;
            async_green.is_enabled = false;

            async_red.rate_ms = blink_rate;
            async_red.timeout = timeout;
            async_red.start_time = millis();
            async_red.last_tick = 0;

            async_green.rate_ms = blink_rate;
            async_green.timeout = timeout;
            async_green.start_time = async_red.start_time;
            async_green.last_tick = 0;

            async_red.is_enabled = true;
            async_green.is_enabled = true;

        }
            break;
        case LED_ALTER:
        {
            async_green.is_enabled = false;
            async_red.is_enabled = false;

            async_green.rate_ms = blink_rate;
            async_green.timeout = timeout;
            async_green.start_time = millis();
            async_green.last_tick = 0;

            async_red.rate_ms = blink_rate;
            async_red.timeout = timeout;
            async_red.start_time = async_green.start_time + blink_rate;
            async_red.last_tick = async_green.start_time;

            async_green.is_enabled = true;
            async_red.is_enabled = true;
        }
            break;

        default:return;
    }
}

/**
 * Stop blinking led from async task.
 * @param led led to stop/remove blink task.
 */
void blink_led_async_stop(led_t led) {
    switch (led) {
        case LED_GREEN:
        {
            async_green.is_enabled = false;
            GREEN_LED_SetHigh();
        }
            break;
        case LED_RED:
        {
            async_red.is_enabled = false;
            RED_LED_SetHigh();
        }
            break;

        case LED_YELLOW:
        {

            async_red.is_enabled = false;
            async_green.is_enabled = false;
            RED_LED_SetHigh();
            GREEN_LED_SetHigh();
        }
            break;
        default:return;
    }
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Receiver switch on board">
static volatile recv_switch_t recv_switch;

/**
 * RA0 on-change interrupt handler.
 */
static void ra0_ioc_handler(void) {
    bool released = RECV_SW_GET();

    if (released) {
        recv_switch.released_time = millis();

        recv_switch.duration = recv_switch.released_time - recv_switch.pressed_time;
        recv_switch.pressed_time = 0;

        if ((uint32_t) (recv_switch.duration) >= SWITCH_DEBOUNCE_IN_MS) {
            recv_switch.event = 1;
        } else {
            recv_switch.event = 0;
        }

    } else {

        recv_switch.pressed_time = millis();
    }
}

/**
 * Initialize receiver switch to default.
 * @return receiver switch pointer.
 */
recv_switch_t *recv_switch_begin() {
    memset(&recv_switch, 0, sizeof (recv_switch));

    return &recv_switch;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Relay driver">
static volatile relay_task_t relay_tasker[CONFIG_RELAY_MAX] = {0};

/**
 * Initialize relay task structure.
 */
void relay_begin(void) {
    memset(&relay_tasker, 0, sizeof (relay_tasker));
}

/**
 * Set pulse width for specific relay.
 * @param relay relay no to save pulse width.
 * @param timems pulse width in ms.
 */
void relay_set_pulse_time(relay_t relay, uint32_t timems) {
    if (timems >= 0 && timems < CONFIG_PARAM_INVALID) {
        system_cfg.pulse_width[relay].pulse_width.val = timems;
        system_cfg_last_update = millis();
        system_cfg_update = true;
    }
}

/**
 * Forget pulse width time and set default time for specific relay
 * @param relay relay to forget pulse time.
 */
void relay_forget_pulse_time(relay_t relay) {

    relay_set_pulse_time(relay, CONFIG_RELAY_PULSE_DEFAULT_VAL);
}

/**
 * Get pulse width of specific relay.s
 * @param relay relay index.
 * @return pulse width value.
 */
uint32_t relay_get_pulse_time(relay_t relay) {

    return system_cfg.pulse_width[relay].pulse_width.val;
}

/**
 * Set trigger edge value
 * @param active_state positive/negative edge trigger value.
 */
void relay_set_trigger_edge(edge_t active_state) {
    system_cfg.trigger_setting.active_low = active_state;
    system_cfg_last_update = millis();
    system_cfg_update = true;
}

/**
 * Get relay trigger edge value.
 * @return relay trigger edge.
 */
edge_t relay_get_trigger_edge() {
    return (system_cfg.trigger_setting.active_low & 0xFF);
}

/**
 * Set wait time for trigger.
 * Time to wait before trigger after trigger event occur.
 * @param t time in ms.
 */
void relay_set_trigger_wait(uint32_t t) {
    system_cfg.trigger_setting.wait_time = t;
    system_cfg_last_update = millis();
    system_cfg_update = true;
}

/**
 * Get trigger wait time value.
 * @return wait time in ms.
 */
uint32_t relay_get_trigger_wait() {
    return system_cfg.trigger_setting.wait_time;
}

/**
 * Set trigger width time in ms.
 * Time to keep active after wait time.
 * @param t time in ms.
 */
void relay_set_trigger_width(uint32_t t) {
    system_cfg.trigger_setting.width_time = t;
    system_cfg_last_update = millis();
    system_cfg_update = true;
}

/**
 * Get trigger width time.
 * @return time in ms.
 */
uint32_t relay_get_trigger_width() {
    return system_cfg.trigger_setting.width_time;
}

/**
 * Trigger relay on/off.
 * @param relay relay no.
 * @param value true/false
 */
void relay_set_value(relay_t relay, bool value) {
    switch (relay) {
        case RELAY_01:
        {
            if (value) {
                RELAY0_SetHigh();
            } else {
                RELAY0_SetLow();
            }
            break;
        }
        case RELAY_02:
        {
            if (value) {
                RELAY1_SetHigh();
            } else {
                RELAY1_SetLow();
            }
            break;
        }
        case RELAY_03:
        {
            if (value) {
                RELAY2_SetHigh();
            } else {
                RELAY2_SetLow();
            }
            break;
        }
        case RELAY_04:
        {
            if (value) {
                RELAY3_SetHigh();
            } else {

                RELAY3_SetLow();
            }
            break;
        }
        default:break;
    }
}

/**
 * Get relay current state.
 * @param relay relay no.
 * @return current state.
 */
bool relay_get_value(uint8_t relay) {
    bool state = 0;

    switch (relay) {
        case RELAY_01:
        {
            state = RELAY0_GetValue();
            break;
        }
        case RELAY_02:
        {
            state = RELAY1_GetValue();
            break;
        }
        case RELAY_03:
        {
            state = RELAY2_GetValue();
            break;
        }
        case RELAY_04:
        {
            state = RELAY3_GetValue();

            break;
        }
        default:break;
    }
    return state;
}

/**
 * Keep running relay task to get relay working.
 */
void relay_task(void) {
    for (uint8_t i = 0; i < CONFIG_RELAY_MAX; i++) {
        if (relay_tasker[i].pending) {
            if ((millis()>=relay_tasker[i].start_time) && ((uint32_t )(millis() - relay_tasker[i].start_time)<=( relay_tasker[i].timeout))) {
                if (!relay_get_value(i)) {
                    relay_set_value(i, true);
                } else {
                    relay_set_value(i, false);
                }
                relay_tasker[i].wait = false;
            } else if (relay_tasker[i].wait == false) {
                relay_set_value(i, false);
                relay_tasker[i].pending = false;
            } else {
            }
        }
    }

}

/**
 * Turn relay on/ pulse on for specific time.
 * @param relay relay no.
 * @param manual_tout time in milli-seconds to keep turn on.
 */
void relay_pulse_on(relay_t relay, int manual_tout) {
    if (relay < CONFIG_RELAY_MAX) {
        relay_tasker[relay].start_time = millis();
        relay_tasker[relay].timeout = (manual_tout > 0) ? manual_tout : relay_get_pulse_time(relay);
        relay_tasker[relay].wait = false;
        if (relay_tasker[relay].timeout > 0 && relay_tasker[relay].timeout < CONFIG_PARAM_INVALID) {

            relay_tasker[relay].pending = true;
        }
    }
}

/**
 * Trigger relay for provided config.
 */
void relay_trigger_on() {
    uint32_t wait_time = relay_get_trigger_wait();
    relay_tasker[1].start_time = millis() + ((wait_time == CONFIG_PARAM_INVALID) ? 2000 : wait_time);
    relay_tasker[1].timeout = relay_get_trigger_width();
    relay_tasker[1].wait = true;
    relay_tasker[1].pending = true;
}

/**
 * Trigger relay for provided config.
 */
void relay_trigger_off() {
    relay_tasker[1].pending = false;
    relay_set_value(1, false);
}

/**
 * Set relay working mode.
 * @param relay relay no.
 * @param mode  mode.
 * @sa relay_mode_list_t
 */
void relay_set_mode(relay_t relay, uint8_t mode) {
    if (relay >= RELAY_01 && relay <= RELAY_04) {
        system_cfg.relay_mode.mode[relay] = mode;
        system_cfg_last_update = millis();
        system_cfg_update = true;
    }
}

/**
 * Get relay working mode.
 * @param relay relay no.
 * @return activating mode.
 */
uint8_t relay_get_mode(relay_t relay) {
    return system_cfg.relay_mode.mode[relay];
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="RF-433 Receiver">
// Must set same baudrate to UART module 2.
// rf packet receive data..
static volatile uint8_t packet_buffer[RF_PACKET_BUFFER_SIZE];
static uint8_t decrypt_packet[RF_PACKET_DCRYPT_BUFF_SIZE];
static volatile uint8_t byte_count;
static volatile bool packet_found;
static volatile bool packet_received;

/**
 * Receives RF data via UART-2.
 * Must called in UART2 Rx interrupt handle.
 */

static void uart2_isr_handler() {
    uint8_t t = U2RXB;
    if ((t == 0xF0) && (!packet_found) && (!packet_received)) {
        packet_found = true;
        byte_count = 0;
        packet_buffer[byte_count++] = t;
    } else {
        if (packet_found) {
            packet_buffer[byte_count++] = t;
            if (byte_count >= RF_PACKET_BUFFER_SIZE) {

                t = (packet_buffer[1] + packet_buffer[2] + packet_buffer[3] +
                        packet_buffer[5] + packet_buffer[6] + packet_buffer[7] +
                        packet_buffer[9] + packet_buffer[10]);

                if ((packet_buffer[0] == 0xF0 && packet_buffer[4] == 0xFE) && (packet_buffer[8] == 0x5A)) {
                    if (t == packet_buffer[11]) {
                        packet_received = true;
                    } else {
                        packet_received = false;
                    }
                }
                packet_found = 0;
            }
        }
    }
}

/**
 * To calculate packet checksum.
 * @param buffer data pointer to calculate checksum.
 * @param len total bytes to compute checksum.
 * @return  calculated checksum
 */

static uint8_t crc8(uint8_t *buffer, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc += buffer[i];
    }
    crc = (~crc) + 1;

    return crc;
}

/**
 * Initialize RF receiver and initialize all variables.
 * Attach UART Receive ISR callback to parse function.
 * 
 */
void rf_receiver_begin() {

    byte_count = 0;
    packet_received = 0;
    packet_found = 0;
}

/**
 * Check if any RF packet available or not.
 * @return true if available.
 */
bool rf_receiver_available(void) {
    if (packet_received) {
        uint8_t cypher_msg[RF_PACKET_DCRYPT_BUFF_SIZE * 2];

        // copy the message remove 5 preamble bytes..
        memcpy(cypher_msg, &packet_buffer[12], sizeof (cypher_msg));

        // Decrypt...
        decrypt(decrypt_packet, cypher_msg, sizeof (decrypt_packet), sizeof (cypher_msg));

        uint8_t crc = crc8(decrypt_packet, RF_PACKET_DCRYPT_BUFF_SIZE - 1);

        if (crc == decrypt_packet[RF_PACKET_DCRYPT_BUFF_SIZE - 1]) {
            return true;
        }
        packet_received = false;
    }

    return false;
}

/**
 * Get transmitter id from packet.
 * @return transmitter id.
 */
uint32_t rf_receiver_get_tx_id(void) {
    id_t id;
    id.id = 0xFFFFFFFF;

    if (rf_receiver_available()) {
        id.u8[0] = decrypt_packet[16];
        id.u8[1] = decrypt_packet[18];
        id.u8[2] = decrypt_packet[20];
        id.u8[3] = decrypt_packet[22];
    }

    return id.id;
}

/**
 * To get received packet to buffer.
 * 
 * After calling this function it will remove packet from internal buffer
 * and search for next packet.
 * 
 * @param buffer pointer to get packet.
 * @param blen provided buffer size.
 * @return packet payload count if buffer size if okay else -1.
 */
int rf_receiver_get_msg(uint8_t *buffer, uint8_t blen) {
    if (blen >= RF_PACKET_PAYLOAD_COUNT) {
        uint8_t count = decrypt_packet[25];

        count = (count > RF_PACKET_PAYLOAD_COUNT) ? RF_PACKET_PAYLOAD_COUNT : count;

        // Copy counter value to buffer...
        memcpy(recv_counter_value, &decrypt_packet[0], sizeof (recv_counter_value));

        memcpy(buffer, &decrypt_packet[25], RF_PACKET_PAYLOAD_COUNT);

        packet_received = false;
        return count;
    }
    packet_received = false;

    return -1;
}

/**
 * 
 * Verify that the transmitter is Master or Normal.
 * @return 
 */
bool is_transmitter_is_master(uint32_t id) {
    if (id == RF_MASTER_TRANSMITTER_ID)return true;
    return false;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Rotary switch(DS)">
// Current position of switch.
static volatile uint8_t rotary_pos = 0;

/**
 * Update rotary switch position from GPIO state.
 * @return rotary position(0-9) step.
 */
static uint8_t rotary_swtich_update(void) {
    uint8_t pos = 0;

    pos |= (~(ROT_SW_1_GetValue() << 0))&0x01;
    pos |= (~(ROT_SW_2_GetValue() << 1))&0x02;
    pos |= (~(ROT_SW_3_GetValue() << 2))&0x04;
    pos |= (~(ROT_SW_4_GetValue() << 3))&0x08;

    rotary_switch_on_change_handler();

    return pos;
}

/**
 * Pin change interrupt  of rotary switch.
 * Must called in B0/B1/B2/B3 pin change interrupt.
 * IOC : Any
 */
void rb0to3_ioc_handler() {

    uint8_t pos = 0;
    pos = rotary_swtich_update();
    rotary_pos = pos;
}

/**
 * Initialize rotary switch initial state;
 */
void rotary_switch_begin() {

    rotary_pos = rotary_swtich_update();
}

/**
 * To get rotary switch position(0-9).
 * @return position of switch(0-9).
 */
rotary_position_t rotary_position_now() {

    uint8_t pos = 0;

    pos |= (~(ROT_SW_1_GetValue() << 0))&0x01;
    pos |= (~(ROT_SW_2_GetValue() << 1))&0x02;
    pos |= (~(ROT_SW_3_GetValue() << 2))&0x04;
    pos |= (~(ROT_SW_4_GetValue() << 3))&0x08;

    return pos;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="EEPROM">
/*
 * =============================================================================
 *                              I2C Low Level                                   
 * =============================================================================
 */

/**
 * I2C Write function.
 * @param address write location in eeprom.
 * @param data pointer to write data buffer.
 * @param len no. of bytes to write.
 */
static void I2C1_WriteNBytesFunc(i2c1_address_t address, void* data, size_t len) {
    while (!I2C1_Open(address)); // sit here until we get the bus..
    I2C1_SetBuffer(data, len);
    I2C1_SetAddressNackCallback(NULL, NULL); //NACK polling?
    I2C1_MasterWrite();

    while (I2C1_BUSY == I2C1_Close()); // sit here until finished.
}

/**
 * I2C Read Function.
 * @param address read location in eeprom.
 * @param data pointer to store data buffer.
 * @param len no. of bytes to read.
 */
static void I2C1_ReadNBytesFunc(i2c1_address_t address, uint8_t *data, size_t len) {
    while (!I2C1_Open(address)); // sit here until we get the bus..
    I2C1_SetBuffer(data, len);
    I2C1_MasterRead();

    while (I2C1_BUSY == I2C1_Close()); // sit here until finished.
}

/*
 *==============================================================================
 *                              EEPROM API
 *==============================================================================
 */

/**
 * Used to read single byte from given address/location.
 * @param add read from address.
 * @return value.
 */
uint8_t eeprom_read_byte(uint32_t add) {

    uint8_t read = 0;
    eeprom_read_nbytes(add, &read, 1);

    return read;
}

/**
 * Used to write single byte to given location/address in eeprom.
 * @param add write location.
 * @param data data byte to write.
 */
void eeprom_write_byte(uint32_t add, uint8_t data) {

    eeprom_write_nbytes(add, &data, 1);
}

/**
 * Used to write array to eeprom.
 * Max len is limited to page size (128 bytes)
 * 
 * @param add write location.
 * @param pbuffer pointer to write buffer.
 * @param len total number of bytes to write (Max 128).
 */
void eeprom_read_nbytes(uint32_t add, uint8_t *pbuffer, uint8_t len) {

    uint8_t n = len;

    uint16_t in_add;
    uint32_t address = add;
    uint8_t count = 0;
    while (n > 0) {

        in_add = (uint8_t) address;
        in_add = in_add << 8;
        in_add |= (uint8_t) (address >> 8);

        uint8_t byte_to_read = n > 128 ? 128 : n;

        int i2c_address = EEPROM_I2C_ADDRESS;
        if (address > 65535) {

            i2c_address = i2c_address | 0x08;
        }

        // seven-bit address
        i2c_address = (i2c_address >> 1);

        I2C1_WriteNBytesFunc((uint8_t) i2c_address, &in_add, 2);
        I2C1_ReadNBytesFunc((uint8_t) i2c_address, &pbuffer[count * 128], byte_to_read);
        n -= byte_to_read;
        address += byte_to_read;
        count++;
    }
}

/**
 * Used to read array to eeprom.
 * Max len is limited to page size (128 bytes)
 * 
 * @param add read location.
 * @param pbuffer pointer to read buffer.
 * @param len total number of bytes to read (Max 128).
 */
void eeprom_write_nbytes(uint32_t add, uint8_t *pbuffer, uint8_t len) {

    uint8_t n = len;

    uint16_t in_add;
    uint32_t address = add;
    uint8_t count = 0;




    uint8_t buff[130];

    while (n > 0) {
        buff[0] = ((uint8_t) (address >> 8));
        buff[1] = (uint8_t) address;
        uint8_t byte_to_write = n > 128 ? 128 : n;

        int i2c_address = EEPROM_I2C_ADDRESS;
        if (address > 65535) {

            i2c_address = i2c_address | 0x08;
        }

        // seven-bit address
        i2c_address = (i2c_address >> 1);

        memcpy(&buff[2], &pbuffer[128 * count], byte_to_write);
        I2C1_WriteNBytesFunc((uint8_t) i2c_address, buff, byte_to_write + 2);
        n -= byte_to_write;
        address += byte_to_write;
        count++;
        delay_ms(5);
    }
}

/**
 * Enables eeprom write protection.
 */
void eeprom_enable_wp() {

    EEP_WP_SetHigh();
}

/**
 * Disable eeprom write protection.
 */
void eeprom_disable_wp() {

    EEP_WP_SetLow();
}

/*
 * =============================================================================
 *                          APP SPECIFIC API
 * =============================================================================
 */

void eep_begin(void) {

    eeprom_disable_wp();
}

void eep_erase(void) {
    system_config_t cfg;
    memset(&cfg, 0xFF, sizeof (cfg));
    eeprom_write_nbytes(CONFIG_BEGIN_ADD, (uint8_t*) & cfg, sizeof (cfg));
}

int eep_remember_tx_id(transmitter_id_t *id) {

    // Check if already exist don't add
    int tx_idx = eep_check_tx_id(id->id);

    if (tx_idx >= 0 && tx_idx <= 3) {
        system_cfg.transmitter[tx_idx].id = CONFIG_PARAM_INVALID;
    }

    // Add new
    uint8_t n = 0;
    for (n = 0; n < CONFIG_RF_ID_MAX; n++) {
        if (system_cfg.transmitter[n].id == CONFIG_PARAM_INVALID) {

            system_cfg.transmitter[n].id = id->id;
            system_cfg.transmitter[n].key = id->key;

            memcpy(system_cfg.transmitter[n].counter, id->counter, sizeof (id->counter));

            system_cfg_last_update = millis();
            system_cfg_update = true;
            return n;
        }
    }
    return ERR_RF_STORAGE_FULL;
}

int eep_check_tx_id(uint32_t id) {
    uint8_t n = 0;
    for (n = 0; n < CONFIG_RF_ID_MAX + 1; n++) {
        if (system_cfg.transmitter[n].id == id) {
            return n;
        }
    }
    return ERR_RF_NOT_EXIST;
}

int eep_forget_tx_id(uint32_t id) {
    uint8_t n = 0;
    for (n = 0; n < CONFIG_RF_ID_MAX; n++) {
        if (system_cfg.transmitter[n].id == id) {
            system_cfg.transmitter[n].id = CONFIG_PARAM_INVALID;
            system_cfg.transmitter[n].key = CONFIG_PARAM_INVALID;

            memset(system_cfg.transmitter[n].counter, 0xFF, sizeof (counter_t));

            system_cfg_last_update = millis();
            system_cfg_update = true;
            return n;
        }
    }
    return ERR_RF_NOT_EXIST;
}

bool eep_forget_all() {
    uint8_t n = 0;
    for (n = 0; n < CONFIG_RF_ID_MAX; n++) {
        system_cfg.transmitter[n].id = CONFIG_PARAM_INVALID;
        system_cfg.transmitter[n].key = CONFIG_PARAM_INVALID;
        memset(system_cfg.transmitter[n].counter, 0xFF, sizeof (counter_t));
    }
    system_cfg_last_update = millis();
    system_cfg_update = true;

    return true;
}

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Trigger handle">

void rc6_interrupt_handler() {
    if (rotary_position_now() == ROT_POS_0) {

        bool state = TRIGGER_GetValue();
        edge_t edge = relay_get_trigger_edge();
        if (state) {
            if (edge == TRIGGER_ACTIVE_HIGH) {
                relay_trigger_on();
            } else {
                relay_trigger_off();
            }
        } else {
            if (edge == TRIGGER_ACTIVE_LOW) {
                relay_trigger_on();
            } else {
                relay_trigger_off();
            }
        }
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Interrupt Handler">

void __interrupt() INTERRUPT_InterruptManager(void) {
    // interrupt handler
    if (PIE3bits.TMR0IE == 1 && PIR3bits.TMR0IF == 1) {
        // clear the TMR0 interrupt flag
        PIR3bits.TMR0IF = 0;
        timer0_isr_handler();

    } else if (PIE6bits.U2TXIE == 1 && PIR6bits.U2TXIF == 1) {
        // Not used
    } else if (PIE6bits.U2RXIE == 1 && PIR6bits.U2RXIF == 1) {

        if (U2ERRIRbits.FERIF) {
            U2ERRIRbits.FERIF = 0;
        }

        if (U2ERRIRbits.RXFOIF) {
            U2ERRIRbits.RXFOIF = 0;
        }

        uart2_isr_handler();

        PIR6bits.U2RXIF = 0;

    } else if (PIE0bits.IOCIE == 1 && PIR0bits.IOCIF == 1) {

        // interrupt on change for pin IOCAF0
        if (IOCAFbits.IOCAF0 == 1) {
            ra0_ioc_handler();
            IOCAFbits.IOCAF0 = 0;
        }
        // interrupt on change for pin IOCBF0
        if (IOCBFbits.IOCBF0 == 1) {
            rb0to3_ioc_handler();
            IOCBFbits.IOCBF0 = 0;
        }
        // interrupt on change for pin IOCBF1
        if (IOCBFbits.IOCBF1 == 1) {
            rb0to3_ioc_handler();
            IOCBFbits.IOCBF1 = 0;
        }
        // interrupt on change for pin IOCBF2
        if (IOCBFbits.IOCBF2 == 1) {
            rb0to3_ioc_handler();
            IOCBFbits.IOCBF2 = 0;
        }
        // interrupt on change for pin IOCBF3
        if (IOCBFbits.IOCBF3 == 1) {
            rb0to3_ioc_handler();
            IOCBFbits.IOCBF3 = 0;
        }

        // interrupt on change for pin IOCCF6
        if (IOCCFbits.IOCCF6 == 1) {
            IOCCFbits.IOCCF6 = 0;
#if (CONFIG_USE_LIMITED_FUNC == 1)
            // do nothing...
#else            
            rc6_interrupt_handler();
#endif
        }

        PIR0bits.IOCIF = 0;

    } else {
        //Unhandled Interrupt
    }
}
// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Main">

uint8_t msg_buffer[RF_PACKET_PAYLOAD_COUNT] = {0}; // [pressed key, 32bit time data]
rotary_position_t position = ROT_POS_INVALID;
recv_switch_t *receiver_switch;

#define SEC_TO_MS                               (1000U)
#define MIN_TO_SEC                              (60U)
#define AIR_LEARNING_SESSION_TIMEOUT            (15UL * SEC_TO_MS )

static uint32_t air_learning_time_stamp = 0;
static bool air_learning_id_enroll_done = 0;
static bool air_learning_enabled = 0;

static bool indicate_after_task = false;
static uint8_t rotary_pos_last = 0;
static uint32_t rotary_pos_last_tick = 0;

static login_t signup;
static int login_retry;
static uint32_t login_stamp;
static bool device_locked = false;

void rotary_switch_on_change_handler() {

    blink_led_async_stop(LED_RED);
    blink_led_async_stop(LED_GREEN);
    rotary_pos_last_tick = millis();
    indicate_after_task = false;
}

/**
 * To check remote transmitter de-bounce.
 * @param hold_time key hold duration.
 * @return pressed or not.
 */
bool is_key_pressed(uint32_t hold_time) {
    if ((hold_time >= 50 + 75) && (hold_time < 160 + 75)) {
        return true;
    }
    return false;
}

static login_t data;

bool rf_available = false;
bool tx_is_master = false;
int tx_index = ERR_RF_NOT_EXIST;
uint32_t pressed_time = 0;
transmitter_id_t tx;
static bool login_success = false;

uint32_t last_rf_available = 0;
uint32_t last_tx_id = 0;
static int user_retry_count = CONFIG_LOGIN_RETRY_COUNT;

uint8_t get_key_event() {

    if (rf_available && tx_index >= 0) {

        if (is_transmitter_is_master(tx.id)) {
            return msg_buffer[0];
        }

#if (CONFIG_USE_LIMITED_FUNC == 1)
        return msg_buffer[0];
#else
        //        if (login_success) {
        //            if (millis() - data.last_session > 2000) {
        //                login_success = false;
        //                memset(&data, 0, sizeof (data));
        //            }
        //        }

        if (system_get_user_pin() != CONFIG_PARAM_INVALID) {
            // have to passed by user pin.

            if (is_key_pressed(pressed_time)) {
                if (data.count < 4) {
                    data.password[data.count] = msg_buffer[0];
                    data.last_session = millis();


                    data.apply = true;
                } else {
                    DEBUG_MSG("[%02X %02X %02X %02X]\r\n", data.password[0], data.password[1], data.password[2], data.password[3]);
                    if (system_get_user_pin() == data.u32pass) {
                        data.key = msg_buffer[0];
                        DEBUG_MSG("User pin correct... key is %02X\r\n", data.key);
                        login_success = true;
                        memset(&data, 0, sizeof (data));
                        user_retry_count = CONFIG_LOGIN_RETRY_COUNT;
                        return msg_buffer[0];
                    } else {
                        DEBUG_MSG("User pin incorrect... key is %02X\r\n", data.key);
                        login_success = false;
                        memset(&data, 0, sizeof (data));
                        user_retry_count--;
                        DEBUG_MSG("Retry count %d\r\n", user_retry_count);
                        if (user_retry_count <= 0) {
                            user_blocked_timer = CONFIG_LOGIN_BLOCK_TIMEOUT;
                            system_set_block_timeout(CONFIG_LOGIN_BLOCK_TIMEOUT);
                        }
                        return 0;
                    }
                }
                data.count++;
            } else {
                DEBUG_MSG("Else %u\r\n", login_success);
                if (pressed_time > 195 && login_success == true) {
                    DEBUG_MSG(" Alive key is %02X\r\n", msg_buffer[0]);
                    data.last_session = millis();
                    return msg_buffer[0];
                }
            }
        } else {
            // not required.
            DEBUG_MSG(" key is %02X\r\n", msg_buffer[0]);
            return msg_buffer[0];
        }
#endif
    } else {
        if (millis() - data.last_session > 5000 && data.count > 0) {
            login_success = false;
            memset(&data, 0, sizeof (data));
        }
    }
    return 0;
}

bool verify_counter_value() {

    if (tx_is_master) {
        return true;
    }
    if ((last_tx_id != tx.id) || (millis() - last_rf_available > 300)) {
        last_rf_available = millis();

        counter_t *temp_counter;
        uint8_t temp_counter_u8[RF_PACKET_COUNTER_SIZE];
        memcpy(temp_counter_u8, system_cfg.transmitter[tx_index].counter, sizeof (temp_counter_u8));
        temp_counter = (counter_t*) & temp_counter_u8[0];

#if (DEBUG_ENABLE==1)
        DEBUG_MSG("Recv Counter: ");
        for (uint8_t i = 0; i<sizeof (counter_t); i++)DEBUG_MSG("%02X ", recv_counter_value[i]);
        DEBUG_MSG("\r\n");
#endif        
        // have to check counter...
        for (uint32_t i = 0; i < COUNTER_CHECK_RANGE; i++) {

            last_tx_id = tx.id;
            if (memcmp(temp_counter_u8, recv_counter_value, sizeof (temp_counter_u8)) == 0) {
                memcpy(system_cfg.transmitter[tx_index].counter, temp_counter_u8, sizeof (temp_counter_u8));

                system_cfg_last_update = millis();
                system_cfg_update = true;

                return true;

            } else {
                counter_inc_by_n(temp_counter, CONSTANT_INC_BY_UINT16_VAL);
#if (DEBUG_ENABLE==1)
                DEBUG_MSG("Checking %0d: ", i);
                for (uint8_t i = 0; i<sizeof (counter_t); i++)DEBUG_MSG("%02X ", temp_counter_u8[i]);
                DEBUG_MSG("\r\n");
#endif 
            }

        }
        return false;
    }
    return true;
}

void normal_mode_task() {

    if (rf_available > 0) {
        if (millis() - last_blink_yellow > 200) {
            blink_led_async(LED_YELLOW, 100, 200);
            last_blink_yellow = millis();
        }

        if (tx_index >= 0) {
            if (verify_counter_value() == false) {
                DEBUG_MSG("Invalid counter value");
                return;
            }
            uint8_t mode = 0;
            uint8_t key = get_key_event();

            if (tx_index >= 0) {

                if (system_cfg.transmitter[tx_index].key != CONFIG_PARAM_INVALID) {
                    if (key >= BUTTON_1 && key <= BUTTON_4) {
                        if (key != (uint8_t) system_cfg.transmitter[tx_index].key) {
                            key = 0;
                        } else {
                            key = BUTTON_1;
                        }
                    }
                }



                switch (key) {
                    case 0:
                    {
                        break;
                    }

                    case BUTTON_1:
                    {
                        mode = relay_get_mode(RELAY_01);
                        DEBUG_MSG("Relay 1 mode : 0x%02x > %lu\r\n", mode, pressed_time);

                        if (mode == RLY_DISABLED) {
                            // Disabled...
                            DEBUG_MSG("Disabled\r\n");
                        } else if (mode == RLY_MOMENTARY_MODE) {
                            relay_pulse_on(RELAY_01, 1000);
                        } else if (mode == RLY_LATCH0_MODE) {
                            if (is_key_pressed(pressed_time))
                                RELAY0_SetToggle();
                        } else if (mode == RLY_LATCH1_MODE) {
                            if (is_key_pressed(pressed_time))
                                RELAY0_SetHigh();
                        } else if (mode == RLY_PULSE_MODE) {
                            if (is_key_pressed(pressed_time))
                                relay_pulse_on(RELAY_01, -1);
                        }
                    }
                        break;
                    case BUTTON_2:
                    {
                        mode = relay_get_mode(RELAY_02);
                        DEBUG_MSG("Relay 2 mode : 0x%02x\r\n", mode);
                        if (mode == RLY_DISABLED) {
                            // Disabled...
                            DEBUG_MSG("Disabled\r\n");
                        } else if (mode == RLY_MOMENTARY_MODE) {
                            relay_pulse_on(RELAY_02, 1000);
                        } else if (mode == RLY_LATCH0_MODE) {
                            if (is_key_pressed(pressed_time))
                                RELAY1_SetToggle();
                        } else if (mode == RLY_LATCH1_MODE) {
                            if (is_key_pressed(pressed_time))
                                RELAY0_SetLow();
                        } else if (mode == RLY_PULSE_MODE) {
                            if (is_key_pressed(pressed_time))
                                relay_pulse_on(RELAY_02, -1);
                        }
                    }
                        break;
                    case BUTTON_3:
                    {
                        mode = relay_get_mode(RELAY_03);
                        DEBUG_MSG("Relay 3 mode : 0x%02x\r\n", mode);
                        if (mode == RLY_DISABLED) {
                            // Disabled...
                            DEBUG_MSG("Disabled\r\n");
                        } else if (mode == RLY_MOMENTARY_MODE) {
                            relay_pulse_on(RELAY_03, 1000);
                        } else if (mode == RLY_LATCH0_MODE) {
                            if (is_key_pressed(pressed_time))
                                RELAY2_SetToggle();
                        } else if (mode == RLY_LATCH1_MODE) {
                            if (is_key_pressed(pressed_time))
                                RELAY1_SetHigh();
                        } else if (mode == RLY_PULSE_MODE) {
                            if (is_key_pressed(pressed_time))
                                relay_pulse_on(RELAY_03, -1);
                        }
                    }
                        break;
                    case BUTTON_4:
                    {
                        mode = relay_get_mode(RELAY_04);
                        DEBUG_MSG("Relay 4 mode : 0x%02x\r\n", mode);

                        if (mode == RLY_DISABLED) {
                            // Disabled...
                            DEBUG_MSG("Disabled\r\n");
                        } else if (mode == RLY_MOMENTARY_MODE) {
                            relay_pulse_on(RELAY_04, 1000);
                        } else if (mode == RLY_LATCH0_MODE) {
                            if (is_key_pressed(pressed_time))
                                RELAY3_SetToggle();
                        } else if (mode == RLY_LATCH1_MODE) {
                            if (is_key_pressed(pressed_time))
                                RELAY1_SetLow();
                        } else if (mode == RLY_PULSE_MODE) {
                            if (is_key_pressed(pressed_time))
                                relay_pulse_on(RELAY_04, -1);
                        }
                    }
                        break;

                    default:
                    {
                    }
                        break;
                }

            } else {
                if (millis() - last_blink_red > 200) {
                    blink_led_async(LED_RED, 50, 200);
                    last_blink_yellow = millis();
                }
                DEBUG_MSG("[Not authorized] Transmitter id :0x%lx\r\n", tx.id);
            }
        }
    }
}

void main(void) {
    system_initialize();
    receiver_switch = recv_switch_begin();
    rotary_switch_begin();
    rf_receiver_begin();
    blink_init();
    eep_begin();
    relay_begin();
    INTCON0bits.GIE = 1;

    system_config_begin();

    blink_led_async(LED_YELLOW, 50, 1000);
    delay_ms(1000);

    DEBUG_MSG("\r\n********* Parameter *********\r\n");
    DEBUG_MSG("> System user pin            :0x%lX\r\n", system_get_user_pin());
    DEBUG_MSG("> System technician pin      :0x%lX\r\n", system_get_technician_pin());
    DEBUG_MSG("> Registered Transmitter 0   :0x%lX\r\n", system_cfg.transmitter[0].id);
    DEBUG_MSG("> Registered Transmitter 1   :0x%lX\r\n", system_cfg.transmitter[1].id);
    DEBUG_MSG("> Registered Transmitter 2   :0x%lX\r\n", system_cfg.transmitter[2].id);
    DEBUG_MSG("> Registered Transmitter 3   :0x%lX\r\n\r\n", system_cfg.transmitter[3].id);

    DEBUG_MSG("> Relay1 Trigger Wait        :0x%lx\r\n", relay_get_trigger_wait());
    DEBUG_MSG("> Relay1 Trigger Width       :0x%lx\r\n", relay_get_trigger_width());
    DEBUG_MSG("> Relay1 Trigger Edge        :0x%x\r\n\r\n", relay_get_trigger_edge());

    DEBUG_MSG("> Relay1 Activation Mode     :0x%02x\r\n", relay_get_mode(RELAY_01));
    DEBUG_MSG("> Relay2 Activation Mode     :0x%02x\r\n", relay_get_mode(RELAY_02));
    DEBUG_MSG("> Relay3 Activation Mode     :0x%02x\r\n", relay_get_mode(RELAY_03));
    DEBUG_MSG("> Relay4 Activation Mode     :0x%02x\r\n\r\n", relay_get_mode(RELAY_04));
    DEBUG_MSG("****************************\r\n");

    DEBUG_MSG("\r\nReceiver started\r\n");

    last_blink_red = 0;
    last_blink_green = 0;
    last_blink_yellow = 0;

    position = rotary_position_now();
    rotary_pos_last = ROT_POS_INVALID;
    login_stamp = 0;
    login_retry = CONFIG_LOGIN_RETRY_COUNT;
    user_retry_count = CONFIG_LOGIN_RETRY_COUNT;
    bool test_mode =  false;
    device_locked = ((system_get_technician_pin() == CONFIG_PARAM_INVALID) ? false : true);
    user_blocked_timer = system_get_block_timeout();

    DEBUG_MSG("System block timeout %lu\r\n", user_blocked_timer);
    while (1) {

        if (user_blocked_timer <= 0) {
            rf_available = false;
            tx_index = ERR_RF_NOT_EXIST;
            pressed_time = 0;
            tx_is_master = false;
            if (millis() - rotary_pos_last_tick >= 500) {
                if (rotary_pos_last != rotary_position_now()) {
                    uint8_t p = rotary_position_now();

                    if (p <= ROT_POS_9) {
                        position = p;
                    }

                    if (position == 0) {
                        blink_led_async(LED_GREEN, 1500, 3000);
                    } else {
                        blink_led_async(LED_GREEN, 300, position * 600);
                    }
                    DEBUG_MSG("\r\nRotary Pos Changed : %d\r\n", position);

                    if (position == ROT_POS_7) {
                        memset(&signup, 0, sizeof (login_t));
                    }

                    rotary_pos_last = position;
                }
                rotary_pos_last_tick = millis();
            }
            if (rf_receiver_available()) {
                tx.id = rf_receiver_get_tx_id();
                tx_index = eep_check_tx_id(tx.id);

                tx_is_master = is_transmitter_is_master(tx.id);
                rf_receiver_get_msg(msg_buffer, sizeof (msg_buffer));
                memcpy(&pressed_time, &msg_buffer[1], sizeof (pressed_time));

                rf_available = true;

                if (pressed_time == 0) {
                    last_blink_yellow = 0;
                    last_blink_red = 0;
                    last_blink_green = 0;
                }
            }

            if ((rf_available == true) && ((position > ROT_POS_3) || ((position > ROT_POS_3)))) {
                if (!verify_counter_value()) {
                    rf_available = false;
                }
            }

            if (air_learning_enabled) {
                if (position != ROT_POS_0) air_learning_enabled = false;

                if ((uint32_t) (millis() - air_learning_time_stamp) <= AIR_LEARNING_SESSION_TIMEOUT) {
                    // Learn ID.
                    if (rf_available) {
                        if (tx_index >= 0) {
                            normal_mode_task();
                        } else {
                            tx.key = CONFIG_PARAM_INVALID;
                            memcpy(tx.counter, recv_counter_value, sizeof (recv_counter_value));

                            int err = eep_remember_tx_id(&tx);
                            if (err == ERR_RF_STORAGE_FULL) {
                                blink_led_async(LED_ALTER, 100, 3000);
                            } else {
                                blink_led_async(LED_GREEN, -1, 3000);
                            }
                            air_learning_enabled = false;
                            delay_ms(3000);
                            DEBUG_MSG("Transmitter id :0x%lx Added rc:%d\r\n", tx.id, err);
                            DEBUG_MSG("exiting from air learning mode : device enrolled\r\n");
                        }
                    }
                } else {
                    air_learning_enabled = false;
                    DEBUG_MSG("exiting from air learning mode : timeout\r\n");
                }



            } else {

                // Check device is locked or not...
                // If locked only access case 7 + pressed...
                rotary_position_t access_case = position;

#if (CONFIG_USE_LIMITED_FUNC == 1)
                if ((access_case != ROT_POS_0) &&
                        (access_case != ROT_POS_5)) {
                    access_case = ROT_POS_INVALID;
                }
#else
                if ((device_locked == true) && (tx_is_master == false)) {
                    if ((position != ROT_POS_7) && (position != ROT_POS_0)) {
                        access_case = ROT_POS_INVALID;
                    }
                }
#endif
                switch (access_case) {
                    case ROT_POS_0:
                    {
                        if (millis() - data.last_session > 5000 && data.count > 0) {
                            login_success = false;
                            memset(&data, 0, sizeof (data));
                        }


                        if (millis() - last_blink_green >= 3000) {
                            blink_led_async(LED_GREEN, 100, 200);
                            last_blink_green = millis();
                        }

                        // Normal task..
                        if (RECV_SW_GET() == SW_PRESSED) {

                            if ((device_locked == true) && (tx_is_master == false)) {
                                if (receiver_switch->event) {
                                    receiver_switch->event = 0;
                                }
                                DEBUG_MSG("Device locked...Not allowed");
                                blink_led_async(LED_ALTER, 500, 3000);
                                delay_ms(3000);
                                break;
                            }

                            // PUSH button is pressed >3sec
                            if (receiver_switch->pressed_time > 0) {
                                uint32_t pressed_time = millis() - receiver_switch->pressed_time;

                                if (millis() - last_blink_yellow > 200) {
                                    blink_led_async(LED_YELLOW, 100, 200);
                                    last_blink_yellow = millis();
                                }

                                if (pressed_time >= 3000) {

                                    if (rf_available) {
                                        if (tx_index >= 0) {
                                            blink_led_async(LED_GREEN, -1, 3000);
                                            delay_ms(3000);
                                            DEBUG_MSG("Transmitter id :0x%lx Already exist\r\n", tx.id);
                                        } else {
                                            memcpy(tx.counter, recv_counter_value, sizeof (recv_counter_value));

                                            blink_led_async(LED_YELLOW, 100, -1);
                                            tx.key = CONFIG_PARAM_INVALID;
                                            int err = eep_remember_tx_id(&tx);
                                            blink_led_async_stop(LED_YELLOW);
                                            relay_pulse_on(RELAY_01, 3000);
                                            if (err == ERR_RF_STORAGE_FULL) {
                                                blink_led_async(LED_ALTER, 100, 3000);
                                            } else {
                                                blink_led_async(LED_GREEN, 100, 3000);
                                            }
                                            delay_ms(3000);
                                            DEBUG_MSG("Transmitter id :0x%lx Added rc:%d\r\n", tx.id, err);
                                        }
                                    }
                                }
                            }
                        } else {
                            // PUSH button is not pressed...
                            if (receiver_switch->event) {
                                receiver_switch->event = 0;
                            }
                            
                            if(JUMPER1_GetValue() == JUMPER_ON && JUMPER2_GetValue() == JUMPER_ON) {
                                DEBUG_MSG("in a Testing Mode \r\n");
                                test_mode = true;
                                
                            }
                                

                            if (rf_available) {
                                if (JUMPER1_GetValue() == JUMPER_ON) {

                                    if ((device_locked == true) && (tx_is_master == false)) {
                                        DEBUG_MSG("Device locked...Not allowed");
                                        blink_led_async(LED_ALTER, 500, 3000);
                                        delay_ms(3000);
                                        break;
                                    }

                                    if (tx_index >= 0 && (msg_buffer[0] == 0xAA)) {
                                        DEBUG_MSG("Enter into air-learning mode..\r\n");
                                        air_learning_time_stamp = millis();
                                        air_learning_enabled = true;
                                        blink_led(LED_GREEN, 3000);
                                    } else {
                                        if (millis() - last_blink_yellow > 200) {
                                            blink_led_async(LED_YELLOW, 100, 200);
                                            last_blink_yellow = millis();
                                        }
                                        DEBUG_MSG("invalid code for air learning...\r\n");
                                    }

                                }
                                // Normal mode.
                                normal_mode_task();
                            }
                        }
                    }
                        break;

                    case ROT_POS_1:
                    {
                        if (rf_available) {
                            if (RECV_SW_GET() == SW_PRESSED) {
                                // Pressed
                                if (tx_index >= 0 && tx.key == msg_buffer[0]) {

                                    if (millis() - last_blink_green > 3000) {
                                        blink_led_async(LED_GREEN, -1, 3000);
                                        last_blink_green = millis();
                                    }

                                    DEBUG_MSG("Transmitter id :0x%lx Already exist\r\n", tx.id);
                                } else {
                                    blink_led_async(LED_YELLOW, 100, -1);
                                    tx.key = msg_buffer[0];

                                    memcpy(tx.counter, recv_counter_value, sizeof (recv_counter_value));

                                    int err = eep_remember_tx_id(&tx);
                                    blink_led_async_stop(LED_YELLOW);
                                    relay_pulse_on(RELAY_01, 3000);
                                    if (err == ERR_RF_STORAGE_FULL) {
                                        blink_led_async(LED_ALTER, 100, 3000);
                                    } else {
                                        blink_led_async(LED_GREEN, 100, 3000);
                                    }
                                    delay_ms(3000);
                                    DEBUG_MSG("Transmitter id :0x%lx Added rc:%d\r\n", tx.id, err);
                                }

                            } else {
                                // Not pressed
                                if (tx_index >= 0) {
                                    if (millis() - last_blink_green > 3000) {
                                        blink_led_async(LED_GREEN, -1, 3000);
                                        last_blink_green = millis();
                                    }
                                    DEBUG_MSG("Transmitter id :0x%lx Already exist\r\n", tx.id);
                                } else {
                                    blink_led_async(LED_YELLOW, 100, -1);
                                    tx.key = CONFIG_PARAM_INVALID;

                                    memcpy(tx.counter, recv_counter_value, sizeof (recv_counter_value));

                                    int err = eep_remember_tx_id(&tx);
                                    blink_led_async_stop(LED_YELLOW);
                                    relay_pulse_on(RELAY_01, 3000);
                                    if (err == ERR_RF_STORAGE_FULL) {
                                        blink_led_async(LED_ALTER, 100, 3000);
                                    } else {
                                        blink_led_async(LED_GREEN, 100, 3000);
                                    }
                                    delay_ms(3000);
                                    DEBUG_MSG("Transmitter id :0x%lx Added rc:%d\r\n", tx.id, err);
                                }
                            }
                        }
                    }
                        break;

                    case ROT_POS_2:
                    {
                        // Forget ID.
                        if (rf_available) {
                            if (tx_index >= 0) {
                                blink_led_async(LED_YELLOW, 100, -1);
                                int err = eep_forget_tx_id(tx.id);
                                DEBUG_MSG("Forgetting Transmitter id :0x%lx rc :%d\r\n", tx.id, err);
                                blink_led_async_stop(LED_YELLOW);
                                blink_led(LED_RED, 3000);
                            } else {
                                if (millis() - last_blink_red > 3000) {
                                    blink_led_async(LED_RED, -1, 3000);
                                    last_blink_red = millis();
                                }
                                DEBUG_MSG("Transmitter id :0x%lx Not exist.\r\n", tx.id);
                            }
                        }
                    }
                        break;

                    case ROT_POS_3:
                    {
                        if (RECV_SW_GET() == SW_PRESSED) {

                            if (millis() - last_blink_yellow > 200) {
                                blink_led_async(LED_YELLOW, 100, 200);
                                last_blink_yellow = millis();
                            }

                            if (receiver_switch->pressed_time > 0) {
                                uint32_t pressed_time = millis() - receiver_switch->pressed_time;

                                if (pressed_time >= 3000) {
                                    blink_led_async_stop(LED_YELLOW);
                                    blink_led_async(LED_RED, 100, 3000);
                                    eep_erase();
                                    system_config_begin();
                                    DEBUG_MSG("EEPROM Erase Done\r\n");
                                    delay_ms(3000);
                                }
                            }
                        } else {
                            if (receiver_switch->event) {
                                receiver_switch->event = 0;
                            }
                        }
                    }
                        break;

                    case ROT_POS_4:
                    {
                        if (rf_available && tx_index >= 0) {

                            indicate_after_task = true;
                            last_blink_green = millis();

                            if (millis() - last_blink_yellow > 200) {
                                blink_led_async(LED_YELLOW, 100, 200);
                                last_blink_yellow = millis();
                            }

                            uint8_t jmp_state = 0;
                            jmp_state = (JUMPER3_GetValue() == JUMPER_ON ? 1 : 0);
                            jmp_state = (uint8_t) (jmp_state << 1);
                            jmp_state |= (JUMPER2_GetValue() == JUMPER_ON ? 1 : 0);

                            switch (jmp_state) {
                                case 0b00:
                                {
                                    // Jump2 off, Jump3 off.
                                    DEBUG_MSG("> Setting pulse mode with pulse time => Relay#%d =>%u ms\r\n", msg_buffer[0] - 0xA0, pressed_time);
                                    relay_set_mode(msg_buffer[0] - BUTTON_1, RLY_PULSE_MODE);

                                    switch (msg_buffer[0]) {
                                        case BUTTON_1: relay_set_pulse_time(RELAY_01, pressed_time);
                                            break;
                                        case BUTTON_2: relay_set_pulse_time(RELAY_02, pressed_time);
                                            break;
                                        case BUTTON_3: relay_set_pulse_time(RELAY_03, pressed_time);
                                            break;
                                        case BUTTON_4: relay_set_pulse_time(RELAY_04, pressed_time);
                                            break;
                                    }

                                }
                                    break;

                                case 0b01:
                                {
                                    // Jump2 on, Jump3 off.
                                    DEBUG_MSG(" Setting Mode Latch0 for Relay#%d\r\n", msg_buffer[0] - 0xA0);
                                    relay_set_mode(msg_buffer[0] - BUTTON_1, RLY_LATCH0_MODE);

                                }
                                    break;

                                case 0b10:
                                {
                                    // Jump2 off, Jump3 on.

                                    // Button 0 or 1 pressed or both pressed
                                    if ((msg_buffer[0] == BUTTON12_PRESSED_SYMBOL) || (msg_buffer[0] - 0xA0) <= 0x02) {
                                        relay_set_mode(RELAY_01, RLY_LATCH1_MODE);
                                        relay_set_mode(RELAY_02, RLY_LATCH1_MODE);
                                        DEBUG_MSG(" Setting Mode Latch1 for Relay#0\r\n");
                                    }

                                    // Button 2 or 3 pressed or both pressed
                                    if ((msg_buffer[0] == BUTTON34_PRESSED_SYMBOL) || (((msg_buffer[0] - 0xA0) >= 0x03) && (msg_buffer[0] - 0xA0) <= 0x04)) {
                                        relay_set_mode(RELAY_03, RLY_LATCH1_MODE);
                                        relay_set_mode(RELAY_04, RLY_LATCH1_MODE);
                                        DEBUG_MSG(" Setting Mode Latch1 for Relay#1\r\n");
                                    }

                                }
                                    break;

                                case 0b11:
                                {
                                    // Jump2 on, Jump3 on.
                                    relay_set_mode(msg_buffer[0] - BUTTON_1, RLY_MOMENTARY_MODE);
                                    DEBUG_MSG(" Setting Mode Momentary for Relay#%d\r\n", msg_buffer[0] - 0xA0);
                                }
                                    break;
                            }

                        } else {
                            if (indicate_after_task) {
                                if (millis() - last_blink_green > 1000) {
                                    blink_led_async(LED_GREEN, 100, 3000);
                                    indicate_after_task = false;
                                }
                            }
                        }
                    }
                        break;

                    case ROT_POS_5:
                    {
                        static bool task = 0;

                        if (RECV_SW_GET() == SW_PRESSED) {

                            if (!task) {
                                blink_led_async(LED_YELLOW, 100, 10000);
                                task = true;
                            }

                            if (receiver_switch->pressed_time > 0) {
                                uint32_t pressed_time = millis() - receiver_switch->pressed_time;

                                // Forget all transmitter...
                                if (pressed_time >= 5000) {
                                    DEBUG_MSG("Pressed for %u \r\n", pressed_time);
                                    DEBUG_MSG("Forgetting all transmitter...\r\n");
                                    eep_forget_all();
                                    if (task) {
                                        blink_led_async_stop(LED_YELLOW);
                                        task = false;
                                    }
                                    blink_led_async(LED_RED, 100, 3000);
                                    while (!RECV_SW_GET()) {
                                        if (rotary_position_now() != ROT_POS_5) break;
                                    }
                                }
                            }
                        } else {

                            if (task) {
                                blink_led_async_stop(LED_YELLOW);
                                task = false;
                            }

                            if (receiver_switch->event) {
                                receiver_switch->event = 0;
                            }
                        }

                    }
                        break;

                    case ROT_POS_6:
                    {
                        if (rf_available) {

                            if (millis() - last_blink_yellow > 200) {
                                blink_led_async(LED_YELLOW, 100, 200);
                                last_blink_yellow = millis();
                            }

                            if (tx_index >= 0) {

                                indicate_after_task = true;
                                last_blink_green = millis();

                                switch (msg_buffer[0]) {
                                    case BUTTON_1:
                                    {
                                        relay_set_trigger_wait(pressed_time);
                                        DEBUG_MSG("> Setting Trigger wait time. =>%u ms\r\n", pressed_time);
                                        break;
                                    }
                                    case BUTTON_2:
                                    {
                                        relay_set_trigger_width(pressed_time);
                                        DEBUG_MSG("> Setting Trigger width time =>%u ms\r\n", pressed_time);
                                        break;
                                    }
                                    case BUTTON_3:
                                    {
                                        relay_set_trigger_edge(TRIGGER_ACTIVE_LOW);
                                        DEBUG_MSG("> Setting Trigger active low\r\n");
                                        break;
                                    }
                                    case BUTTON_4:
                                    {
                                        relay_set_trigger_edge(TRIGGER_ACTIVE_HIGH);
                                        DEBUG_MSG("> Setting Trigger active High\r\n");
                                        break;
                                    }
                                }
                            }
                        } else {
                            if (indicate_after_task) {
                                if (millis() - last_blink_green > 1000) {
                                    blink_led_async(LED_GREEN, 100, 3000);
                                    indicate_after_task = false;
                                }
                            }
                        }

                    }
                        break;

                    case ROT_POS_7:
                    {
                        if ((device_locked == true) &&
                                (RECV_SW_GET() == SW_NOT_PRESSED)) {
                            if (!tx_is_master) {
                                break;
                            }
                        }

                        if (rf_available && tx_index >= 0) {

                            if (millis() - last_blink_yellow > 200) {
                                blink_led_async(LED_YELLOW, 100, 200);
                                last_blink_yellow = millis();
                            }

                            if ((tx_is_master == true) && (msg_buffer[0] == BUTTON12_PRESSED_SYMBOL)) {

                                DEBUG_MSG("> Master mode...\r\n");
                                DEBUG_MSG("> Disabled learning lock...\r\n");
                                system_set_technician_pin(CONFIG_PARAM_INVALID);
                                blink_led_async(LED_GREEN, 100, 3000);
                                device_locked = false;
                                delay_ms(3000);

                            } else {

                                if (is_key_pressed(pressed_time)) {

                                    signup.password[signup.count] = msg_buffer[0];
                                    signup.last_session = millis();

                                    signup.count++;
                                    signup.apply = true;
                                    if (signup.count >= 4) {

                                        if (RECV_SW_GET() == SW_PRESSED) {
                                            if (login_retry > 0) {
                                                if (system_get_technician_pin() == CONFIG_PARAM_INVALID) {
                                                    // No pin registered 
                                                    // Register new pin.
                                                    // System locked...
                                                    DEBUG_MSG("> Setting learning lock...\r\n");
                                                    system_set_technician_pin(signup.u32pass);
                                                    blink_led_async(LED_RED, 100, 3000);
                                                    device_locked = true;
                                                    delay_ms(3000);
                                                } else {
                                                    // Try to unlock system with entered pin with retry logic
                                                    if (signup.u32pass == system_get_technician_pin()) {
                                                        // correct pin..
                                                        DEBUG_MSG("> Disabled learning lock...\r\n");
                                                        system_set_technician_pin(CONFIG_PARAM_INVALID);
                                                        blink_led_async(LED_GREEN, 100, 3000);
                                                        device_locked = false;
                                                        delay_ms(3000);
                                                    } else {
                                                        // incorrect pin...
                                                        login_retry--;
                                                        DEBUG_MSG("> Incorrect technician code..\r\n");
                                                        DEBUG_MSG("> Retry count.. %d\r\n", login_retry);
                                                    }
                                                    login_stamp = millis();
                                                }
                                            } else {
                                                DEBUG_MSG("> Login blocked for [%u] seconds\r\n", (uint32_t) (CONFIG_LOGIN_BLOCK_TIMEOUT - (millis() - login_stamp)));
                                            }
                                        } else {
                                            if (signup.u32pass != system_get_user_pin()) {
                                                system_set_user_pin(signup.u32pass);
                                                blink_led_async(LED_GREEN, 100, 3000);
                                            } else {
                                                blink_led_async(LED_GREEN, -1, 3000);
                                            }

                                            DEBUG_MSG("> User pin: %02x %02x %02x %02x\r\n",
                                                    signup.password[0], signup.password[1],
                                                    signup.password[2], signup.password[3]);
                                            delay_ms(3000);
                                        }
                                        memset(&signup, 0, sizeof (signup));
                                    }
                                }
                            }
                        } else {
                            // no activity from 5 sec
                            // Make release all keys and start from scratch.
                            if (signup.apply) {
                                if (millis() - signup.last_session >= 5000) {
                                    DEBUG_MSG("> User pin set timeout occur\r\n");
                                    memset(&signup, 0, sizeof (signup));
                                }
                            }
                        }

                    }
                        break;

                    case ROT_POS_8:
                    {
                        if (RECV_SW_GET() == SW_PRESSED) {

                            if (millis() - last_blink_yellow > 200) {
                                blink_led_async(LED_YELLOW, 100, 200);
                                last_blink_yellow = millis();
                            }

                            if (receiver_switch->pressed_time > 0) {
                                uint32_t pressed_time = millis() - receiver_switch->pressed_time;

                                // Forget all transmitter...
                                if (pressed_time >= 3000) {
                                    DEBUG_MSG("Pressed for %u \r\n", pressed_time);
                                    DEBUG_MSG("Forgetting user pin...\r\n");

                                    if (system_get_user_pin() != CONFIG_PARAM_INVALID) {
                                        system_set_user_pin(CONFIG_PARAM_INVALID);
                                        blink_led_async(LED_RED, 100, 3000);
                                    } else {
                                        blink_led_async(LED_RED, -1, 3000);
                                    }

                                    delay_ms(3000);

                                    while (!RECV_SW_GET()) {
                                        if (rotary_position_now() != ROT_POS_8) break;
                                    }
                                }
                            }
                        } else {
                            if (receiver_switch->event) {
                                receiver_switch->event = 0;
                            }
                        }
                    }
                        break;

                    case ROT_POS_9:
                    {
                        if (RECV_SW_GET() == SW_PRESSED) {
                            if (rf_available) {

                                if (millis() - last_blink_yellow > 200) {
                                    blink_led_async(LED_YELLOW, 100, 200);
                                    last_blink_yellow = millis();
                                }

                                if (tx_index >= 0) {
                                    if (pressed_time >= 3000 && pressed_time <= 3300) {

                                        switch (msg_buffer[0]) {
                                            case BUTTON_1:
                                            {
                                                DEBUG_MSG("Forgot pulse time Relay#%d\r\n", msg_buffer[0] - 0xA0);
                                                if (relay_get_mode(RELAY_01) != RLY_DISABLED) {
                                                    relay_set_mode(RELAY_01, RLY_DISABLED);
                                                    blink_led_async(LED_RED, 100, 3000);
                                                } else {
                                                    blink_led_async(LED_RED, -1, 3000);
                                                }
                                                delay_ms(3000);
                                                break;
                                            }
                                            case BUTTON_2:
                                            {
                                                DEBUG_MSG("Forgot pulse time Relay#%d\r\n", msg_buffer[0] - 0xA0);
                                                if (relay_get_mode(RELAY_02) != RLY_DISABLED) {
                                                    relay_set_mode(RELAY_02, RLY_DISABLED);
                                                    blink_led_async(LED_RED, 100, 3000);
                                                } else {
                                                    blink_led_async(LED_RED, -1, 3000);
                                                }
                                                delay_ms(3000);
                                                break;
                                            }
                                            case BUTTON_3:
                                            {
                                                DEBUG_MSG("Forgot pulse time Relay#%d\r\n", msg_buffer[0] - 0xA0);
                                                if (relay_get_mode(RELAY_03) != RLY_DISABLED) {
                                                    relay_set_mode(RELAY_03, RLY_DISABLED);
                                                    blink_led_async(LED_RED, 100, 3000);
                                                } else {
                                                    blink_led_async(LED_RED, -1, 3000);
                                                }
                                                delay_ms(3000);
                                                break;
                                            }
                                            case BUTTON_4:
                                            {
                                                DEBUG_MSG("Forgot pulse time Relay#%d\r\n", msg_buffer[0] - 0xA0);
                                                if (relay_get_mode(RELAY_04) != RLY_DISABLED) {
                                                    relay_set_mode(RELAY_04, RLY_DISABLED);
                                                    blink_led_async(LED_RED, 100, 3000);
                                                } else {
                                                    blink_led_async(LED_RED, -1, 3000);
                                                }
                                                delay_ms(3000);
                                                break;
                                            }
                                        }

                                    }
                                }
                            }
                        }
                    }
                        break;

                    default:
                    {
                        if (rf_available > 0 && tx_index >= 0) {
                            DEBUG_MSG("> **************\r\n");
                            DEBUG_MSG("> Invalid event... \r\n");
                            blink_led_async(LED_ALTER, 500, 3000);
                            delay_ms(3000);
                        }
                    }
                        break;
                }
            }
            
     

            if (login_retry == 0) {
                if ((uint32_t) (millis() - login_stamp) >= CONFIG_LOGIN_BLOCK_TIMEOUT) {
                    login_retry = CONFIG_LOGIN_RETRY_COUNT;
                }
            }

        } else {
            delay_ms(CONFIG_LOGIN_BLOCK_UPDATE_RATE);
            user_retry_count = CONFIG_LOGIN_RETRY_COUNT;
            user_blocked_timer = user_blocked_timer - CONFIG_LOGIN_BLOCK_UPDATE_RATE;
            if (user_blocked_timer < 1000)user_blocked_timer = 0;
            system_set_block_timeout(user_blocked_timer);
        }

        if (test_mode) {
            DEBUG_MSG("Turn rotaty to 1 for DS 1 Mode \r\n");
            while (!position) {
                position = rotary_position_now();
                bool task_done = false;
                while (position==1) {
                    if(JUMPER1_GetValue() == JUMPER_ON && JUMPER2_GetValue() == JUMPER_ON){
                        DEBUG_MSG("DS 1 Mode Set. \r\n");
                        DEBUG_MSG("Pressed  Push Button and Press  the button on the transmitter for activate Relay #0 \r\n ");
                        DEBUG_MSG("if you only Pressed button on Transmitter it will Remember the transmitter ID.\r\n");
                        while(!rf_receiver_available());
                            if (rf_receiver_available()) {
                               
                                tx.id = rf_receiver_get_tx_id();
                                tx_index = eep_check_tx_id(tx.id);
                                tx_is_master = is_transmitter_is_master(tx.id);
                                rf_receiver_get_msg(msg_buffer, sizeof (msg_buffer));
                                memcpy(&pressed_time, &msg_buffer[1], sizeof (pressed_time));
                                rf_available = true;
                                
                                if (pressed_time == 0) {
                                    last_blink_yellow = 0;
                                    last_blink_red = 0;
                                    last_blink_green = 0;
                                }
                            }
                            if (rf_available) {
                                
                                if (RECV_SW_GET() == SW_PRESSED) {
                                    // Pressed
                                    if (tx_index >= 0 && tx.key == msg_buffer[0]) {

                                        if (millis() - last_blink_green > 3000) {
                                            blink_led_async(LED_GREEN, -1, 3000);
                                            last_blink_green = millis();
                                        }

                                        DEBUG_MSG("Transmitter id :0x%lx Already exist\r\n", tx.id);
                                    } else {
                                        blink_led_async(LED_YELLOW, 100, -1);
                                        tx.key = msg_buffer[0];

                                        memcpy(tx.counter, recv_counter_value, sizeof (recv_counter_value));

                                        int err = eep_remember_tx_id(&tx);
                                        blink_led_async_stop(LED_YELLOW);
                                        relay_pulse_on(RELAY_01, 3000);
                                        if (err == ERR_RF_STORAGE_FULL) {
                                            blink_led_async(LED_ALTER, 100, 3000);
                                        } else {
                                            blink_led_async(LED_GREEN, 100, 3000);
                                        }
                                        delay_ms(3000);
                                        DEBUG_MSG("Transmitter id :0x%lx Added rc:%d\r\n", tx.id, err);
                                    }
                                    task_done = true;
                                    if(task_done) {
                                        break;
                                    }
                                } else {
                                    // Not pressed
                                    if (tx_index >= 0) {
                                        if (millis() - last_blink_green > 3000) {
                                            blink_led_async(LED_GREEN, -1, 3000);
                                            last_blink_green = millis();
                                        }
                                        DEBUG_MSG("Transmitter id :0x%lx Already exist\r\n", tx.id);
                                    } else {
                                        blink_led_async(LED_YELLOW, 100, -1);
                                        tx.key = CONFIG_PARAM_INVALID;

                                        memcpy(tx.counter, recv_counter_value, sizeof (recv_counter_value));

                                        int err = eep_remember_tx_id(&tx);
                                        blink_led_async_stop(LED_YELLOW);
                                        relay_pulse_on(RELAY_01, 3000);
                                        if (err == ERR_RF_STORAGE_FULL) {
                                            blink_led_async(LED_ALTER, 100, 3000);
                                        } else {
                                            blink_led_async(LED_GREEN, 100, 3000);
                                        }
                                        delay_ms(3000);
                                        DEBUG_MSG("Transmitter id :0x%lx Added rc:%d\r\n", tx.id, err);
                                    }
                                }
                                    task_done = true;
                                    if(task_done) {
                                        break;
                                    }
                            }         
                    } else {
                        test_mode = false;
                        position = ROT_POS_INVALID;
                        DEBUG_MSG("Exit from Testing Mode \r\n");
                        break;
                    }
                   
                }
                while (task_done) {
                     if (test_mode) {
                     DEBUG_MSG("Turn rotaty to 2 for DS 2 Mode \r\n");  
                     while (!(position == 2)) {
                            position = rotary_position_now();
                            while (position == 2) {
                                if (JUMPER1_GetValue() == JUMPER_ON && JUMPER2_GetValue() == JUMPER_ON) {
                                    DEBUG_MSG("WE are in 2nd Mode \r\n");
                                } else {
                                    test_mode = false;
                                    task_done = false;
                                    position = ROT_POS_INVALID;
                                    DEBUG_MSG("Exit from Testing Mode \r\n");
                                    break;
                                }
                            }
                     }
         

                }
            }

                
                
            }
        }

        system_config_update_task();
    }
}
// </editor-fold> 


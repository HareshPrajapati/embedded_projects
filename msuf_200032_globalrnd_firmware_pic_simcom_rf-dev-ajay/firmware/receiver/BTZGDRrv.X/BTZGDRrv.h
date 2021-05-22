#ifndef _BTZGDRrv_H_
#define _BTZGDRrv_H_

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

// <editor-fold defaultstate="collapsed" desc="General">
#define _XTAL_FREQ                  16000000
#define DEBUG_ENABLE                1

#if (DEBUG_ENABLE==1)
#define DEBUG_MSG(...)              printf(__VA_ARGS__)
#else 
#define DEBUG_MSG(...)              
#endif

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Counter">
#define CONSTANT_INC_BY_UINT16_VAL      5
#define COUNTER_CHECK_RANGE             100

// 16 Byte counter value...

typedef struct {
    uint16_t word0;
    uint16_t word1;
    uint16_t word2;
    uint16_t word3;
    uint16_t word4;
    uint16_t word5;
    uint16_t word6;
    uint16_t word7;
} counter_t;

void counter_inc(counter_t *ctx);
void counter_inc_by_n(counter_t *ctx, uint16_t val);

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Decryption">
static uint8_t decrypt_key = 10;

bool decrypt(uint8_t *plain_msg, uint8_t *cypher_msg, uint8_t plain_len, uint8_t cypher_len);
int bit_convt_6to4(uint8_t symbol);

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Jumper">
#define JUMPER1_GetValue()              (PORTAbits.RA1)
#define JUMPER2_GetValue()              (PORTAbits.RA2)
#define JUMPER3_GetValue()              (PORTAbits.RA3)

#define JUMPER_ON                       0
#define JUMPER_OFF                      1
// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Login data">

typedef struct {
    uint32_t last_session;
    uint32_t key;

    union {
        uint32_t u32pass;
        uint8_t password[4];
    };
    uint8_t count;
    bool apply;
} login_t;
// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="System Config">
#define CONFIG_USE_LIMITED_FUNC             (0)

#define CONFIG_BEGIN_ADD                    (0x0000)
#define CONFIG_PARAM_SIZE                   (4)
#define CONFIG_PARAM_INVALID                (0xFFFFFFFF)

#define CONFIG_RELAY_MAX                    (4)
#define CONFIG_RF_ID_MAX                    (4)

#define CONFIG_LOGIN_RETRY_COUNT            (3)
#define CONFIG_LOGIN_BLOCK_TIMEOUT          (5U * 60U * 1000UL)
#define CONFIG_LOGIN_BLOCK_UPDATE_RATE      (30000UL)

typedef struct {

    union {
        uint32_t val;
        uint8_t u8[4];
    } pulse_width;
} cfg_pulse_width_t;

typedef struct {
    uint32_t wait_time;
    uint32_t width_time;
    uint32_t active_low;
} cfg_trigger_t;

typedef struct {
    uint32_t id;
    uint32_t key;
    uint8_t counter[sizeof (counter_t)];
} transmitter_id_t;

typedef struct {
    uint8_t mode[CONFIG_RELAY_MAX];
} relay_mode_t;

typedef struct {
    transmitter_id_t transmitter[CONFIG_RF_ID_MAX + 1];
    cfg_pulse_width_t pulse_width[CONFIG_RELAY_MAX];
    cfg_trigger_t trigger_setting;
    relay_mode_t relay_mode;
    uint32_t user_pin;
    uint32_t technician_pin;
    uint32_t system_block_timeout_ms;
    uint32_t cfg_size;
} system_config_t;

#define CONFIG_RELAY_PULSE_DEFAULT_VAL      (3000)

void system_config_begin(void);
void system_config_update_task(void);

uint32_t system_get_block_timeout(void);
void     system_set_block_timeout(uint32_t tout);

void system_set_user_pin(uint32_t pin);
uint32_t system_get_user_pin();

void system_set_technician_pin(uint32_t pin);
uint32_t system_get_technician_pin();

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Milli-seconds functions">
typedef void (*external_tick_cb_t)(uint32_t);

uint32_t millis(void);
bool millis_attach_external_callback(external_tick_cb_t cb);
void delay_ms(uint32_t ms);

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C">

typedef enum {
    I2C1_NOERR, // The message was sent.
    I2C1_BUSY, // Message was not sent, bus was busy.
    I2C1_FAIL // Message was not sent, bus failure
    // If you are interested in the failure reason,
    // Sit on the event call-backs.
} i2c1_error_t;

typedef enum {
    I2C1_STOP = 1,
    I2C1_RESTART_READ,
    I2C1_RESTART_WRITE,
    I2C1_CONTINUE,
    I2C1_RESET_LINK
} i2c1_operations_t;

typedef uint8_t i2c1_address_t;
typedef i2c1_operations_t(*i2c1_callback_t)(void *funPtr);

// common callback responses
i2c1_operations_t I2C1_CallbackReturnStop(void *funPtr);
i2c1_operations_t I2C1_CallbackReturnReset(void *funPtr);
i2c1_operations_t I2C1_CallbackRestartWrite(void *funPtr);
i2c1_operations_t I2C1_CallbackRestartRead(void *funPtr);

/**
 * \brief Initialize I2C1 interface
 *
 * \return Nothing
 */
void I2C1_Initialize(void);

/**
 * \brief Open the I2C1 for communication
 *
 * \param[in] address The slave address to use in the transfer
 *
 * \return Initialization status.
 * \retval I2C1_NOERR The I2C1 open was successful
 * \retval I2C1_BUSY  The I2C1 open failed because the interface is busy
 * \retval I2C1_FAIL  The I2C1 open failed with an error
 */
i2c1_error_t I2C1_Open(i2c1_address_t address);

/**
 * \brief Close the I2C1 interface
 *
 * \return Status of close operation.
 * \retval I2C1_NOERR The I2C1 open was successful
 * \retval I2C1_BUSY  The I2C1 open failed because the interface is busy
 * \retval I2C1_FAIL  The I2C1 open failed with an error
 */
i2c1_error_t I2C1_Close(void);

/**
 * \brief Start an operation on an opened I2C1 interface
 *
 * \param[in] read Set to true for read, false for write
 *
 * \return Status of operation
 * \retval I2C1_NOERR The I2C1 open was successful
 * \retval I2C1_BUSY  The I2C1 open failed because the interface is busy
 * \retval I2C1_FAIL  The I2C1 open failed with an error
 */
i2c1_error_t I2C1_MasterOperation(bool read);

/**
 * \brief Identical to I2C1_MasterOperation(false);
 */
i2c1_error_t I2C1_MasterWrite(void); // to be depreciated

/**
 * \brief Identical to I2C1_MasterOperation(true);
 */
i2c1_error_t I2C1_MasterRead(void); // to be depreciated

/**
 * \brief Set timeout to be used for I2C1 operations. Uses the Timeout driver.
 *
 * \param[in] to Timeout in ticks
 *
 * \return Nothing
 */
void I2C1_SetTimeOut(uint8_t timeOutValue);

/**
 * \brief Sets up the data buffer to use, and number of bytes to transfer
 *
 * \param[in] buffer Pointer to data buffer to use for read or write data
 * \param[in] bufferSize Number of bytes to read or write from slave
 *
 * \return Nothing
 */
void I2C1_SetBuffer(void *buffer, size_t bufferSize);

// Event Callback functions.

/**
 * \brief Set callback to be called when all specifed data has been transferred.
 *
 * \param[in] cb Pointer to callback function
 * \param[in] ptr  Pointer to the callback function's parameters
 *
 * \return Nothing
 */
void I2C1_SetDataCompleteCallback(i2c1_callback_t cb, void *ptr);

/**
 * \brief Set callback to be called when there has been a bus collision and arbitration was lost.
 *
 * \param[in] cb Pointer to callback function
 * \param[in] ptr  Pointer to the callback function's parameters
 *
 * \return Nothing
 */
void I2C1_SetWriteCollisionCallback(i2c1_callback_t cb, void *ptr);

/**
 * \brief Set callback to be called when the transmitted address was Nack'ed.
 *
 * \param[in] cb Pointer to callback function
 * \param[in] ptr  Pointer to the callback function's parameters
 *
 * \return Nothing
 */
void I2C1_SetAddressNackCallback(i2c1_callback_t cb, void *ptr);

/**
 * \brief Set callback to be called when the transmitted data was Nack'ed.
 *
 * \param[in] cb Pointer to callback function
 * \param[in] ptr  Pointer to the callback function's parameters
 *
 * \return Nothing
 */
void I2C1_SetDataNackCallback(i2c1_callback_t cb, void *ptr);

/**
 * \brief Set callback to be called when there was a bus timeout.
 *
 * \param[in] cb Pointer to callback function
 * \param[in] ptr  Pointer to the callback function's parameters
 *
 * \return Nothing
 */
void I2C1_SetTimeoutCallback(i2c1_callback_t cb, void *ptr);

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="UART2">

uint8_t uart2_read(void);
void uart2_write(uint8_t data);

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Led">

#define LED_BLINKING_RATE_MS                (100)

#if (DEBUG_ENABLE==1)
#define GREEN_LED_GetValue()                  0
#define GREEN_LED_SetHigh()                   do { } while(0)
#define GREEN_LED_SetLow()                    do { } while(0)
#define GREEN_LED_Toggle()                    do { } while(0)
#else      
#define GREEN_LED_GetValue()                  PORTBbits.RB6
#define GREEN_LED_SetHigh()                   do { LATBbits.LATB6 = 1; } while(0)
#define GREEN_LED_SetLow()                    do { LATBbits.LATB6 = 0; } while(0)
#define GREEN_LED_Toggle()                    do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#endif

#define RED_LED_GetValue()                  PORTBbits.RB7
#define RED_LED_SetHigh()                   do { LATBbits.LATB7 = 1; } while(0)
#define RED_LED_SetLow()                    do { LATBbits.LATB7 = 0; } while(0)
#define RED_LED_Toggle()                    do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)

typedef enum {
    LED_GREEN,
    LED_RED,
    LED_YELLOW,
    LED_ALTER
} led_t;

typedef struct {
    bool is_enabled;
    int rate_ms;
    int timeout;
    uint32_t start_time;
    uint32_t last_tick;
} led_async_t;

void blink_init();
void blink_led(led_t led, uint32_t ms);
void blink_led_async(led_t led, int blink_rate, int timeout);
void blink_led_async_stop(led_t led);


// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Receiver switch on board">
#define SWITCH_DEBOUNCE_IN_MS           150
#define RECV_SW_GET()                   PORTAbits.RA0
#define SW_PRESSED                      0U
#define SW_NOT_PRESSED                  1U

typedef struct {
    uint32_t pressed_time;
    uint32_t released_time;
    uint32_t duration;
    bool event;
} recv_switch_t;

recv_switch_t *recv_switch_begin();

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Relay driver">

#define RELAY0_GetValue()           PORTAbits.RA7
#define RELAY1_GetValue()           PORTAbits.RA6
#define RELAY2_GetValue()           PORTAbits.RA5
#define RELAY3_GetValue()           PORTAbits.RA4

#define RELAY0_SetHigh()            do { LATAbits.LATA7 = 1; } while(0)
#define RELAY0_SetLow()             do { LATAbits.LATA7 = 0; } while(0)
#define RELAY0_SetToggle()          do { LATAbits.LATA7 = ~LATAbits.LATA7; } while(0)
#define RELAY1_SetHigh()            do { LATAbits.LATA6 = 1; } while(0)
#define RELAY1_SetLow()             do { LATAbits.LATA6 = 0; } while(0)
#define RELAY1_SetToggle()          do { LATAbits.LATA6 = ~LATAbits.LATA6; } while(0)
#define RELAY2_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define RELAY2_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define RELAY2_SetToggle()          do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define RELAY3_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define RELAY3_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define RELAY3_SetToggle()          do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)

#define BUTTON12_PRESSED_SYMBOL     (0xAA)
#define BUTTON34_PRESSED_SYMBOL     (0xAB)

typedef enum {
    BUTTON_1 = 0xA1,
    BUTTON_2,
    BUTTON_3,
    BUTTON_4
} button_id_t;

typedef enum {
    TRIGGER_ACTIVE_LOW = 0x55,
    TRIGGER_ACTIVE_HIGH = 0xAA
} edge_t;

typedef enum {
    RELAY_01,
    RELAY_02,
    RELAY_03,
    RELAY_04,
    RELAY_MAX
} relay_t;

typedef struct {
    bool wait;
    bool pending;
    uint32_t start_time;
    uint32_t timeout;
} relay_task_t;

typedef enum {
    RLY_MOMENTARY_MODE = 0x1,
    RLY_LATCH0_MODE,
    RLY_LATCH1_MODE,
    RLY_PULSE_MODE,
    RLY_DISABLED,
} relay_mode_list_t;

void relay_begin(void);
void relay_task(void);
void relay_update_config(void);

void relay_set_pulse_time(relay_t relay, uint32_t timems);
uint32_t relay_get_pulse_time(relay_t relay);
void relay_forget_pulse_time(relay_t relay);

void relay_set_trigger_edge(edge_t active_state);
edge_t relay_get_trigger_edge();

void relay_set_trigger_wait(uint32_t t);
uint32_t relay_get_trigger_wait();

void relay_set_trigger_width(uint32_t t);
uint32_t relay_get_trigger_width();

void relay_set_mode(relay_t relay, uint8_t mode);
uint8_t relay_get_mode(relay_t relay);

void relay_pulse_on(relay_t relay, int manual_tout);
void relay_trigger_on();
void relay_trigger_off();

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="RF-433 Receiver">
// Must set same baudrate to UART module 2.
#define RF_RECEIVER_BAUD                (4800)

// RF packet defines.
#define RF_PACKET_BUFFER_SIZE           (74)
#define RF_PACKET_DCRYPT_BUFF_SIZE      (31)
#define RF_PACKET_IDENTITY_SIZE         (12)
#define RF_PACKET_TRANSMITTER_ID_SIZE   (04)
#define RF_PACKET_TRANSMITTER_CRC_SIZE  (01)
#define RF_PACKET_COUNTER_SIZE          (sizeof(counter_t))
#define RF_PACKET_PAYLOAD_COUNT         ((RF_PACKET_DCRYPT_BUFF_SIZE-(RF_PACKET_TRANSMITTER_ID_SIZE+RF_PACKET_TRANSMITTER_CRC_SIZE+1+RF_PACKET_COUNTER_SIZE + 4)))

#define RF_MASTER_TRANSMITTER_ID        (0x55665566)

// Quick data conversion.

typedef union {
    uint32_t id;
    uint8_t u8[4];
} id_t;

void rf_receiver_begin();
bool rf_receiver_available(void);
uint32_t rf_receiver_get_tx_id(void);
int rf_receiver_get_msg(uint8_t *buffer, uint8_t blen);
bool is_transmitter_is_master(uint32_t id);

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Rotary switch(DS)">
// Enum to hold switch position.

#define ROT_SW_1_GetValue()           PORTBbits.RB3
#define ROT_SW_2_GetValue()           PORTBbits.RB2
#define ROT_SW_3_GetValue()           PORTBbits.RB1
#define ROT_SW_4_GetValue()           PORTBbits.RB0

typedef enum {
    ROT_POS_0,
    ROT_POS_1,
    ROT_POS_2,
    ROT_POS_3,
    ROT_POS_4,
    ROT_POS_5,
    ROT_POS_6,
    ROT_POS_7,
    ROT_POS_8,
    ROT_POS_9,
    ROT_POS_INVALID,
} rotary_position_t;

void rotary_switch_begin(void);
void rotary_switch_on_change_handler(void);
rotary_position_t rotary_position_now(void);

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="EEPROM">
#define EEPROM_SIZE_IN_KB                   (128)
#define EEPROM_SIZE_IN_BYTE                 (EEPROM_SIZE_IN_KB*1024)
#define EEPROM_PAGE_SIZE_BYTES              (128)
#define EEPROM_I2C_ADDRESS                  (0xA0)
#define EEPROM_START_ADDRESS                (0x000000)
#define EEPROM_END_ADDRESS                  (0x01FFFF)

#define EEP_WP_SetLow()                     do { LATCbits.LATC5 = 0; } while(0)
#define EEP_WP_SetHigh()                    do { LATCbits.LATC5 = 1; } while(0)

#define ERR_RF_ALREADY_EXIST                -1
#define ERR_RF_STORAGE_FULL                 -2
#define ERR_RF_NOT_EXIST                    -3

void eeprom_read_nbytes(uint32_t add, uint8_t *pbuffer, uint8_t len);
void eeprom_write_nbytes(uint32_t add, uint8_t *pbuffer, uint8_t len);
void eeprom_write_byte(uint32_t add, uint8_t data);
uint8_t eeprom_read_byte(uint32_t add);
void eeprom_enable_wp();
void eeprom_disable_wp();

void eep_begin(void);
void eep_erase(void);
uint32_t eep_get_tx_id_count(void);
int eep_remember_tx_id(transmitter_id_t *id);
int eep_forget_tx_id(uint32_t id);
bool eep_forget_all(void);
int eep_check_tx_id(uint32_t id);

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Trigger pin">
#define TRIGGER_GetValue()           PORTCbits.RC6
// </editor-fold> 

#endif

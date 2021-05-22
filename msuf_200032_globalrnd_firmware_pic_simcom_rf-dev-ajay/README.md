# Global R&D firmware.

| Firmware   | Description         |
| ---------- | ------------------- |
| BTZGDRrv.X | RF receiver demo    |
| BTZGDRtx.X | RF Transmitter demo |

## Receiver
- Receiver `(DS=0)` Run mode.
- Extra pattern added for this mode.
- If transmitter is valid yellow led will blink @50ms.
- If transmitter is invalid red led will blink @50ms.

## How encryption works.
- Below flow-graph shows how message is encrypted before transfer.

    ![flow_graph](/assets/RF_cypher_flow.svg)


## Transmitter packet format.
- Packet size is fixed `11 bytes`.
- Plain message size is `11 bytes`.
- Encryption sending nibbles as symbol so encrypted message needs `22 bytes`
- So, buffer to hold `cypher_msg` must have `22 byte` array/buffer.

- Plain_msg:
  - Total 11 bytes. 

    | 4 byte transmitter id | 1 byte len | 5 byte payload | 1 byte crc    |
    | --------------------- | ---------- | -------------- | ------------- |
    | 0xaabbccdd            | 5          | [5 byte data]  | crc of packet |

- Encryped_msg:
  - Total 22 bytes.

    | byte 1 | byte 2 | byte 3 | ... | byte22 |
    | ------ | ------ | ------ | --- | ------ |
    | symbol | symbol | symbol | ... | symbol |
## How transmitter send message. 
- Transmitter first send 4 `0xf0` bytes to make receiver in sync.
- Then send `5 preamble` bytes to identify packet starting [0xFE,0xF0,0xFE,0xF0,0xFE].
- Then send `22 byte` encrypred message.
- Total bytes to receive is `5 preamble bytes + 22 msg bytes = 27` 

## How receiver receive messsage.
- Receiver continue check for start byte `0xFE`.
- After start byte found receive remaining `26 bytes`.
- Now check for preamble message `[0xFE,0xF0,0xFE,0xF0,0xFE]`.
- If preamble found it will pass to decryptor.
- Decryptor decrypt message to buffer(`11 byte`).
- Now, Check for crc if valid message is accepted else message discarded.

## CRC generation.
- CRC calculated for first `10 bytes` because 11th byte is crc itself.

```c
    uint8_t crc8(uint8_t *buffer, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc += buffer[i];
    }
    crc = (~crc) + 1;
    return crc;
}
```
## Change transmitter id.
- Transmitter id is define in `BTZGDRtx.h` file.
- Transmitter id is 4-bytes long.
- Every byte of transmitter id must fall within (0x01-0xfe). `0x00` and `0xFF` not allowed.

```c
// Change your transmitter ID (4 byte) here...  
#define RF_TRANSMITTER_ID               0x22112244
```

## How to enable `master transmitter mode`.
- To program transmitter as `Master transmitter` follow below steps.
  - Change `RF_TRANSMITTER_MASTER_MODE` macro to 1. 
    
    ```c
    #define RF_TRANSMITTER_MASTER_MODE 1
    ```
  - Build and flash.

## Master transmitter use in DS=7 while technician lock is set.
- to reset technician lock from master trasnmitter follow below steps.
  - Set mode `DS=7`.
  - Press and hold receiver button.
  - Press `key 1 and 2 combo` from master transmitter.
  - Done.

## Configure receiver `LIMITED Mode (DS=0 && 5)`
- Please changed macro value as shown below to activate `Limited Functionality Mode`.
  - `#define CONFIG_USE_LIMITED_FUNC             (1)`
- Please changed macro value as shown below to activate `Normal Mode`.
  - `#define CONFIG_USE_LIMITED_FUNC             (0)`


## Extra pattern.
- Led blinks `alter` mode @500ms.
-  When device is technician locked and waiting for timeout of 5 min (because of max retry login failed) and recieves key press event.
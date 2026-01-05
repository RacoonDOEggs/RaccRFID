# RaccRFID Library

A lightweight Arduino library for interfacing with MFRC522 RFID reader modules via SPI. Designed for ESP32 and other Arduino-compatible boards.

## Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Class Reference](#class-reference)
- [Register Definitions](#register-definitions)
- [Command Definitions](#command-definitions)
- [Function Reference](#function-reference)
- [Error Codes](#error-codes)
- [Memory Layout](#memory-layout)

---

## Installation

Place the `RaccRFID` folder in your project's `lib/` directory. The library will be automatically detected by PlatformIO.

---

## Quick Start

```cpp
#include <Arduino.h>
#include <RaccRFID.h>

#define SCK_PIN   18
#define MISO_PIN  16
#define MOSI_PIN  17
#define SS_PIN    34
#define RST_PIN   11

SPIClass rfidSPI;
RaccRFID rfid(rfidSPI, SS_PIN, RST_PIN);

void setup() {
  Serial.begin(115200);
  rfid.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  
  byte version = rfid.readRegister(VersionReg);
  Serial.print("MFRC522 Version: 0x");
  Serial.println(version, HEX);
}

void loop() {
  byte atqa[2];
  byte uid[5];
  byte sak[3];
  
  if (rfid.PICC_RequestA(atqa) == 0) {
    if (rfid.PICC_Anticollision(uid) == 0) {
      if (rfid.PICC_Select(uid, sak) == 0) {
        if (rfid.PICC_Authenticate(PICC_AUTH_A, 4, RaccRFID::defaultKey, uid) == 0) {
          byte data[18];
          byte len;
          rfid.PICC_Read(4, data, &len);
        }
        rfid.PICC_HaltA();
      }
    }
  }
  delay(500);
}
```

---

## Class Reference

### Constructor

```cpp
RaccRFID(SPIClass &spi, uint8_t ssPin, uint8_t rstPin)
```

Creates a new RaccRFID instance.

| Parameter | Description |
|-----------|-------------|
| `spi` | Reference to an SPIClass instance |
| `ssPin` | Slave Select (CS) pin number |
| `rstPin` | Reset pin number |

### Static Members

```cpp
static byte defaultKey[6]
```

Factory default MIFARE key: `{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}`

---

## Register Definitions

### Command and Status Registers

| Define | Address | Description |
|--------|---------|-------------|
| `CommandReg` | 0x01 | Starts and stops command execution |
| `ComIEnReg` | 0x02 | Interrupt enable/disable control |
| `ComIrqReg` | 0x04 | Interrupt request flags |
| `DivIrqReg` | 0x05 | Interrupt request flags (additional) |
| `ErrorReg` | 0x06 | Error flags from last command |
| `Status1Reg` | 0x07 | Communication status |
| `Status2Reg` | 0x08 | Receiver and transmitter status |

### FIFO Registers

| Define | Address | Description |
|--------|---------|-------------|
| `FIFODataReg` | 0x09 | Input/output of 64-byte FIFO buffer |
| `FIFOLevelReg` | 0x0A | Number of bytes stored in FIFO |

### Control Registers

| Define | Address | Description |
|--------|---------|-------------|
| `ControlReg` | 0x0C | Miscellaneous control |
| `BitFramingReg` | 0x0D | Bit-oriented frame adjustments |
| `CollReg` | 0x0E | Collision detection settings |

### Mode Registers

| Define | Address | Description |
|--------|---------|-------------|
| `ModeReg` | 0x11 | General mode settings |
| `TxModeReg` | 0x12 | Transmitter data rate settings |
| `RxModeReg` | 0x13 | Receiver data rate settings |
| `TxControlReg` | 0x14 | Antenna driver pin control |
| `TxASKReg` | 0x15 | ASK modulation settings |

### CRC Registers

| Define | Address | Description |
|--------|---------|-------------|
| `CRCResultRegH` | 0x21 | CRC calculation result (high byte) |
| `CRCResultRegL` | 0x22 | CRC calculation result (low byte) |

### Timer Registers

| Define | Address | Description |
|--------|---------|-------------|
| `TModeReg` | 0x2A | Timer settings |
| `TPrescalerReg` | 0x2B | Timer prescaler |
| `TReloadRegH` | 0x2C | Timer reload value (high byte) |
| `TReloadRegL` | 0x2D | Timer reload value (low byte) |

### Test Registers

| Define | Address | Description |
|--------|---------|-------------|
| `VersionReg` | 0x37 | Software version (0x91 or 0x92 for MFRC522) |

---

## Command Definitions

### PCD (Reader) Commands

| Define | Value | Description |
|--------|-------|-------------|
| `PCD_Idle` | 0x00 | No action, cancels current command |
| `PCD_CalcCRC` | 0x03 | Calculate CRC for data in FIFO |
| `PCD_Transceive` | 0x0C | Transmit and receive data |
| `PCD_MFAuthent` | 0x0E | Perform MIFARE authentication |
| `PCD_SoftReset` | 0x0F | Reset the MFRC522 |

### PICC (Card) Commands

| Define | Value | Description |
|--------|-------|-------------|
| `PICC_REQA` | 0x26 | Request Type A (wake idle cards) |
| `PICC_WUPA` | 0x52 | Wake-Up Type A (wake all cards) |
| `PICC_ANTICOLL1` | 0x93 | Anti-collision level 1 |
| `PICC_SELECT1` | 0x93 | Select card level 1 |
| `PICC_AUTH_A` | 0x60 | Authenticate with Key A |
| `PICC_AUTH_B` | 0x61 | Authenticate with Key B |
| `PICC_READ` | 0x30 | Read 16-byte block |
| `PICC_WRITE` | 0xA0 | Write 16-byte block |
| `PICC_HALT` | 0x50 | Halt card communication |

---

## Function Reference

### Initialization

#### begin()

```cpp
void begin(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin)
```

Initializes the SPI bus and MFRC522 module.

| Parameter | Description |
|-----------|-------------|
| `sckPin` | SPI clock pin |
| `misoPin` | SPI MISO pin |
| `mosiPin` | SPI MOSI pin |

---

### Low-Level Register Operations

#### writeRegister()

```cpp
void writeRegister(byte reg, byte value)
```

Writes a single byte to a register.

| Parameter | Description |
|-----------|-------------|
| `reg` | Register address |
| `value` | Byte value to write |

#### readRegister()

```cpp
byte readRegister(byte reg)
```

Reads a single byte from a register.

| Parameter | Description |
|-----------|-------------|
| `reg` | Register address |

**Returns:** Byte value read from the register.

#### writeFIFO()

```cpp
void writeFIFO(byte *buffer, byte length)
```

Writes multiple bytes to the FIFO buffer.

| Parameter | Description |
|-----------|-------------|
| `buffer` | Pointer to data array |
| `length` | Number of bytes to write |

#### readFIFO()

```cpp
void readFIFO(byte *buffer, byte length)
```

Reads multiple bytes from the FIFO buffer.

| Parameter | Description |
|-----------|-------------|
| `buffer` | Pointer to buffer for received data |
| `length` | Number of bytes to read |

#### setBitMask()

```cpp
void setBitMask(byte reg, byte mask)
```

Sets specific bits in a register using OR operation.

| Parameter | Description |
|-----------|-------------|
| `reg` | Register address |
| `mask` | Bit mask to set |

#### clearBitMask()

```cpp
void clearBitMask(byte reg, byte mask)
```

Clears specific bits in a register using AND NOT operation.

| Parameter | Description |
|-----------|-------------|
| `reg` | Register address |
| `mask` | Bit mask to clear |

---

### PCD (Reader) Operations

#### PCD_Init()

```cpp
void PCD_Init()
```

Performs hardware and software reset, configures timers, and enables the antenna. Called automatically by `begin()`.

#### PCD_TransceiveData()

```cpp
byte PCD_TransceiveData(byte *sendData, byte sendLen, byte *backData, byte *backLen, bool checkCRC = false)
```

Transmits data to the card and receives the response.

| Parameter | Description |
|-----------|-------------|
| `sendData` | Pointer to data to send |
| `sendLen` | Number of bytes to send |
| `backData` | Pointer to buffer for response |
| `backLen` | Pointer to variable receiving response length |
| `checkCRC` | Whether to verify CRC (optional, default: false) |

**Returns:** 0 on success, error code on failure.

---

### PICC (Card) Operations

#### PICC_RequestA()

```cpp
byte PICC_RequestA(byte *atqa)
```

Sends REQA command to detect cards in the RF field.

| Parameter | Description |
|-----------|-------------|
| `atqa` | Pointer to 2-byte buffer for ATQA response |

**Returns:** 0 on success (card detected), 0xFF on failure.

#### PICC_Anticollision()

```cpp
byte PICC_Anticollision(byte *uid)
```

Performs anti-collision procedure to retrieve the card's UID.

| Parameter | Description |
|-----------|-------------|
| `uid` | Pointer to 5-byte buffer (4-byte UID + 1-byte BCC) |

**Returns:** 0 on success, 0xFE on BCC error, other values on communication error.

#### PICC_Select()

```cpp
byte PICC_Select(byte *uid, byte *sak)
```

Selects a card for further communication.

| Parameter | Description |
|-----------|-------------|
| `uid` | Pointer to 5-byte UID buffer (from PICC_Anticollision) |
| `sak` | Pointer to 3-byte buffer for SAK response |

**Returns:** 0 on success, 0xFF on failure.

#### PICC_Authenticate()

```cpp
byte PICC_Authenticate(byte command, byte blockAddr, byte *key, byte *uid)
```

Authenticates a sector using Key A or Key B.

| Parameter | Description |
|-----------|-------------|
| `command` | PICC_AUTH_A (0x60) or PICC_AUTH_B (0x61) |
| `blockAddr` | Block number to authenticate |
| `key` | Pointer to 6-byte key |
| `uid` | Pointer to 4-byte UID |

**Returns:** 0 on success, 0xFF on timeout, 0xFE on error, 0xFD if crypto not enabled.

#### PICC_Read()

```cpp
byte PICC_Read(byte blockAddr, byte *buffer, byte *bufferLen)
```

Reads a 16-byte block from the card. Must authenticate first.

| Parameter | Description |
|-----------|-------------|
| `blockAddr` | Block number to read (0-63 for 1K, 0-255 for 4K) |
| `buffer` | Pointer to 18-byte buffer (16 data + 2 CRC) |
| `bufferLen` | Pointer to variable receiving actual length |

**Returns:** 0 on success, error code on failure.

#### PICC_Write()

```cpp
byte PICC_Write(byte blockAddr, byte *data)
```

Writes 16 bytes to a block on the card. Must authenticate first.

| Parameter | Description |
|-----------|-------------|
| `blockAddr` | Block number to write |
| `data` | Pointer to 16-byte data array |

**Returns:** 0 on success, 0xFE on NAK (write command), 0xFD on NAK (data).

**Warning:** Never write to block 0 (manufacturer data) or sector trailer blocks (3, 7, 11, 15, ...) unless you fully understand the consequences.

#### PICC_HaltA()

```cpp
void PICC_HaltA()
```

Sends HALT command to stop communication with the card. The card will not respond to REQA until removed from the field.

---

## Error Codes

| Code | Meaning |
|------|---------|
| 0x00 | Success |
| 0xFF | Timeout or communication failure |
| 0xFE | BCC error, NAK received, or authentication error |
| 0xFD | Crypto not enabled or data NAK |
| 0x01 | Protocol error |
| 0x02 | Parity error |
| 0x04 | CRC error |
| 0x08 | Collision detected |
| 0x10 | Buffer overflow |

---

## Memory Layout

### MIFARE Classic 1K Structure

- 16 sectors, 4 blocks each (64 blocks total)
- Each block is 16 bytes
- Block 0: Manufacturer data (read-only)
- Blocks 3, 7, 11, 15, ...: Sector trailers (keys and access bits)

### Sector Trailer Format (bytes)

| Bytes | Content |
|-------|---------|
| 0-5 | Key A (6 bytes, write-only) |
| 6-9 | Access bits (4 bytes) |
| 10-15 | Key B (6 bytes) |

### Safe Blocks for Data Storage

Sectors 1-15, blocks 0-2 of each sector:
- Sector 1: Blocks 4, 5, 6
- Sector 2: Blocks 8, 9, 10
- And so on...

---

## License

This library is provided as-is for educational and hobbyist use.

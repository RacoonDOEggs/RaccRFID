#ifndef RACCRFID_H
#define RACCRFID_H

#include <Arduino.h>
#include <SPI.h>

// MFRC522 Registers
#define CommandReg      0x01
#define ComIEnReg       0x02
#define ComIrqReg       0x04
#define DivIrqReg       0x05
#define ErrorReg        0x06
#define Status1Reg      0x07
#define Status2Reg      0x08
#define FIFODataReg     0x09
#define FIFOLevelReg    0x0A
#define ControlReg      0x0C
#define BitFramingReg   0x0D
#define CollReg         0x0E
#define ModeReg         0x11
#define TxModeReg       0x12
#define RxModeReg       0x13
#define TxControlReg    0x14
#define TxASKReg        0x15
#define CRCResultRegH   0x21
#define CRCResultRegL   0x22
#define TModeReg        0x2A
#define TPrescalerReg   0x2B
#define TReloadRegH     0x2C
#define TReloadRegL     0x2D
#define VersionReg      0x37

// MFRC522 Commands
#define PCD_Idle        0x00
#define PCD_CalcCRC     0x03
#define PCD_Transceive  0x0C
#define PCD_MFAuthent   0x0E
#define PCD_SoftReset   0x0F

// PICC Commands
#define PICC_REQA       0x26
#define PICC_WUPA       0x52
#define PICC_ANTICOLL1  0x93
#define PICC_SELECT1    0x93
#define PICC_AUTH_A     0x60
#define PICC_AUTH_B     0x61
#define PICC_READ       0x30
#define PICC_WRITE      0xA0
#define PICC_HALT       0x50

class RaccRFID {
public:
  // Constructor
  RaccRFID(SPIClass &spi, uint8_t ssPin, uint8_t rstPin);
  
  // Initialize the RFID reader
  void begin(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin);
  
  // Low-level register operations
  void writeRegister(byte reg, byte value);
  byte readRegister(byte reg);
  void readFIFO(byte *buffer, byte length);
  void writeFIFO(byte *buffer, byte length);
  void setBitMask(byte reg, byte mask);
  void clearBitMask(byte reg, byte mask);
  
  // PCD (reader) operations
  void PCD_Init();
  byte PCD_TransceiveData(byte *sendData, byte sendLen, byte *backData, byte *backLen, bool checkCRC = false);
  
  // PICC (card) operations
  byte PICC_RequestA(byte *atqa);
  byte PICC_Anticollision(byte *uid);
  byte PICC_Select(byte *uid, byte *sak);
  byte PICC_Authenticate(byte command, byte blockAddr, byte *key, byte *uid);
  byte PICC_Read(byte blockAddr, byte *buffer, byte *bufferLen);
  byte PICC_Write(byte blockAddr, byte *data);
  void PICC_HaltA();
  
  // Default key
  static byte defaultKey[6];

private:
  SPIClass *_spi;
  uint8_t _ssPin;
  uint8_t _rstPin;
};

#endif

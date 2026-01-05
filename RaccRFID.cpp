#include "RaccRFID.h"

// Default key (factory default)
byte RaccRFID::defaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Constructor
RaccRFID::RaccRFID(SPIClass &spi, uint8_t ssPin, uint8_t rstPin) {
  _spi = &spi;
  _ssPin = ssPin;
  _rstPin = rstPin;
}

// Initialize the RFID reader
void RaccRFID::begin(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin) {
  pinMode(_ssPin, OUTPUT);
  digitalWrite(_ssPin, HIGH);
  _spi->begin(sckPin, misoPin, mosiPin, _ssPin);
  PCD_Init();
}

// Write a byte to a specific MFRC522 register
void RaccRFID::writeRegister(byte reg, byte value) {
  _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_ssPin, LOW);
  // Address byte: bit 7 = 0 (write), bits 6-1 = address, bit 0 = 0
  _spi->transfer((reg << 1) & 0x7E);  // Write mode
  _spi->transfer(value);
  digitalWrite(_ssPin, HIGH);
  _spi->endTransaction();
}

// Read a byte from a specific MFRC522 register
byte RaccRFID::readRegister(byte reg) {
  byte value;
  _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_ssPin, LOW);
  // Address byte: bit 7 = 1 (read), bits 6-1 = address, bit 0 = 0
  _spi->transfer(((reg << 1) & 0x7E) | 0x80);  // Read mode
  value = _spi->transfer(0x00);  // Send dummy byte to clock out response
  digitalWrite(_ssPin, HIGH);
  _spi->endTransaction();
  return value;
}

// Read multiple bytes from FIFO
void RaccRFID::readFIFO(byte *buffer, byte length) {
  if (length == 0) return;
  
  byte address = ((FIFODataReg << 1) & 0x7E) | 0x80;  // Read FIFO address
  
  _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_ssPin, LOW);
  
  _spi->transfer(address);  // Send first read address
  
  // For continuous reads, keep sending the address to get next byte
  for (byte i = 0; i < length - 1; i++) {
    buffer[i] = _spi->transfer(address);  // Send address again to continue reading
  }
  // Last byte - send 0x00 to end the transfer
  buffer[length - 1] = _spi->transfer(0x00);
  
  digitalWrite(_ssPin, HIGH);
  _spi->endTransaction();
}

// Write multiple bytes to FIFO
void RaccRFID::writeFIFO(byte *buffer, byte length) {
  _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_ssPin, LOW);
  _spi->transfer((FIFODataReg << 1) & 0x7E);  // Write FIFO
  for (byte i = 0; i < length; i++) {
    _spi->transfer(buffer[i]);
  }
  digitalWrite(_ssPin, HIGH);
  _spi->endTransaction();
}

// Set specific bits in a register
void RaccRFID::setBitMask(byte reg, byte mask) {
  byte current = readRegister(reg);
  writeRegister(reg, current | mask);
}

// Clear specific bits in a register
void RaccRFID::clearBitMask(byte reg, byte mask) {
  byte current = readRegister(reg);
  writeRegister(reg, current & (~mask));
}

// Initialize the MFRC522
void RaccRFID::PCD_Init() {
  // Hardware reset
  pinMode(_rstPin, OUTPUT);
  digitalWrite(_rstPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_rstPin, HIGH);
  delay(50);
  
  // Soft reset
  writeRegister(CommandReg, PCD_SoftReset);
  delay(50);
  
  // Timer settings for timeout
  writeRegister(TModeReg, 0x8D);       // TAuto=1, prescaler bits
  writeRegister(TPrescalerReg, 0x3E);  // Prescaler lower bits
  writeRegister(TReloadRegH, 0x00);    // Reload timer high
  writeRegister(TReloadRegL, 0x30);    // Reload timer low (~25ms timeout)
  
  writeRegister(TxASKReg, 0x40);       // 100% ASK modulation
  writeRegister(ModeReg, 0x3D);        // CRC preset value 0x6363
  
  // Turn on antenna
  setBitMask(TxControlReg, 0x03);
}

// Communicate with PICC (card)
byte RaccRFID::PCD_TransceiveData(byte *sendData, byte sendLen, byte *backData, byte *backLen, bool checkCRC) {
  writeRegister(CommandReg, PCD_Idle);           // Stop any active command
  writeRegister(ComIrqReg, 0x7F);                // Clear interrupt flags
  writeRegister(FIFOLevelReg, 0x80);             // Flush FIFO buffer
  clearBitMask(CollReg, 0x80);                   // Clear ValuesAfterColl bit
  
  writeFIFO(sendData, sendLen);                  // Write data to FIFO
  
  writeRegister(CommandReg, PCD_Transceive);    // Start transceive
  setBitMask(BitFramingReg, 0x80);              // Start transmission
  
  // Wait for completion
  uint16_t i = 2000;
  byte irq;
  do {
    irq = readRegister(ComIrqReg);
    i--;
  } while (i && !(irq & 0x30));  // RxIRq or IdleIRq
  
  clearBitMask(BitFramingReg, 0x80);            // Stop transmission
  
  if (i == 0) {
    Serial.println("DEBUG: Timeout");
    return 0xFF;  // Timeout
  }
  
  byte error = readRegister(ErrorReg);
  Serial.print("DEBUG: ErrorReg=0x");
  Serial.print(error, HEX);
  Serial.print(" IRQ=0x");
  Serial.println(irq, HEX);
  
  // For anti-collision, ignore collision errors
  if (error & 0x13) {  // BufferOvfl, ParityErr, ProtocolErr (not CollErr)
    return error;
  }
  
  // Read received data
  byte n = readRegister(FIFOLevelReg);
  Serial.print("DEBUG: FIFO has ");
  Serial.print(n);
  Serial.println(" bytes");
  
  if (backLen) *backLen = n;
  if (backData && n > 0) {
    readFIFO(backData, n);
  }
  
  return 0;  // Success
}

// Request card (REQA)
byte RaccRFID::PICC_RequestA(byte *atqa) {
  byte command = PICC_REQA;
  byte backLen = 2;
  
  writeRegister(BitFramingReg, 0x07);  // 7 bits for REQA
  byte status = PCD_TransceiveData(&command, 1, atqa, &backLen);
  writeRegister(BitFramingReg, 0x00);  // Reset to 8 bits
  
  if (status != 0 || backLen != 2) return 0xFF;
  return 0;
}

// Anti-collision to get UID
byte RaccRFID::PICC_Anticollision(byte *uid) {
  byte buffer[9];
  byte backLen = 5;
  
  clearBitMask(CollReg, 0x80);  // ValuesAfterColl=0, all bits received after collision are cleared
  
  buffer[0] = PICC_ANTICOLL1;
  buffer[1] = 0x20;  // NVB: 2 bytes sent, 0 bits
  
  byte status = PCD_TransceiveData(buffer, 2, uid, &backLen);
  
  Serial.print("DEBUG: Anticoll status=0x");
  Serial.print(status, HEX);
  Serial.print(" backLen=");
  Serial.println(backLen);
  
  if (status != 0) return status;
  if (backLen != 5) return 0xFF;
  
  // Check BCC (XOR of UID bytes)
  byte bcc = uid[0] ^ uid[1] ^ uid[2] ^ uid[3];
  if (bcc != uid[4]) {
    Serial.print("DEBUG: BCC error. Expected=0x");
    Serial.print(bcc, HEX);
    Serial.print(" Got=0x");
    Serial.println(uid[4], HEX);
    return 0xFE;  // BCC error
  }
  
  return 0;
}

// Select card
byte RaccRFID::PICC_Select(byte *uid, byte *sak) {
  byte buffer[9];
  byte backLen = 3;
  
  buffer[0] = PICC_SELECT1;
  buffer[1] = 0x70;  // NVB: 7 bytes sent
  memcpy(&buffer[2], uid, 5);  // UID + BCC
  
  // Calculate CRC
  writeRegister(CommandReg, PCD_Idle);
  writeRegister(FIFOLevelReg, 0x80);
  writeFIFO(buffer, 7);
  writeRegister(CommandReg, PCD_CalcCRC);
  
  uint16_t i = 5000;
  while (i--) {
    byte n = readRegister(DivIrqReg);
    if (n & 0x04) break;  // CRC done
  }
  
  buffer[7] = readRegister(CRCResultRegL);
  buffer[8] = readRegister(CRCResultRegH);
  
  byte status = PCD_TransceiveData(buffer, 9, sak, &backLen);
  
  if (status != 0 || backLen != 3) return 0xFF;
  return 0;
}

// Authenticate with card
byte RaccRFID::PICC_Authenticate(byte command, byte blockAddr, byte *key, byte *uid) {
  byte buffer[12];
  
  buffer[0] = command;      // AUTH_A or AUTH_B
  buffer[1] = blockAddr;
  memcpy(&buffer[2], key, 6);   // 6-byte key
  memcpy(&buffer[8], uid, 4);   // 4-byte UID
  
  writeRegister(CommandReg, PCD_Idle);
  writeRegister(ComIrqReg, 0x7F);
  writeRegister(FIFOLevelReg, 0x80);
  writeFIFO(buffer, 12);
  writeRegister(CommandReg, PCD_MFAuthent);
  
  uint16_t i = 2000;
  byte irq;
  do {
    irq = readRegister(ComIrqReg);
    i--;
  } while (i && !(irq & 0x10));  // IdleIRq
  
  if (i == 0) return 0xFF;
  if (readRegister(ErrorReg) & 0x1B) return 0xFE;
  if (!(readRegister(Status2Reg) & 0x08)) return 0xFD;  // MFCrypto1On not set
  
  return 0;
}

// Read 16-byte block from card
byte RaccRFID::PICC_Read(byte blockAddr, byte *buffer, byte *bufferLen) {
  byte cmd[4];
  
  cmd[0] = PICC_READ;
  cmd[1] = blockAddr;
  
  // Calculate CRC for read command
  writeRegister(CommandReg, PCD_Idle);
  writeRegister(FIFOLevelReg, 0x80);
  writeFIFO(cmd, 2);
  writeRegister(CommandReg, PCD_CalcCRC);
  
  uint16_t i = 5000;
  while (i--) {
    if (readRegister(DivIrqReg) & 0x04) break;
  }
  
  cmd[2] = readRegister(CRCResultRegL);
  cmd[3] = readRegister(CRCResultRegH);
  
  *bufferLen = 18;  // 16 data + 2 CRC
  return PCD_TransceiveData(cmd, 4, buffer, bufferLen);
}

// Write 16-byte block to card
byte RaccRFID::PICC_Write(byte blockAddr, byte *data) {
  byte cmd[4];
  byte response[1];
  byte responseLen = 1;
  
  // Step 1: Send WRITE command + block address
  cmd[0] = PICC_WRITE;
  cmd[1] = blockAddr;
  
  // Calculate CRC for write command
  writeRegister(CommandReg, PCD_Idle);
  writeRegister(FIFOLevelReg, 0x80);
  writeFIFO(cmd, 2);
  writeRegister(CommandReg, PCD_CalcCRC);
  
  uint16_t i = 5000;
  while (i--) {
    if (readRegister(DivIrqReg) & 0x04) break;
  }
  
  cmd[2] = readRegister(CRCResultRegL);
  cmd[3] = readRegister(CRCResultRegH);
  
  // Send write command and wait for ACK
  byte status = PCD_TransceiveData(cmd, 4, response, &responseLen);
  
  if (status != 0) {
    Serial.print("DEBUG: Write cmd failed, status=0x");
    Serial.println(status, HEX);
    return status;
  }
  
  // Check for ACK (0x0A = 4 bits)
  if (responseLen != 1 || (response[0] & 0x0F) != 0x0A) {
    Serial.print("DEBUG: No ACK, response=0x");
    Serial.println(response[0], HEX);
    return 0xFE;  // NAK or error
  }
  
  // Step 2: Send 16 bytes of data + CRC
  byte dataBuffer[18];
  memcpy(dataBuffer, data, 16);
  
  // Calculate CRC for data
  writeRegister(CommandReg, PCD_Idle);
  writeRegister(FIFOLevelReg, 0x80);
  writeFIFO(dataBuffer, 16);
  writeRegister(CommandReg, PCD_CalcCRC);
  
  i = 5000;
  while (i--) {
    if (readRegister(DivIrqReg) & 0x04) break;
  }
  
  dataBuffer[16] = readRegister(CRCResultRegL);
  dataBuffer[17] = readRegister(CRCResultRegH);
  
  // Send data and wait for ACK
  responseLen = 1;
  status = PCD_TransceiveData(dataBuffer, 18, response, &responseLen);
  
  if (status != 0) {
    Serial.print("DEBUG: Write data failed, status=0x");
    Serial.println(status, HEX);
    return status;
  }
  
  // Check for ACK
  if (responseLen != 1 || (response[0] & 0x0F) != 0x0A) {
    Serial.print("DEBUG: Data NAK, response=0x");
    Serial.println(response[0], HEX);
    return 0xFD;
  }
  
  return 0;  // Success
}

// Stop communication with card
void RaccRFID::PICC_HaltA() {
  byte buffer[4];
  
  buffer[0] = PICC_HALT;
  buffer[1] = 0x00;
  
  writeRegister(CommandReg, PCD_Idle);
  writeRegister(FIFOLevelReg, 0x80);
  writeFIFO(buffer, 2);
  writeRegister(CommandReg, PCD_CalcCRC);
  
  uint16_t i = 5000;
  while (i--) {
    if (readRegister(DivIrqReg) & 0x04) break;
  }
  
  buffer[2] = readRegister(CRCResultRegL);
  buffer[3] = readRegister(CRCResultRegH);
  
  byte dummy[2];
  byte len = 2;
  PCD_TransceiveData(buffer, 4, dummy, &len);
  
  clearBitMask(Status2Reg, 0x08);  // Clear MFCrypto1On
}

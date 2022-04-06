#ifndef _I2C_H_
#define _I2C_H_

void i2cInit(void);
void i2cWriteByte(uint8_t devAdd, uint8_t regAdd, uint8_t data);
uint8_t i2cReadByte(uint8_t devAdd, uint8_t regAdd);
void i2cReadBytes(uint8_t devAdd, uint8_t regAdd, uint8_t* data, uint8_t length);

#endif
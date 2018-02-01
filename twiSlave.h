#ifndef TWI_SLAVE_H
#define TWI_SLAVE_H

void I2C_init(uint8_t address);
void I2C_stop(void);
void I2C_writeTxBuffer16Bit(uint8_t address,uint8_t dataH, uint8_t dataL);
void I2C_writeTxBuffer8Bit(uint8_t address,uint8_t data);
int16_t I2C_readRxBuffer16Bit(uint8_t address);
uint8_t I2C_readRxBuffer8Bit(uint8_t address);
bool I2C_gotMessage();
#endif // I2C_SLAVE_H
#ifndef TWI_SLAVE_H
#define TWI_SLAVE_H

void I2C_init(uint8_t address);
void I2C_stop(void);
void I2C_writeRegister(uint8_t address,uint8_t dataL, uint8_t dataH);
#endif // I2C_SLAVE_H
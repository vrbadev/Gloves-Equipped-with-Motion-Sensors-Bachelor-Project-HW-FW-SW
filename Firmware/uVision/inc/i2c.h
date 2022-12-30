#ifndef I2C_H__
#define I2C_H__

#include "BlueNRG1_conf.h"

void i2c_init(void);
int i2c_write(uint8_t reg, uint8_t* data, uint8_t len);
int i2c_read(uint8_t reg, uint8_t* data, uint8_t len);
uint8_t i2c_read_reg(uint8_t reg);
void i2c_change_bus(uint8_t bus);
void i2c_change_addr(uint8_t addr);

#endif

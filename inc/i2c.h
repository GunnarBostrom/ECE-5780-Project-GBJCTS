/**
I2C protocol

Supports:
    - only 7-bit address mode
    - 8-bit and 16-bit peripheral registers
    - either Standard-mode or Fast-mode

STM32F072 Discovery Board

Wiring:
STM PB6 (I2C1_SCL) → IMU SCL
STM PB6 (I2C1_SCL) → LiDAR SCL
STM PB7 (I2C1_SDA) → IMU SDA
STM PB7 (I2C1_SDA) → LiDAR SDA

*/

#ifndef I2C_H
#define I2C_H

#include <stdint.h>


/* ––––––––––––––– API (user facing) ––––––––––––––– */
// Configures and initializes I2C
void i2c_init(uint16_t i2c_freq);

// Writes to I2C slave
void i2c_write(uint8_t slave_addr, uint16_t reg_addr, uint8_t* data, uint8_t len);

// Reads from I2C slave
void i2c_read(uint8_t slave_addr, uint16_t reg_addr, uint8_t* buf, uint8_t len);



/* ––––––––––––––– Driver internal ––––––––––––––– */
// Sets RCC clocks
void GPIO_clocks_enable(void);

// Sets I2C frequency
void i2c_set_TIMINGR(uint16_t i2c_freq);

// Enables I2C after configure
void i2c_enable(void);

// // One shot write to slave
// void i2c_write(uint8_t slave_addr, uint16_t reg_addr, uint8_t* data, uint8_t len);

// Writes to slave
void i2c_write_transaction(uint8_t slave_addr, uint16_t reg_addr, uint8_t* data, uint8_t len);

// // Preps master for read
// void i2c_read(uint8_t slave_addr, uint16_t reg_addr, uint8_t* buf,uint8_t len);

// Reads from slave
void i2c_read_transaction(uint8_t slave_addr, uint8_t* buf, uint8_t len);



// FIXME: remove (temp for signaling)
void led_init(void);

#endif // I2C_H
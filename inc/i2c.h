/**
I2C protocol
STM32F072 Discovery Board

Wiring:
STM PB6 (I2C1_SCL) → IMU SCL
STM PB6 (I2C1_SCL) → LiDAR SCL
STM PB7 (I2C1_SDA) → IMU SDA
STM PB7 (I2C1_SDA) → LiDAR SDA

*/

// API - user facing
void i2c_init(uint8_t mode_freq);
void i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t* data, uint8_t len);
void i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t* buf, uint8_t len);

// i2c internal
void GPIO_clocks_enable(void);
void i2c_set_TIMINGR(uint8_t mode_freq);
void i2c_enable(void);
void i2c_set_write_params(uint8_t slave_addr, uint8_t len);
void i2c_write_transaction(uint8_t reg_addr, uint8_t* data, uint8_t len);
void i2c_set_read_params(uint8_t slave_addr, uint8_t len);
void i2c_read_transaction(uint8_t reg_addr, uint8_t* buf, uint8_t len);

void led_init(void);            // temp for signaling
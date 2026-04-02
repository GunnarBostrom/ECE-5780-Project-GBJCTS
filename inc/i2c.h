/**
I2C protocol

Wiring:
STM PB6 (I2C1_SCL) → IMU SCL
STM PB6 (I2C1_SCL) → LiDAR SCL
STM PB7 (I2C1_SDA) → IMU SDA
STM PB7 (I2C1_SDA) → LiDAR SDA

*/


void GPIO_clocks_enable(void);
void led_init(void);            // for signaling

void i2c_init(void);
void i2c_set_TIMINGR(void);
void i2c_enable(void);
void i2c_set_write_params(void);
void i2c_set_read_params(void);
void i2c_write(void);
void i2c_read(void);
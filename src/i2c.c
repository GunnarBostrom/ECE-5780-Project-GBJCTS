/**
I2C protocol

Wiring:
STM PB6 (I2C1_SCL) → IMU SCL
STM PB6 (I2C1_SCL) → LiDAR SCL
STM PB7 (I2C1_SDA) → IMU SDA
STM PB7 (I2C1_SDA) → LiDAR SDA

*/

#include "i2c.h"


void i2c_init() {
    
    
    GPIO_clocks_enable();
    led_init();


}

void GPIO_clocks_enable() {
  //RCC->AHBENR |= (1 << 18); // GPIOB for I2C2
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  //RCC->AHBENR |= (1 << 19); // GPIOC for LEDs
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    
  //RCC->APB1ENR |= (1 << 22); // for I2C2
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
}

void led_init() {
  // set PC6, PC7, PC8, PC9 to output mode
  GPIOC->MODER &= ~(0b11 << 12); // clear PC6 red
  GPIOC->MODER |=  (0b01 << 12); // set PC6 output

  GPIOC->MODER &= ~(0b11 << 14); // clear PC7 blue
  GPIOC->MODER |=  (0b01 << 14); // set PC7 output

  GPIOC->MODER &= ~(0b11 << 16); // clear PC8 orange
  GPIOC->MODER |=  (0b01 << 16); // set PC8 output

  GPIOC->MODER &= ~(0b11 << 18); // clear PC9 green
  GPIOC->MODER |=  (0b01 << 18); // set PC9 output
}

void i2c_set_TIMINGR() {}
void i2c_enable() {}
void i2c_set_write_params(void) {}
void i2c_set_read_params(void) {}
void i2c_write(void) {}
void i2c_read(void) {}
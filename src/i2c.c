/**
I2C protocol
STM32F072 Discovery Board

Wiring:
STM PB6 (I2C1_SCL) → IMU SCL
STM PB6 (I2C1_SCL) → LiDAR SCL
STM PB7 (I2C1_SDA) → IMU SDA
STM PB7 (I2C1_SDA) → LiDAR SDA

Notes:
    maybe do...
    STM PB13 (I2C2_SCL) → IMU SCL
    STM PB13 (I2C2_SCL) → LiDAR SCL
    STM PB11 (I2C2_SDA) → IMU SDA
    STM PB11 (I2C2_SDA) → LiDAR SDA

*/

#include "i2c.h"
//#include "stm32f0xx_hal.h"

#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_it.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/_intsup.h>
#include <sys/types.h>


void i2c_init(uint16_t i2c_freq) {
    
    
    GPIO_clocks_enable();
    //led_init();

    // set PB11 to alternate function mode, open-drain output type, I2C2_SDA as the alt function
    GPIOB->MODER &= ~(0b11 << 22);    // clear alt function mode
    GPIOB->MODER |= (0b10 << 22);     // set alt function mode

    GPIOB->OTYPER &= ~(0b1 << 11);    // clear output type
    GPIOB->OTYPER |= (0b1 << 11);     // set output type to open-drain

    GPIOB->AFR[1] &= ~(0b1111 << 12); // clear alt function type
    GPIOB->AFR[1] |= (0b0001 << 12);  // set alt function type to i2c2_sda (AF1)

    
    // set PB13 to alternate function mode, open-drain output type, I2C2_SCL as the alt function
    GPIOB->MODER &= ~(0b11 << 26);
    GPIOB->MODER |= (0b10 << 26); // alt function mode

    GPIOB->OTYPER &= ~(0b1 << 13);
    GPIOB->OTYPER |= (0b1 << 13); // set output type to open-drain

    GPIOB->AFR[1] &= ~(0b1111 << 20);
    GPIOB->AFR[1] |= (0b0101 << 20); // set alt function type to i2c2_scl (AF5)

    // // PB11 - I2C2_SDA
    // GPIOB->MODER  &= ~(3U << (11 * 2)); 
    // GPIOB->MODER  |=  (2U << (11 * 2)); 
    // GPIOB->OTYPER |=  (1U << 11);
    // GPIOB->AFR[1] &= ~(0xF << ((11 - 8) * 4));
    // GPIOB->AFR[1] |=  (1U  << ((11 - 8) * 4));

    // // PB13 - I2C2_SCL
    // GPIOB->MODER  &= ~(3U << (13 * 2));
    // GPIOB->MODER  |=  (2U << (13 * 2));
    // GPIOB->OTYPER |=  (1U << 13);
    // GPIOB->AFR[1] &= ~(0xF << ((13 - 8) * 4));
    // GPIOB->AFR[1] |=  (5U  << ((13 - 8) * 4));



    i2c_set_TIMINGR(i2c_freq);
    i2c_enable();


}

/**
 * @brief Writes data to a peripheral register
 * 
 * @param slave_addr  I2C slave address of peripheral
 * @param reg_addr    Target peripheral register address
 * @param data        Pointer to data buffer to write to target register
 * @param len         Number of bytes to write
 */
void i2c_write(uint8_t slave_addr, uint16_t reg_addr, uint8_t* data, uint8_t len) {
    
    // if (!i2c_set_write_params(slave_addr, reg_addr, len)) { return 0; }
    // if (!i2c_write_transaction(reg_addr, data, len, 1)) { return 0; }// send stop

    i2c_write_transaction(slave_addr, reg_addr, data, len);

    //return 1;
}

/**
 * @brief Reads data from a peripheral
 * 
 * @param slave_addr  I2C slave address of peripheral
 * @param reg_addr    Target peripheral register address
 * @param buf         Pointer to data buffer to write from target register
 * @param len         Number of bytes to write
 */
void i2c_read(uint8_t slave_addr, uint16_t reg_addr, uint8_t* buf, uint8_t len) {
    
    // if (!i2c_set_write_params(slave_addr, reg_addr, 0)) {return 0;};
    // if (!i2c_write_transaction(reg_addr, NULL, 0, 0)) {return 0;} // don't send stop
    // if (!i2c_set_read_params(slave_addr, len)) {return 0;}
    // if (!i2c_read_transaction(buf, len)) {return 0;}

    // return 1;

    i2c_write_transaction(slave_addr, reg_addr, NULL, 0);
    i2c_read_transaction(slave_addr, buf, len);
    
}

void GPIO_clocks_enable() {
  RCC->AHBENR |= (1 << 18); // GPIOB for I2C2
  //RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

//   RCC->AHBENR |= (1 << 19); // GPIOC for LEDs
//   //RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    
  RCC->APB1ENR |= (1 << 22); // for I2C2
  //RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
}

/**
 * This is temporary and is just to use LEDs as signals for debugging
 */
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

/**
 * @brief Sets I2C frequency.
 * 
 * This function is to blame for this I2C driver only
 * supporting Standard-mode and Fast-mode.
 * If an invalid frequency is provided, 100 kHz (Standard-mode) will
 * be selected.
 * 
 * @param i2c_freq  I2C frequency in kHz
*/
void i2c_set_TIMINGR(uint16_t i2c_freq) {
  
  switch(i2c_freq) { // 8MHz
    case 100: // Standard-mode - 100 kHz
        I2C2->TIMINGR =
            (0x1  << 28) |   // PRESC
            (0x4  << 20) |   // SCLDEL
            (0x2  << 16) |   // SDADEL
            (0x0F <<  8) |   // SCLH
            (0x13 <<  0);    // SCLL
            break;
    case 400: // Fast-mode - 400 kHz
        I2C2->TIMINGR =
            (0x0 << 28) |   // PRESC
            (0x3 << 20) |   // SCLDEL
            (0x1 << 16) |   // SDADEL
            (0x3 <<  8) |   // SCLH
            (0x9 <<  0);    // SCLL
            break;
    default: // Standard-mode - 100 kHz
        I2C2->TIMINGR =
            (0x1  << 28) |   // PRESC
            (0x4  << 20) |   // SCLDEL
            (0x2  << 16) |   // SDADEL
            (0x0F <<  8) |   // SCLH
            (0x13 <<  0);    // SCLL
            break;
  }

    // switch(i2c_freq) { // 48MHz

    //     case 100: // 100 kHz at 48MHz
    //         I2C2->TIMINGR =
    //             (0xB << 28) |  // PRESC
    //             (0x4 << 20) |  // SCLDEL
    //             (0x2 << 16) |  // SDADEL
    //             (0xF <<  8) |  // SCLH
    //             (0x13 << 0);   // SCLL
    //         break;
    //     case 400: // 400 kHz at 48MHz
    //         I2C2->TIMINGR =
    //             (0x5 << 28) |  // PRESC
    //             (0x3 << 20) |  // SCLDEL
    //             (0x1 << 16) |  // SDADEL
    //             (0x3 <<  8) |  // SCLH
    //             (0x9 <<  0);   // SCLL
    //         break;
    //     default: // Standard-mode - 100 kHz at 48MHz
    //         I2C2->TIMINGR =
    //             (0xB << 28) |  // PRESC
    //             (0x4 << 20) |  // SCLDEL
    //             (0x2 << 16) |  // SDADEL
    //             (0xF <<  8) |  // SCLH
    //             (0x13 << 0);   // SCLL
    //         break;
    // }
}

/**
 * @brief IC2 already configured, now enable.
 */
void i2c_enable() {
  
      // Disable first
    I2C2->CR1 &= ~I2C_CR1_PE;

    // Small delay (optional but safe)
    for (volatile int i = 0; i < 1000; i++);

    // Re-enable
    I2C2->CR1 |= I2C_CR1_PE;

  //SYSCFG->EXTICR[0] |= (0b100 << 4);
}



/**
 * @brief Writes data to slave.
 * 
 * @brief Preps master for writing to slave.
//  * 
//  * Returns 0 if slave does not ACK.
//  * 
//  * This and the i2c_set_read_params function are to 
//  * blame for this I2C driver only supporting 7-bit 
//  * address mode. Fix the SADD shift if you actually 
//  * care that much.

 * 
 * @param reg_addr  Peripheral register to be written to
 * @param data      Pointer to data to write to register
 * @param len       Number of bytes of data to write
 */
void i2c_write_transaction(uint8_t slave_addr, uint16_t reg_addr, uint8_t* data, uint8_t len) {
        // total bytes sent includes register as well
    uint8_t total_bytes = (reg_addr > 0xFF) ? len + 2 : len + 1;

    //I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); // clear NBYTES (16-23) and SADD (0-9)
    //I2C2->CR2 &= ~((0xFF << 16) | (0x3FF << 0)); // clear NBYTES (16-23) and SADD (0-9)
    // I2C2->CR2 &= ~(I2C_CR2_SADD |
    //            I2C_CR2_NBYTES |
    //            I2C_CR2_RD_WRN |
    //            I2C_CR2_START |
    //            I2C_CR2_STOP);
    // I2C2->CR2 &= ~((0x3FF << 0) | (0x7F << 16) | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND); // why 0x7F?
    //I2C2->CR2 &= ~((0x3FF << 0) | (0xFF << 16) | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND); // why 0x7F?
    I2C2->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND);

    I2C2->CR2 |= slave_addr << 1;   // SADD shifted for 7-bit address mode
    I2C2->CR2 |= total_bytes << 16; // NBYTES
    

    I2C2->CR2 &= ~(I2C_CR2_RD_WRN); // request write transfer

    I2C2->CR2 |= I2C_CR2_AUTOEND;
    I2C2->CR2 |= I2C_CR2_START;     // 13

    // SLAVE DOES NOT ACK HERE

    // wait for slave to ACK
    // while (!(I2C2->ISR & I2C_ISR_TXIS) && !(I2C2->ISR & I2C_ISR_NACKF)) {}
    // if (I2C2->ISR & I2C_ISR_NACKF) {
    //     I2C2->ICR |= I2C_ICR_NACKCF; // clear flag
    //     //I2C2->CR2 |= I2C_CR2_STOP;   // stop
    //     return;                    // Error: NACKF, return 0
    // }



    //while (!(I2C2->ISR & (1 << 1)) && !(I2C2->ISR & (1 << 4))); // wait for ready to transmit or error
    while (!(I2C2->ISR & I2C_ISR_TXIS) && !(I2C2->ISR & I2C_ISR_NACKF)) {} // wait for ready to transmit or error
    if (I2C2->ISR & I2C_ISR_NACKF) {
        // NACKF: probably should throw an error
        I2C2->ICR |= I2C_ICR_NACKCF; // clear flag
        I2C2->CR2 |= I2C_CR2_STOP;   // stop
        return;
    }

    if (reg_addr > 0xFF) { // if 16-bit register

        // send high byte 
        I2C2->TXDR = (reg_addr >> 8) & 0xFF;

        while (!(I2C2->ISR & I2C_ISR_TXIS) && !(I2C2->ISR & I2C_ISR_NACKF)) {} // wait for ready to transmit or error
        if (I2C2->ISR & I2C_ISR_NACKF) {
            // NACKF: probably should throw an error
            I2C2->ICR |= I2C_ICR_NACKCF; // clear flag
            I2C2->CR2 |= I2C_CR2_STOP;   // stop
            return;
        }

        // send low byte
        I2C2->TXDR = reg_addr & 0xFF;

    }
    else {
        while (!(I2C2->ISR & I2C_ISR_TXIS) && !(I2C2->ISR & I2C_ISR_NACKF)) {}
        if (I2C2->ISR & I2C_ISR_NACKF) {
            // NACKF: probably should throw an error
            I2C2->ICR |= I2C_ICR_NACKCF; // clear flag
            I2C2->CR2 |= I2C_CR2_STOP;   // stop
            return; 
        }

        I2C2->TXDR = reg_addr;
    }

    //write bytes to transmit
    for (int i = 0; i < len; i++)
    {
        while (!(I2C2->ISR & I2C_ISR_TXIS) && !(I2C2->ISR & I2C_ISR_NACKF)) {} // wait for ready to transmit or error
        if (I2C2->ISR & I2C_ISR_NACKF) {
            // NACKF: probably should throw an error
            I2C2->ICR |= I2C_ICR_NACKCF; // clear flag
            I2C2->CR2 |= I2C_CR2_STOP;   // stop
            return;
        }
        I2C2->TXDR = data[i];
    }

    while( !(I2C2->ISR & (1 << 6)) ) {}        // wait for transmit complete
    //while( !(I2C2->ISR & I2C_ISR_TC) ) {}
    //if (send_stop) {
        I2C2->CR2 |= I2C_CR2_STOP;                 // 14?
        while ( !(I2C2->ISR & I2C_ISR_STOPF) ) {}  // wait for stop condition
        I2C2->ICR |= I2C_ICR_STOPCF;               // clear flag
    //}


}


/**
 * @brief Reads data from slave.
 * @brief Preps the master for reading from slave.
 * 
 * This and the i2c_set_write_params function are to 
 * blame for this I2C driver only supporting 7-bit 
 * address mode. Fix the SADD shift if you actually 
 * care that much.
 * 
 * @param buf   Buffer to receive data from slave
 * @param len   Number of bytes to save to buffer
 */
void i2c_read_transaction(uint8_t slave_addr, uint8_t* buf, uint8_t len) {
    
    // don't need to account for address in total bytes read (address sent with write params)

    //I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); // clear NBYTES (16-23) and SADD (0-9)
    //I2C2->CR2 &= ~((0xFF << 16) | (0x3FF << 0)); // clear NBYTES (16-23) and SADD (0-9)
    I2C2->CR2 &= ~(I2C_CR2_SADD |
               I2C_CR2_NBYTES |
               I2C_CR2_RD_WRN |
               I2C_CR2_START |
               I2C_CR2_STOP);
    
    // I2C2->CR2 = 
    //     (len << 16) |  // nbytes
    //     (0x1 << 10) |  // rd_wrn - request read transfer
    //     (0x69 << 1);   // sadd

    I2C2->CR2 |= len << 16;        // NBYTES
    I2C2->CR2 |= slave_addr << 1;  // SADD shifted for 7-bit address mode

    I2C2->CR2 |= I2C_CR2_RD_WRN; // request read transfer
    I2C2->CR2 |= I2C_CR2_START;

    for (int i = 0; i < len; i++) {

        while ( !(I2C2->ISR & I2C_ISR_RXNE) && !(I2C2->ISR & I2C_ISR_NACKF) ) {} // wait for ready to read or error

        if (I2C2->ISR & I2C_ISR_NACKF) {
            I2C2->ICR |= I2C_ICR_NACKCF; // clear flag
            I2C2->CR2 |= I2C_CR2_STOP;   // stop
            // NACKF: probably should throw an error
        }
        buf[i] = I2C2->RXDR;

    }

    while ( !(I2C2->ISR & I2C_ISR_TC) ) {}     // wait for transfer complete
    I2C2->CR2 |= I2C_CR2_STOP;                 // stop
    while ( !(I2C2->ISR & I2C_ISR_STOPF) ) {}  // wait for stop condition
    I2C2->ICR |= I2C_ICR_STOPCF;               // clear flag

}
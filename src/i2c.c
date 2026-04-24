/**
 * I2C driver
 * STM32F072 Discovery Board
 *
 * Wiring (active configuration):
 *   PB6  — I2C1_SCL
 *   PB7  — I2C1_SDA
 *
 * Supports:
 *   - 7-bit addressing
 *   - 8-bit and 16-bit register addresses (auto-detected by value)
 *   - Standard-mode (100 kHz) and Fast-mode (400 kHz)
 *   - Bus recovery (9-clock bit-bang) called automatically in i2c_init()
 *   - 10 ms timeout on every blocking ISR poll — no more infinite hangs
 */

#include "i2c.h"
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_it.h"
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Internal helpers                                                     */
/* ------------------------------------------------------------------ */
void GPIO_clocks_enable(void);
void i2c_bus_reset(void);
void i2c_set_TIMINGR(uint16_t i2c_freq);
void i2c_enable(void);
void i2c_write_transaction(uint8_t slave_addr, uint16_t reg_addr,
                            uint8_t *data, uint8_t len, uint8_t send_stop);
void i2c_read_transaction(uint8_t slave_addr, uint8_t *buf, uint8_t len);

/* Timeout in milliseconds for every blocking ISR poll */
#define I2C_TIMEOUT_MS  10U

/**
 * @brief Macro: wait for condition with a 10 ms timeout.
 *
 * If the timeout expires the macro forces a STOP, clears error flags,
 * and returns from the enclosing function.
 */
#define I2C_WAIT(condition)                                         \
    do {                                                            \
        uint32_t _t0 = HAL_GetTick();                              \
        while (!(condition)) {                                      \
            if ((HAL_GetTick() - _t0) >= I2C_TIMEOUT_MS) {        \
                I2C1->CR2  |= I2C_CR2_STOP;                        \
                I2C1->ICR   = I2C_ICR_NACKCF | I2C_ICR_ARLOCF |   \
                               I2C_ICR_BERRCF | I2C_ICR_STOPCF;    \
                return;                                             \
            }                                                       \
        }                                                           \
    } while (0)

/* Same as above but the enclosing function returns a value */
#define I2C_WAIT_RET(condition, retval)                             \
    do {                                                            \
        uint32_t _t0 = HAL_GetTick();                              \
        while (!(condition)) {                                      \
            if ((HAL_GetTick() - _t0) >= I2C_TIMEOUT_MS) {        \
                I2C1->CR2  |= I2C_CR2_STOP;                        \
                I2C1->ICR   = I2C_ICR_NACKCF | I2C_ICR_ARLOCF |   \
                               I2C_ICR_BERRCF | I2C_ICR_STOPCF;    \
                return (retval);                                    \
            }                                                       \
        }                                                           \
    } while (0)

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void i2c_init(uint16_t i2c_freq)
{
    GPIO_clocks_enable();
    i2c_bus_reset();        /* 9-clock recovery before configuring peripheral */

    /* PB6 — I2C1_SCL (AF1) */
    GPIOB->MODER  &= ~(0b11   << 12);
    GPIOB->MODER  |=  (0b10   << 12);   /* alternate function */
    GPIOB->OTYPER |=  (0b1    <<  6);   /* open-drain */
    GPIOB->AFR[0] &= ~(0xF    << 24);
    GPIOB->AFR[0] |=  (0x1    << 24);   /* AF1 = I2C1_SCL */

    /* PB7 — I2C1_SDA (AF1) */
    GPIOB->MODER  &= ~(0b11   << 14);
    GPIOB->MODER  |=  (0b10   << 14);
    GPIOB->OTYPER |=  (0b1    <<  7);
    GPIOB->AFR[0] &= ~(0xF    << 28);
    GPIOB->AFR[0] |=  (0x1    << 28);   /* AF1 = I2C1_SDA */

    i2c_set_TIMINGR(i2c_freq);
    i2c_enable();
}

void i2c_write(uint8_t slave_addr, uint16_t reg_addr,
               uint8_t *data, uint8_t len)
{
    i2c_write_transaction(slave_addr, reg_addr, data, len, 1);
}

void i2c_read(uint8_t slave_addr, uint16_t reg_addr,
              uint8_t *buf, uint8_t len)
{
    i2c_write_transaction(slave_addr, reg_addr, NULL, 0, 0);
    i2c_read_transaction(slave_addr, buf, len);
}

/* ------------------------------------------------------------------ */
/* Clock and peripheral enable                                          */
/* ------------------------------------------------------------------ */

void GPIO_clocks_enable(void)
{
    RCC->AHBENR  |= (1 << 18);  /* GPIOB */

    RCC->APB1RSTR |=  (1 << 21); /* reset I2C1 */
    RCC->APB1RSTR &= ~(1 << 21);
    RCC->APB1ENR  |=  (1 << 21); /* enable I2C1 */
}

/* ------------------------------------------------------------------ */
/* Bus recovery — bit-bang 9 SCL clocks on PB6/PB7 to release a        */
/* stuck slave, then issue a STOP condition.                            */
/* Called automatically by i2c_init() on every boot or re-init.        */
/* ------------------------------------------------------------------ */

void i2c_bus_reset(void)
{
    /* Take PB6 (SCL) and PB7 (SDA) as plain GPIO outputs */
    GPIOB->MODER &= ~((0b11 << 12) | (0b11 << 14));
    GPIOB->MODER |=  ((0b01 << 12) | (0b01 << 14));

    /* Both lines high before starting */
    GPIOB->BSRR = (1 << 6) | (1 << 7);
    for (volatile int d = 0; d < 600; d++);

    /* Clock SCL up to 9 times; stop early if SDA is released (goes high) */
    for (int i = 0; i < 9; i++)
    {
        if (GPIOB->IDR & (1 << 7))  /* SDA already high — bus is free */
        {
            break;
        }

        GPIOB->BSRR = (1 << (6 + 16)); /* SCL low  */
        for (volatile int d = 0; d < 600; d++);
        GPIOB->BSRR = (1 << 6);        /* SCL high */
        for (volatile int d = 0; d < 600; d++);
    }

    /* Generate STOP: SDA low → high while SCL is high */
    GPIOB->BSRR = (1 << (7 + 16));     /* SDA low  */
    for (volatile int d = 0; d < 600; d++);
    GPIOB->BSRR = (1 << 6);            /* SCL high */
    for (volatile int d = 0; d < 600; d++);
    GPIOB->BSRR = (1 << 7);            /* SDA high */
    for (volatile int d = 0; d < 600; d++);

    /* Restore PB6 and PB7 to alternate-function mode for I2C peripheral */
    GPIOB->MODER &= ~((0b11 << 12) | (0b11 << 14));
    GPIOB->MODER |=  ((0b10 << 12) | (0b10 << 14));
}

/* ------------------------------------------------------------------ */
/* TIMINGR — 48 MHz I2C1                                                */
/* ------------------------------------------------------------------ */

void i2c_set_TIMINGR(uint16_t i2c_freq)
{
    switch (i2c_freq)
    {
        case 400:   /* Fast-mode 400 kHz @ 48 MHz */
            I2C1->TIMINGR =
                (0x5 << 28) |   /* PRESC  */
                (0x3 << 20) |   /* SCLDEL */
                (0x1 << 16) |   /* SDADEL */
                (0x3 <<  8) |   /* SCLH   */
                (0x9 <<  0);    /* SCLL   */
            break;

        case 100:   /* Standard-mode 100 kHz @ 48 MHz */
        default:
            I2C1->TIMINGR =
                (0xB << 28) |
                (0x4 << 20) |
                (0x2 << 16) |
                (0xF <<  8) |
                (0x13 << 0);
            break;
    }
}

/* ------------------------------------------------------------------ */
/* Peripheral enable                                                    */
/* ------------------------------------------------------------------ */

void i2c_enable(void)
{
    I2C1->CR1 &= ~I2C_CR1_PE;
    for (volatile int i = 0; i < 1000; i++);
    I2C1->CR1 |= I2C_CR1_PE;
}

/* ------------------------------------------------------------------ */
/* Write transaction                                                    */
/*                                                                      */
/* Sends: [START][ADDR+W][reg_high?][reg_low][data...][STOP?]          */
/* 16-bit register addresses are detected automatically (reg > 0xFF).  */
/* Pass send_stop=0 to leave TC set for a subsequent repeated START.   */
/* ------------------------------------------------------------------ */

void i2c_write_transaction(uint8_t slave_addr, uint16_t reg_addr,
                             uint8_t *data, uint8_t len, uint8_t send_stop)
{
    uint8_t total_bytes = (reg_addr > 0xFF) ? len + 2 : len + 1;

    I2C1->CR2 &= ~(I2C_CR2_SADD   | I2C_CR2_NBYTES |
                   I2C_CR2_RD_WRN | I2C_CR2_START   | I2C_CR2_AUTOEND);

    I2C1->CR2 |= (slave_addr << 1);        /* SADD — 7-bit address */
    I2C1->CR2 |= ((uint32_t)total_bytes << 16); /* NBYTES */
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;          /* write direction */
    I2C1->CR2 |= I2C_CR2_START;

    /* Wait for TXIS (ready to send) or NACK */
    I2C_WAIT((I2C1->ISR & I2C_ISR_TXIS) || (I2C1->ISR & I2C_ISR_NACKF));
    if (I2C1->ISR & I2C_ISR_NACKF) {
        I2C1->ICR |= I2C_ICR_NACKCF;
        I2C1->CR2 |= I2C_CR2_STOP;
        return;
    }

    /* Send register address — MSB first for 16-bit registers */
    if (reg_addr > 0xFF) {
        I2C1->TXDR = (reg_addr >> 8) & 0xFF;

        I2C_WAIT((I2C1->ISR & I2C_ISR_TXIS) || (I2C1->ISR & I2C_ISR_NACKF));
        if (I2C1->ISR & I2C_ISR_NACKF) {
            I2C1->ICR |= I2C_ICR_NACKCF;
            I2C1->CR2 |= I2C_CR2_STOP;
            return;
        }
    }
    I2C1->TXDR = reg_addr & 0xFF;

    /* Send data bytes */
    for (int i = 0; i < len; i++)
    {
        I2C_WAIT((I2C1->ISR & I2C_ISR_TXIS) || (I2C1->ISR & I2C_ISR_NACKF));
        if (I2C1->ISR & I2C_ISR_NACKF) {
            I2C1->ICR |= I2C_ICR_NACKCF;
            I2C1->CR2 |= I2C_CR2_STOP;
            return;
        }
        I2C1->TXDR = data[i];
    }

    /* Wait for transfer complete */
    I2C_WAIT(I2C1->ISR & I2C_ISR_TC);

    if (send_stop) {
        I2C1->CR2 |= I2C_CR2_STOP;
        I2C_WAIT(I2C1->ISR & I2C_ISR_STOPF);
        I2C1->ICR |= I2C_ICR_STOPCF;
    }
    /* If not sending STOP, TC remains set and the bus is held for the
       repeated START that i2c_read_transaction() will issue. */
}

/* ------------------------------------------------------------------ */
/* Read transaction                                                     */
/*                                                                      */
/* Sends: [rSTART][ADDR+R][data...][STOP]                              */
/* Typically follows a write transaction that set the register pointer. */
/* ------------------------------------------------------------------ */

void i2c_read_transaction(uint8_t slave_addr, uint8_t *buf, uint8_t len)
{
    I2C1->CR2 &= ~(I2C_CR2_SADD   | I2C_CR2_NBYTES |
                   I2C_CR2_RD_WRN | I2C_CR2_START   |
                   I2C_CR2_STOP   | I2C_CR2_AUTOEND);

    I2C1->CR2 |= ((uint32_t)len << 16);    /* NBYTES */
    I2C1->CR2 |= (slave_addr << 1);        /* SADD   */
    I2C1->CR2 |= I2C_CR2_RD_WRN;          /* read direction */
    I2C1->CR2 |= I2C_CR2_START;

    for (int i = 0; i < len; i++)
    {
        I2C_WAIT((I2C1->ISR & I2C_ISR_RXNE) || (I2C1->ISR & I2C_ISR_NACKF));
        if (I2C1->ISR & I2C_ISR_NACKF) {
            I2C1->ICR |= I2C_ICR_NACKCF;
            I2C1->CR2 |= I2C_CR2_STOP;
            return;
        }
        buf[i] = I2C1->RXDR;
    }

    I2C_WAIT(I2C1->ISR & I2C_ISR_TC);
    I2C1->CR2 |= I2C_CR2_STOP;
    I2C_WAIT(I2C1->ISR & I2C_ISR_STOPF);
    I2C1->ICR |= I2C_ICR_STOPCF;
}

/**
 * IMU driver
 *
 * STM LSM6DS3
 * LSM6DS3 can operate at 100 kHz or 400 kHz.
 *
 * Data provided:
 *   - 3-axis acceleration  (ax_g / ay_g / az_g  in mg)
 *   - 3-axis angular rate  (gx_dps / gy_dps / gz_dps  in mdps)
 *   - temperature (raw, conversion: celsius = raw/16 + 25)
 *
 * Hard-coded configuration:
 *   - Accel: 416 Hz ODR, ±8 g full-scale, 400 Hz anti-aliasing filter
 *   - Gyro:  416 Hz ODR, ±2000 dps full-scale
 */

#include "config.h"
#include "i2c.h"
#include "imu.h"

#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* Register map                                                         */
/* ------------------------------------------------------------------ */
#define IMU_ADDR        0x6B    /* default I2C slave address (SA0=VDD) */
#define WHO_AM_I_REG    0x0F
#define IMU_ID          0x69

#define TEMP_REG        0x20
#define GYRO_REG        0x22
#define ACCEL_REG       0x28

#define ACCEL_CFG       0x10    /* CTRL1_XL */
#define GYRO_CFG        0x11    /* CTRL2_G  */
#define CTRL3_C         0x12
#define CTRL4_C         0x13
#define INT1_CFG        0x0D    /* INT1_CTRL */

/* ------------------------------------------------------------------ */
/* Config bit-fields                                                    */
/* ------------------------------------------------------------------ */
#define ACCEL_ODR_416HZ  (0b0110 << 4)
#define ACCEL_FS_8G      (0b11   << 2)
#define ACCEL_BW_400HZ   (0b00   << 0)

#define GYRO_ODR_416HZ   (0b0110 << 4)
#define GYRO_FS_2000DPS  (0b11   << 2)

#define INT1_GYRO_EN     (0b1 << 1)
#define INT1_ACCEL_EN    (0b1 << 0)

/* ------------------------------------------------------------------ */
/* Sensitivity — chosen full-scale                                      */
/*   Accel ±8 g  : 0.244 mg/LSB                                        */
/*   Gyro ±2000 dps : 70 mdps/LSB                                      */
/* ------------------------------------------------------------------ */
#define ACCEL_SENS_NUM  244     /* numerator   (244/1000 = 0.244 mg/LSB) */
#define ACCEL_SENS_DEN  1000
#define GYRO_SENS_NUM   70      /* numerator   (70/1000  = 0.070 dps/LSB)*/
#define GYRO_SENS_DEN   1000

/* ------------------------------------------------------------------ */
/* Shared flag — set by EXTI ISR, cleared by main loop                  */
/* ------------------------------------------------------------------ */
volatile uint8_t imu_ready = 0;

/* ------------------------------------------------------------------ */
/* Static helpers                                                       */
/* ------------------------------------------------------------------ */
static void imu_convert_units(IMU_t *imu);

/* ================================================================== */
#if USE_IMU   /* real hardware */
/* ================================================================== */

/**
 * @brief Initialize the LSM6DS3.
 *
 * Verifies WHO_AM_I, writes BDU + DRDY_MASK, enables INT1,
 * then sets accel and gyro ODR/FS.
 *
 * Bus recovery (9-clock reset) is already performed by i2c_init()
 * before this function is called, so no separate recovery is needed
 * here.  A short delay after init lets the ODR settle before the
 * first interrupt fires.
 *
 * @param imu  Pointer to driver struct (only used to zero-initialize).
 * @return true on success, false if WHO_AM_I does not match.
 */
bool imu_init(IMU_t *imu, uint8_t slave_addr)
{
    /* Zero-initialise the struct */
    imu->slave_addr = slave_addr;
    imu->ax = imu->ay = imu->az = 0;
    imu->gx = imu->gy = imu->gz = 0;
    imu->temp_raw = 0;
    imu->temp_c = 0.0f;
    imu->ax_g = imu->ay_g = imu->az_g = 0;
    imu->gx_dps = imu->gy_dps = imu->gz_dps = 0;

    /* ---- Verify device ---- */
    uint8_t device_id = 0;
    i2c_read(IMU_ADDR, WHO_AM_I_REG, &device_id, 1);

    if (device_id != IMU_ID)
    {
        /* Retry once — first read after a cold bus can return garbage */
        HAL_Delay(5);
        i2c_read(IMU_ADDR, WHO_AM_I_REG, &device_id, 1);

        if (device_id != IMU_ID)
        {
            /* Wrong device: blink blue + orange rapidly so it is obvious */
            while (1)
            {
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7 | GPIO_PIN_8);
                HAL_Delay(150);
            }
            return false;   /* unreachable, but satisfies compiler */
        }
    }

    /* ---- Block data update — freeze registers during read ---- */
    uint8_t ctrl3 = 0x40;   /* BDU bit */
    i2c_write(IMU_ADDR, CTRL3_C, &ctrl3, 1);

    /* ---- Data-ready mask — suppress spurious INT1 at startup ---- */
    uint8_t ctrl4 = 0x08;   /* DRDY_MASK bit */
    i2c_write(IMU_ADDR, CTRL4_C, &ctrl4, 1);

    /* ---- Configure ODR and full-scale ---- */
    uint8_t accel_cfg = ACCEL_ODR_416HZ | ACCEL_FS_8G | ACCEL_BW_400HZ;
    i2c_write(IMU_ADDR, ACCEL_CFG, &accel_cfg, 1);

    uint8_t gyro_cfg = GYRO_ODR_416HZ | GYRO_FS_2000DPS;
    i2c_write(IMU_ADDR, GYRO_CFG, &gyro_cfg, 1);

    /* ---- Enable INT1 for gyro and accel data-ready ---- */
    uint8_t int1_cfg = INT1_GYRO_EN | INT1_ACCEL_EN;
    i2c_write(IMU_ADDR, INT1_CFG, &int1_cfg, 1);

    /* Short settle time for ODR to stabilize before first interrupt */
    HAL_Delay(10);

    return true;
}

/**
 * @brief Read all inertial data (gyro + accel) in one burst.
 *
 * The LSM6DS3 auto-increments its register pointer, so reading 12 bytes
 * starting at GYRO_REG captures both gyro (0x22-0x27) and accel (0x28-0x2D).
 */
void imu_read(IMU_t *imu)
{
    uint8_t buf[12];

    i2c_read(IMU_ADDR, GYRO_REG, buf, 12);

    imu->gx = (int16_t)(buf[0]  | (buf[1]  << 8));
    imu->gy = (int16_t)(buf[2]  | (buf[3]  << 8));
    imu->gz = (int16_t)(buf[4]  | (buf[5]  << 8));
    imu->ax = (int16_t)(buf[6]  | (buf[7]  << 8));
    imu->ay = (int16_t)(buf[8]  | (buf[9]  << 8));
    imu->az = (int16_t)(buf[10] | (buf[11] << 8));

    imu_convert_units(imu);
}

/** @brief Read accelerometer only (6 bytes). */
void imu_read_accel(IMU_t *imu)
{
    uint8_t buf[6];

    i2c_read(IMU_ADDR, ACCEL_REG, buf, 6);
    imu->ax = (int16_t)(buf[0] | (buf[1] << 8));
    imu->ay = (int16_t)(buf[2] | (buf[3] << 8));
    imu->az = (int16_t)(buf[4] | (buf[5] << 8));

    imu_convert_units(imu);
}

/** @brief Read gyroscope only (6 bytes). */
void imu_read_gyro(IMU_t *imu)
{
    uint8_t buf[6];

    i2c_read(IMU_ADDR, GYRO_REG, buf, 6);
    imu->gx = (int16_t)(buf[0] | (buf[1] << 8));
    imu->gy = (int16_t)(buf[2] | (buf[3] << 8));
    imu->gz = (int16_t)(buf[4] | (buf[5] << 8));

    imu_convert_units(imu);
}

/**
 * @brief Read temperature register (2 bytes, raw).
 *
 * Celsius conversion (not applied here):
 *   temp_c = (raw / 16) + 25
 */
void imu_read_temp(IMU_t *imu)
{
    uint8_t buf[2];

    i2c_read(IMU_ADDR, TEMP_REG, buf, 2);
    imu->temp_raw = (int16_t)(buf[0] | (buf[1] << 8));
}

/** @brief Read all sensor data including temperature (14-byte burst). */
void imu_read_all(IMU_t *imu)
{
    uint8_t buf[14];

    i2c_read(IMU_ADDR, TEMP_REG, buf, 14);
    imu->temp_raw = (int16_t)(buf[0]  | (buf[1]  << 8));
    imu->gx   = (int16_t)(buf[2]  | (buf[3]  << 8));
    imu->gy   = (int16_t)(buf[4]  | (buf[5]  << 8));
    imu->gz   = (int16_t)(buf[6]  | (buf[7]  << 8));
    imu->ax   = (int16_t)(buf[8]  | (buf[9]  << 8));
    imu->ay   = (int16_t)(buf[10] | (buf[11] << 8));
    imu->az   = (int16_t)(buf[12] | (buf[13] << 8));

    imu_convert_units(imu);
}

/**
 * @brief Convert raw LSBs to integer mg and mdps (no float arithmetic).
 *
 * Results are in mg and mdps so the STM32F0 (no FPU) stays in integer.
 * To get g:   ax_g / 1000.0f.
 * To get dps: gx_dps / 1000.0f.
 */
static void imu_convert_units(IMU_t *imu)
{
    imu->ax_g   = (int32_t)imu->ax * ACCEL_SENS_NUM / ACCEL_SENS_DEN;
    imu->ay_g   = (int32_t)imu->ay * ACCEL_SENS_NUM / ACCEL_SENS_DEN;
    imu->az_g   = (int32_t)imu->az * ACCEL_SENS_NUM / ACCEL_SENS_DEN;

    imu->gx_dps = (int32_t)imu->gx * GYRO_SENS_NUM / GYRO_SENS_DEN;
    imu->gy_dps = (int32_t)imu->gy * GYRO_SENS_NUM / GYRO_SENS_DEN;
    imu->gz_dps = (int32_t)imu->gz * GYRO_SENS_NUM / GYRO_SENS_DEN;
}

/* ================================================================== */
#else   /* stub implementations for builds without real hardware */
/* ================================================================== */

bool imu_init(IMU_t *imu, uint8_t slave_addr) { (void)imu; (void)slave_addr; return true; }
void imu_read(IMU_t *imu)      { (void)imu; }
void imu_read_accel(IMU_t *imu){ (void)imu; }
void imu_read_gyro(IMU_t *imu) { (void)imu; }
void imu_read_temp(IMU_t *imu) { (void)imu; }
void imu_read_all(IMU_t *imu)  { (void)imu; }

#endif /* USE_IMU */

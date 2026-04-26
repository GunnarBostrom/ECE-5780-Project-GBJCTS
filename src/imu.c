/**
 * IMU driver — STM LSM6DS3
 *
 * Accel: 416 Hz ODR, ±8 g,    sensitivity 0.244 mg/LSB
 * Gyro:  416 Hz ODR, ±2000 dps, sensitivity 0.070 dps/LSB
 * Output units: mg (milli-g) and mdps (milli-deg/s) — no float.
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
#define IMU_ADDR        0x6B
#define WHO_AM_I_REG    0x0F
#define IMU_ID          0x69

#define TEMP_REG        0x20
#define GYRO_REG        0x22
#define ACCEL_REG       0x28
#define STATUS_REG      0x1E

#define ACCEL_CFG       0x10    /* CTRL1_XL */
#define GYRO_CFG        0x11    /* CTRL2_G  */
#define CTRL3_C         0x12
#define CTRL4_C         0x13
#define INT1_CFG        0x0D    /* INT1_CTRL */

/* ------------------------------------------------------------------ */
/* Config bit-fields                                                    */
/* ------------------------------------------------------------------ */
#define ACCEL_ODR_416HZ     (0b0110 << 4)
#define ACCEL_FS_8G         (0b11   << 2)
#define ACCEL_BW_400HZ      (0b00   << 0)

#define GYRO_ODR_416HZ      (0b0110 << 4)
#define GYRO_FS_2000DPS     (0b11   << 2)

#define INT1_GYRO_EN        (1u << 1)
#define INT1_ACCEL_EN       (1u << 0)
#define STATUS_GYRO_READY   (1u << 1)

/* CTRL3_C: BDU (freeze output regs during read) + IF_INC (auto-increment
 * register address during burst reads — required for 12-byte gyro+accel read).
 * Default power-on value has IF_INC set; writing 0x40 (BDU only) would clear
 * it and break all burst reads. */
#define CTRL3_BDU           (1u << 6)
#define CTRL3_IF_INC        (1u << 2)

/* CTRL4_C: DRDY_MASK — gate INT1 until both gyro and accel data are ready,
 * preventing spurious double pulses when the two ODRs slip slightly. */
#define CTRL4_DRDY_MASK     (1u << 3)

/* ------------------------------------------------------------------ */
/* Sensitivity                                                          */
/* ------------------------------------------------------------------ */
#define ACCEL_SENS_NUM  244     /* 244/1000 = 0.244 mg/LSB (±8 g FS) */
#define ACCEL_SENS_DEN  1000
#define GYRO_SENS_NUM    70     /*  70/1000 = 0.070 dps/LSB (±2000 dps FS) */
#define GYRO_SENS_DEN   1000

/* ------------------------------------------------------------------ */
/* Gyro calibration settings                                            */
/* ------------------------------------------------------------------ */
#define GYRO_CAL_WARMUP_SAMPLES         128U
#define GYRO_CAL_READY_TIMEOUT_MS       100U
#define GYRO_CAL_STABILITY_THRESHOLD    200   /* max allowed peak-to-peak in raw counts */

/* ------------------------------------------------------------------ */
/* Module globals                                                       */
/* ------------------------------------------------------------------ */
volatile uint8_t imu_ready      = 0;
volatile uint8_t imu_error_code = IMU_ERROR_NONE;

/* ------------------------------------------------------------------ */
/* Static helpers                                                       */
/* ------------------------------------------------------------------ */
static void imu_convert_units(IMU_t *imu);
static void imu_read_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz);
static bool imu_wait_for_ready(uint32_t timeout_ms);
static bool imu_status_gyro_ready(void);

/* ================================================================== */
#if USE_IMU
/* ================================================================== */

bool imu_init(IMU_t *imu, uint8_t slave_addr)
{
    imu->slave_addr  = slave_addr;
    imu->ax = imu->ay = imu->az = 0;
    imu->gx = imu->gy = imu->gz = 0;
    imu->temp_raw = 0;
    imu->ax_mg = imu->ay_mg = imu->az_mg = 0;
    imu->gx_mdps = imu->gy_mdps = imu->gz_mdps = 0;
    imu->gx_bias = imu->gy_bias = imu->gz_bias = 0;

    /* Verify device identity with one retry */
    uint8_t device_id = 0;
    i2c_read(IMU_ADDR, WHO_AM_I_REG, &device_id, 1);
    if (device_id != IMU_ID)
    {
        HAL_Delay(5);
        i2c_read(IMU_ADDR, WHO_AM_I_REG, &device_id, 1);
        if (device_id != IMU_ID)
        {
            while (1)
            {
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7 | GPIO_PIN_8);
                HAL_Delay(150);
            }
            return false;
        }
    }

    imu_error_code = IMU_ERROR_NONE;

    /* CTRL3_C: BDU + IF_INC.  IF_INC MUST be set for burst reads to work;
     * the power-on default has it set, but writing only BDU (0x40) clears it. */
    uint8_t ctrl3 = CTRL3_BDU | CTRL3_IF_INC;
    i2c_write(IMU_ADDR, CTRL3_C, &ctrl3, 1);

    /* CTRL4_C: gate INT1 until both XL and gyro data-ready */
    uint8_t ctrl4 = CTRL4_DRDY_MASK;
    i2c_write(IMU_ADDR, CTRL4_C, &ctrl4, 1);

    /* Accel: 416 Hz, ±8 g, 400 Hz AA filter */
    uint8_t accel_cfg = ACCEL_ODR_416HZ | ACCEL_FS_8G | ACCEL_BW_400HZ;
    i2c_write(IMU_ADDR, ACCEL_CFG, &accel_cfg, 1);

    /* Gyro: 416 Hz, ±2000 dps */
    uint8_t gyro_cfg = GYRO_ODR_416HZ | GYRO_FS_2000DPS;
    i2c_write(IMU_ADDR, GYRO_CFG, &gyro_cfg, 1);

    /* Enable INT1 for gyro + accel data-ready */
    uint8_t int1_cfg = INT1_GYRO_EN | INT1_ACCEL_EN;
    i2c_write(IMU_ADDR, INT1_CFG, &int1_cfg, 1);

    HAL_Delay(10);  /* let ODR settle before first interrupt */
    return true;
}

/**
 * @brief Measure and store the gyro zero-rate offset.
 *
 * Collects @p sample_count gyro readings after a warmup period, averages
 * them, and stores the result in imu->gx/gy/gz_bias.  The bias is
 * subtracted from every subsequent imu_convert_units() call.
 *
 * During calibration INT1 is configured for gyro-only so the accel DRDY
 * does not keep the line latched between reads.
 *
 * @return true  Calibration succeeded.
 * @return false Timeout waiting for data, or motion detected (peak-to-peak
 *               spread exceeds GYRO_CAL_STABILITY_THRESHOLD raw counts).
 */
bool imu_calibrate_gyro(IMU_t *imu, uint16_t sample_count)
{
    if (sample_count == 0U)
    {
        return false;
    }

    imu_error_code = IMU_ERROR_NONE;
    imu->gx_bias = imu->gy_bias = imu->gz_bias = 0;
    imu_ready = 0;

    /* Use gyro-only DRDY on INT1 during calibration so accel DRDY cannot
     * keep the line asserted while we're only draining the gyro register. */
    uint8_t cal_int1  = INT1_GYRO_EN;
    uint8_t run_int1  = INT1_GYRO_EN | INT1_ACCEL_EN;
    i2c_write(IMU_ADDR, INT1_CFG, &cal_int1, 1);

    /* Warmup: discard first N samples (ODR settling, filter fill) */
    for (uint16_t i = 0; i < GYRO_CAL_WARMUP_SAMPLES; ++i)
    {
        int16_t gx_raw, gy_raw, gz_raw;
        if (!imu_wait_for_ready(GYRO_CAL_READY_TIMEOUT_MS))
        {
            imu_error_code = IMU_ERROR_CAL_TIMEOUT;
            i2c_write(IMU_ADDR, INT1_CFG, &run_int1, 1);
            return false;
        }
        imu_read_gyro_raw(&gx_raw, &gy_raw, &gz_raw);
    }

    /* Accumulate calibration samples */
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t gx_min = INT16_MAX, gy_min = INT16_MAX, gz_min = INT16_MAX;
    int16_t gx_max = INT16_MIN, gy_max = INT16_MIN, gz_max = INT16_MIN;

    for (uint16_t i = 0; i < sample_count; ++i)
    {
        int16_t gx_raw, gy_raw, gz_raw;
        if (!imu_wait_for_ready(GYRO_CAL_READY_TIMEOUT_MS))
        {
            imu_error_code = IMU_ERROR_CAL_TIMEOUT;
            i2c_write(IMU_ADDR, INT1_CFG, &run_int1, 1);
            return false;
        }
        imu_read_gyro_raw(&gx_raw, &gy_raw, &gz_raw);

        gx_sum += gx_raw;  gy_sum += gy_raw;  gz_sum += gz_raw;
        if (gx_raw < gx_min) gx_min = gx_raw;
        if (gy_raw < gy_min) gy_min = gy_raw;
        if (gz_raw < gz_min) gz_min = gz_raw;
        if (gx_raw > gx_max) gx_max = gx_raw;
        if (gy_raw > gy_max) gy_max = gy_raw;
        if (gz_raw > gz_max) gz_max = gz_raw;
    }

    /* Reject if the board moved during calibration */
    if (((int32_t)gx_max - gx_min) > GYRO_CAL_STABILITY_THRESHOLD ||
        ((int32_t)gy_max - gy_min) > GYRO_CAL_STABILITY_THRESHOLD ||
        ((int32_t)gz_max - gz_min) > GYRO_CAL_STABILITY_THRESHOLD)
    {
        imu_error_code = IMU_ERROR_CAL_MOTION;
        i2c_write(IMU_ADDR, INT1_CFG, &run_int1, 1);
        return false;
    }

    imu->gx_bias = (int16_t)(gx_sum / (int32_t)sample_count);
    imu->gy_bias = (int16_t)(gy_sum / (int32_t)sample_count);
    imu->gz_bias = (int16_t)(gz_sum / (int32_t)sample_count);

    imu_ready = 0;
    i2c_write(IMU_ADDR, INT1_CFG, &run_int1, 1);
    return true;
}

bool imu_data_ready(void)
{
    return imu_ready || imu_status_gyro_ready();
}

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

void imu_read_accel(IMU_t *imu)
{
    uint8_t buf[6];

    i2c_read(IMU_ADDR, ACCEL_REG, buf, 6);
    imu->ax = (int16_t)(buf[0] | (buf[1] << 8));
    imu->ay = (int16_t)(buf[2] | (buf[3] << 8));
    imu->az = (int16_t)(buf[4] | (buf[5] << 8));

    imu_convert_units(imu);
}

void imu_read_gyro(IMU_t *imu)
{
    imu_read_gyro_raw(&imu->gx, &imu->gy, &imu->gz);
    imu_convert_units(imu);
}

void imu_read_temp(IMU_t *imu)
{
    uint8_t buf[2];

    i2c_read(IMU_ADDR, TEMP_REG, buf, 2);
    imu->temp_raw = (int16_t)(buf[0] | (buf[1] << 8));
}

void imu_read_all(IMU_t *imu)
{
    uint8_t buf[14];

    i2c_read(IMU_ADDR, TEMP_REG, buf, 14);
    imu->temp_raw = (int16_t)(buf[0]  | (buf[1]  << 8));
    imu->gx       = (int16_t)(buf[2]  | (buf[3]  << 8));
    imu->gy       = (int16_t)(buf[4]  | (buf[5]  << 8));
    imu->gz       = (int16_t)(buf[6]  | (buf[7]  << 8));
    imu->ax       = (int16_t)(buf[8]  | (buf[9]  << 8));
    imu->ay       = (int16_t)(buf[10] | (buf[11] << 8));
    imu->az       = (int16_t)(buf[12] | (buf[13] << 8));

    imu_convert_units(imu);
}

/* Subtract stored bias then apply sensitivity scale — integer only. */
static void imu_convert_units(IMU_t *imu)
{
    imu->ax_mg   = (int32_t)imu->ax * ACCEL_SENS_NUM / ACCEL_SENS_DEN;
    imu->ay_mg   = (int32_t)imu->ay * ACCEL_SENS_NUM / ACCEL_SENS_DEN;
    imu->az_mg   = (int32_t)imu->az * ACCEL_SENS_NUM / ACCEL_SENS_DEN;

    int32_t gx_c = (int32_t)imu->gx - imu->gx_bias;
    int32_t gy_c = (int32_t)imu->gy - imu->gy_bias;
    int32_t gz_c = (int32_t)imu->gz - imu->gz_bias;

    imu->gx_mdps = gx_c * GYRO_SENS_NUM / GYRO_SENS_DEN;
    imu->gy_mdps = gy_c * GYRO_SENS_NUM / GYRO_SENS_DEN;
    imu->gz_mdps = gz_c * GYRO_SENS_NUM / GYRO_SENS_DEN;
}

static void imu_read_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    i2c_read(IMU_ADDR, GYRO_REG, buf, 6);
    *gx = (int16_t)(buf[0] | (buf[1] << 8));
    *gy = (int16_t)(buf[2] | (buf[3] << 8));
    *gz = (int16_t)(buf[4] | (buf[5] << 8));
}

static bool imu_wait_for_ready(uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();

    while (!imu_ready && !imu_status_gyro_ready())
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            return false;
        }
    }
    imu_ready = 0;
    return true;
}

static bool imu_status_gyro_ready(void)
{
    uint8_t status = 0;

    i2c_read(IMU_ADDR, STATUS_REG, &status, 1);
    return (status & STATUS_GYRO_READY) != 0U;
}

/* ================================================================== */
#else  /* stubs */
/* ================================================================== */

bool imu_init(IMU_t *imu, uint8_t slave_addr)
{
    (void)imu; (void)slave_addr;
    return true;
}
bool imu_calibrate_gyro(IMU_t *imu, uint16_t sample_count)
{
    (void)imu; (void)sample_count;
    return true;
}
bool imu_data_ready(void)       { return true; }
void imu_read(IMU_t *imu)       { (void)imu; }
void imu_read_accel(IMU_t *imu) { (void)imu; }
void imu_read_gyro(IMU_t *imu)  { (void)imu; }
void imu_read_temp(IMU_t *imu)  { (void)imu; }
void imu_read_all(IMU_t *imu)   { (void)imu; }

#endif /* USE_IMU */

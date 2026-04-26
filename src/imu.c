/**
IMU driver

STM LSM6DS3
LSM6DS3 can operate at 100 kHz or 400 kHz.

Data provided:
    - 3 axis acceleration
    - 3 axis angular rate
    - temperature
*/

#include "config.h"
#include "i2c.h"
#include "imu.h"

#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include <stdint.h>

#define IMU_ADDR        0x6B
#define WHO_AM_I_REG    0x0F
#define IMU_ID          0x69

#define ACCEL_REG       0x28
#define GYRO_REG        0x22
#define TEMP_REG        0x20

#define ACCEL_CFG       0x10
#define GYRO_CFG        0x11
#define INT1_CFG        0x0D
#define STATUS_REG      0x1E
#define CTRL3_C         0x12
#define CTRL4_C         0x13

#define ACCEL_ODR_416HZ  (0b0110 << 4)
#define ACCEL_FS_8G      (0b11 << 2)
#define ACCEL_BW_400HZ   (0b00 << 0)

#define GYRO_ODR_416HZ   (0b0110 << 4)
#define GYRO_FS_2000DPS  (0b11 << 2)

#define INT1_GYRO_EN     (0b1 << 1)
#define INT1_ACCEL_EN    (0b1 << 0)
#define STATUS_GYRO_READY (0b1 << 1)
#define CTRL3_BDU        (0b1 << 6)
#define CTRL3_IF_INC     (0b1 << 2)

#define GYRO_CAL_WARMUP_SAMPLES        128U
#define GYRO_CAL_READY_TIMEOUT_MS      100U
#define GYRO_CAL_STABILITY_THRESHOLD_RAW  200

static void imu_convert_units(LSM6DS3_t* imu);
static void imu_read_gyro_raw(int16_t* gx, int16_t* gy, int16_t* gz);
static bool imu_wait_for_ready(uint32_t timeout_ms);
static bool imu_status_gyro_ready(void);

volatile uint8_t imu_ready = 0;
volatile uint8_t imu_error_code = IMU_ERROR_NONE;

#if USE_IMU

bool imu_init(LSM6DS3_t* imu)
{
    uint8_t device_id = 0;
    uint8_t ctrl3 = CTRL3_BDU | CTRL3_IF_INC;
    uint8_t ctrl4 = 0x00;
    uint8_t int1_config = INT1_GYRO_EN | INT1_ACCEL_EN;
    uint8_t accel_config = ACCEL_ODR_416HZ | ACCEL_FS_8G | ACCEL_BW_400HZ;
    uint8_t gyro_config = GYRO_ODR_416HZ | GYRO_FS_2000DPS;
    const uint32_t start = HAL_GetTick();

    do
    {
        HAL_Delay(5);
        i2c_read(IMU_ADDR, WHO_AM_I_REG, &device_id, 1);
    } while ((device_id != IMU_ID) && ((HAL_GetTick() - start) < 100U));

    if (device_id != IMU_ID)
    {
        while (1)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
            HAL_Delay(200);
        }
    }

    imu_error_code = IMU_ERROR_NONE;

    // Keep IF_INC enabled so the 6-byte and 12-byte burst reads used by this
    // driver walk through the gyro/accel output registers correctly.
    i2c_write(IMU_ADDR, CTRL3_C, &ctrl3, 1);
    // Keep the default CTRL4_C state here. Setting bit 3 disables I2C on the
    // LSM6DS3, which prevents the later config writes and calibration reads
    // from working.
    i2c_write(IMU_ADDR, CTRL4_C, &ctrl4, 1);
    i2c_write(IMU_ADDR, INT1_CFG, &int1_config, 1);
    i2c_write(IMU_ADDR, ACCEL_CFG, &accel_config, 1);
    i2c_write(IMU_ADDR, GYRO_CFG, &gyro_config, 1);

    imu_reset_gyro_bias(imu);
    imu->ax = 0;
    imu->ay = 0;
    imu->az = 0;
    imu->gx = 0;
    imu->gy = 0;
    imu->gz = 0;
    imu->temp = 0;
    imu_convert_units(imu);

    return true;
}

bool imu_data_ready(void)
{
    return (imu_ready != 0U) || imu_status_gyro_ready();
}

void imu_reset_gyro_bias(LSM6DS3_t* imu)
{
    imu->gx_bias = 0;
    imu->gy_bias = 0;
    imu->gz_bias = 0;
    imu->gyro_bias_valid = false;
}

bool imu_calibrate_gyro(LSM6DS3_t* imu, uint16_t sample_count)
{
    uint16_t sample_index;
    int32_t gx_sum = 0;
    int32_t gy_sum = 0;
    int32_t gz_sum = 0;
    int16_t gx_min = INT16_MAX;
    int16_t gy_min = INT16_MAX;
    int16_t gz_min = INT16_MAX;
    int16_t gx_max = INT16_MIN;
    int16_t gy_max = INT16_MIN;
    int16_t gz_max = INT16_MIN;
    uint8_t cal_int1_config = INT1_GYRO_EN;
    uint8_t runtime_int1_config = INT1_GYRO_EN | INT1_ACCEL_EN;
    bool calibration_ok = false;

    if (sample_count == 0U)
    {
        return false;
    }

    imu_error_code = IMU_ERROR_NONE;
    imu_reset_gyro_bias(imu);
    imu_ready = 0;

    // During calibration, only gate off the gyro data-ready signal. If accel
    // DRDY is also enabled here, INT1 can remain latched high after the first
    // sample because this routine only reads gyro registers.
    i2c_write(IMU_ADDR, INT1_CFG, &cal_int1_config, 1);

    for (sample_index = 0; sample_index < GYRO_CAL_WARMUP_SAMPLES; ++sample_index)
    {
        int16_t gx_raw;
        int16_t gy_raw;
        int16_t gz_raw;

        if (!imu_wait_for_ready(GYRO_CAL_READY_TIMEOUT_MS))
        {
            imu_error_code = IMU_ERROR_CAL_TIMEOUT;
            goto restore_int1_config;
        }

        imu_read_gyro_raw(&gx_raw, &gy_raw, &gz_raw);
    }

    for (sample_index = 0; sample_index < sample_count; ++sample_index)
    {
        int16_t gx_raw;
        int16_t gy_raw;
        int16_t gz_raw;

        if (!imu_wait_for_ready(GYRO_CAL_READY_TIMEOUT_MS))
        {
            imu_error_code = IMU_ERROR_CAL_TIMEOUT;
            goto restore_int1_config;
        }

        imu_read_gyro_raw(&gx_raw, &gy_raw, &gz_raw);

        gx_sum += gx_raw;
        gy_sum += gy_raw;
        gz_sum += gz_raw;

        if (gx_raw < gx_min) gx_min = gx_raw;
        if (gy_raw < gy_min) gy_min = gy_raw;
        if (gz_raw < gz_min) gz_min = gz_raw;
        if (gx_raw > gx_max) gx_max = gx_raw;
        if (gy_raw > gy_max) gy_max = gy_raw;
        if (gz_raw > gz_max) gz_max = gz_raw;
    }

    // Gyro bias can legitimately be non-zero, especially after soldering the
    // IMU onto a frame. Reject only if the samples are changing too much over
    // the capture window, which is a stronger sign of actual motion.
    if (((int32_t)gx_max - (int32_t)gx_min) > GYRO_CAL_STABILITY_THRESHOLD_RAW ||
        ((int32_t)gy_max - (int32_t)gy_min) > GYRO_CAL_STABILITY_THRESHOLD_RAW ||
        ((int32_t)gz_max - (int32_t)gz_min) > GYRO_CAL_STABILITY_THRESHOLD_RAW)
    {
        imu_error_code = IMU_ERROR_CAL_MOTION;
        imu_reset_gyro_bias(imu);
        goto restore_int1_config;
    }

    imu->gx_bias = (int16_t)(gx_sum / (int32_t)sample_count);
    imu->gy_bias = (int16_t)(gy_sum / (int32_t)sample_count);
    imu->gz_bias = (int16_t)(gz_sum / (int32_t)sample_count);
    imu->gyro_bias_valid = true;
    imu_error_code = IMU_ERROR_NONE;
    calibration_ok = true;

restore_int1_config:
    imu_ready = 0;
    i2c_write(IMU_ADDR, INT1_CFG, &runtime_int1_config, 1);

    return calibration_ok;
}

void imu_read(LSM6DS3_t* imu)
{
    uint8_t buf[12];

    i2c_read(IMU_ADDR, GYRO_REG, buf, 12);
    imu->gx = (int16_t)(buf[0] | (buf[1] << 8));
    imu->gy = (int16_t)(buf[2] | (buf[3] << 8));
    imu->gz = (int16_t)(buf[4] | (buf[5] << 8));
    imu->ax = (int16_t)(buf[6] | (buf[7] << 8));
    imu->ay = (int16_t)(buf[8] | (buf[9] << 8));
    imu->az = (int16_t)(buf[10] | (buf[11] << 8));

    imu_convert_units(imu);
}

void imu_read_accel(LSM6DS3_t* imu)
{
    uint8_t buf[6];

    i2c_read(IMU_ADDR, ACCEL_REG, buf, 6);
    imu->ax = (int16_t)(buf[0] | (buf[1] << 8));
    imu->ay = (int16_t)(buf[2] | (buf[3] << 8));
    imu->az = (int16_t)(buf[4] | (buf[5] << 8));

    imu_convert_units(imu);
}

void imu_read_gyro(LSM6DS3_t* imu)
{
    imu_read_gyro_raw(&imu->gx, &imu->gy, &imu->gz);
    imu_convert_units(imu);
}

void imu_read_temp(LSM6DS3_t* imu)
{
    uint8_t buf[2];

    i2c_read(IMU_ADDR, TEMP_REG, buf, 2);
    imu->temp = (int16_t)(buf[0] | (buf[1] << 8));
    imu_convert_units(imu);
}

void imu_read_all(LSM6DS3_t* imu)
{
    uint8_t buf[14];

    i2c_read(IMU_ADDR, TEMP_REG, buf, 14);
    imu->temp = (int16_t)(buf[0] | (buf[1] << 8));
    imu->gx = (int16_t)(buf[2] | (buf[3] << 8));
    imu->gy = (int16_t)(buf[4] | (buf[5] << 8));
    imu->gz = (int16_t)(buf[6] | (buf[7] << 8));
    imu->ax = (int16_t)(buf[8] | (buf[9] << 8));
    imu->ay = (int16_t)(buf[10] | (buf[11] << 8));
    imu->az = (int16_t)(buf[12] | (buf[13] << 8));

    imu_convert_units(imu);
}

static void imu_convert_units(LSM6DS3_t* imu)
{
    const int32_t gx_corrected = (int32_t)imu->gx - (int32_t)imu->gx_bias;
    const int32_t gy_corrected = (int32_t)imu->gy - (int32_t)imu->gy_bias;
    const int32_t gz_corrected = (int32_t)imu->gz - (int32_t)imu->gz_bias;

    imu->ax_mg = (int32_t)imu->ax * 244 / 1000;
    imu->ay_mg = (int32_t)imu->ay * 244 / 1000;
    imu->az_mg = (int32_t)imu->az * 244 / 1000;

    imu->gx_mdps = gx_corrected * 70 / 1000;
    imu->gy_mdps = gy_corrected * 70 / 1000;
    imu->gz_mdps = gz_corrected * 70 / 1000;

    imu->ax_g = (float)imu->ax_mg / 1000.0f;
    imu->ay_g = (float)imu->ay_mg / 1000.0f;
    imu->az_g = (float)imu->az_mg / 1000.0f;

    imu->gx_dps = (float)imu->gx_mdps / 1000.0f;
    imu->gy_dps = (float)imu->gy_mdps / 1000.0f;
    imu->gz_dps = (float)imu->gz_mdps / 1000.0f;
}

static void imu_read_gyro_raw(int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t buf[6];

    i2c_read(IMU_ADDR, GYRO_REG, buf, 6);
    *gx = (int16_t)(buf[0] | (buf[1] << 8));
    *gy = (int16_t)(buf[2] | (buf[3] << 8));
    *gz = (int16_t)(buf[4] | (buf[5] << 8));
}

static bool imu_wait_for_ready(uint32_t timeout_ms)
{
    const uint32_t start_ms = HAL_GetTick();

    while (!imu_data_ready())
    {
        if ((HAL_GetTick() - start_ms) > timeout_ms)
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

#else

bool imu_init(LSM6DS3_t* imu) { return (imu != 0); }
bool imu_data_ready(void) { return false; }
void imu_reset_gyro_bias(LSM6DS3_t* imu) { (void)imu; }
bool imu_calibrate_gyro(LSM6DS3_t* imu, uint16_t sample_count)
{
    (void)imu;
    (void)sample_count;
    return true;
}
void imu_read(LSM6DS3_t* imu) { (void)imu; }
void imu_read_accel(LSM6DS3_t* imu) { (void)imu; }
void imu_read_gyro(LSM6DS3_t* imu) { (void)imu; }
void imu_read_temp(LSM6DS3_t* imu) { (void)imu; }
void imu_read_all(LSM6DS3_t* imu) { (void)imu; }

#endif

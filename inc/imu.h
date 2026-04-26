#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

/* Calibration error codes reported via imu_error_code */
#define IMU_ERROR_NONE          0U
#define IMU_ERROR_CAL_TIMEOUT   1U  /* gyro not ready within timeout */
#define IMU_ERROR_CAL_MOTION    2U  /* too much motion during calibration */

typedef struct {
    uint8_t slave_addr;

    /* Raw register outputs */
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp_raw;

    /* Physical units — integer only, no FPU needed
     * ax_mg / ay_mg / az_mg : milli-g        (1000 = 1 g)
     * gx_mdps / gy_mdps / gz_mdps : milli-dps (1000 = 1 deg/s)
     * Gyro bias is subtracted before scaling.
     */
    int32_t ax_mg, ay_mg, az_mg;
    int32_t gx_mdps, gy_mdps, gz_mdps;

    /* Per-axis gyro zero-rate offset (raw counts, measured at boot) */
    int16_t gx_bias, gy_bias, gz_bias;
} IMU_t;

extern volatile uint8_t imu_ready;
extern volatile uint8_t imu_error_code;

bool imu_init(IMU_t *imu, uint8_t slave_addr);
bool imu_calibrate_gyro(IMU_t *imu, uint16_t sample_count);
bool imu_data_ready(void);
void imu_read(IMU_t *imu);
void imu_read_accel(IMU_t *imu);
void imu_read_gyro(IMU_t *imu);
void imu_read_temp(IMU_t *imu);
void imu_read_all(IMU_t *imu);

#endif /* IMU_H */

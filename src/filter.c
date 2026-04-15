#include "filter.h"
#include <math.h>

#define RAD_TO_DEG 57.2957795f

void filter_init(Attitude_t* att)
{
    att->roll_deg = 0.0f;
    att->pitch_deg = 0.0f;

    att->gyro_x_dps = 0.0f;
    att->gyro_y_dps = 0.0f;
    att->gyro_z_dps = 0.0f;

    att->accel_x_g = 0.0f;
    att->accel_y_g = 0.0f;
    att->accel_z_g = 0.0f;
}

void filter_update(Attitude_t* att,
                   float gx_dps,
                   float gy_dps,
                   float gz_dps,
                   float ax_g,
                   float ay_g,
                   float az_g,
                   float dt)
{
    const float alpha = 0.98f;

    float roll_acc_deg;
    float pitch_acc_deg;

    att->gyro_x_dps = gx_dps;
    att->gyro_y_dps = gy_dps;
    att->gyro_z_dps = gz_dps;

    att->accel_x_g = ax_g;
    att->accel_y_g = ay_g;
    att->accel_z_g = az_g;

    roll_acc_deg =
        atan2f(ay_g, az_g) * RAD_TO_DEG;

    pitch_acc_deg =
        atan2f(-ax_g, sqrtf((ay_g * ay_g) + (az_g * az_g))) * RAD_TO_DEG;

    att->roll_deg =
        alpha * (att->roll_deg + gx_dps * dt) +
        (1.0f - alpha) * roll_acc_deg;

    att->pitch_deg =
        alpha * (att->pitch_deg + gy_dps * dt) +
        (1.0f - alpha) * pitch_acc_deg;
}
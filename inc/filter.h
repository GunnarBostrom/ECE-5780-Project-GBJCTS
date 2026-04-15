#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

typedef struct
{
    float roll_deg;
    float pitch_deg;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
} Attitude_t;

void filter_init(Attitude_t* att);
void filter_update(Attitude_t* att,
                   float gx_dps,
                   float gy_dps,
                   float gz_dps,
                   float ax_g,
                   float ay_g,
                   float az_g,
                   float dt);

#endif
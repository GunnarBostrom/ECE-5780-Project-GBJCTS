#include "filter.h"
#include <math.h>

// Conversion factor from radians to degrees
#define RAD_TO_DEG 57.2957795f

// Initialize attitude estimate structure

// Starts roll and pitch at zero and clears the stored gyro/accelerometer values

void filter_init(Attitude_t* att)
{
    // Estimated Euler angles
    att->roll_deg = 0.0f;
    att->pitch_deg = 0.0f;

    // Latest gyro measurements
    att->gyro_x_dps = 0.0f;
    att->gyro_y_dps = 0.0f;
    att->gyro_z_dps = 0.0f;

    // Latest accelerometer measurements
    att->accel_x_g = 0.0f;
    att->accel_y_g = 0.0f;
    att->accel_z_g = 0.0f;
}

// Update roll and pitch estimate using a complementary filter
//
// Inputs:
//   gx_dps, gy_dps, gz_dps -> gyro angular rates in deg/s
//   ax_g, ay_g, az_g       -> accelerometer readings in g
//   dt                     -> loop timestep in seconds
//
// Function:
//   - Stores the latest raw IMU data in the attitude structure
//   - Computes roll/pitch from the accelerometer
//   - Integrates gyro rates to predict angle motion
//   - Blends gyro + accel estimates into a stable attitude estimate

void filter_update(Attitude_t* att,
                   float gx_dps,
                   float gy_dps,
                   float gz_dps,
                   float ax_g,
                   float ay_g,
                   float az_g,
                   float dt)
{
    // Complementary filter blending factor
    //
    // alpha close to 1.0:
    //   trust gyro more (fast response, but drifts over time)
    //
    // (1 - alpha):
    //   trust accelerometer correction more (stable long-term, but noisy)
    const float alpha = 0.98f;

    float roll_acc_deg;
    float pitch_acc_deg;

    // Save most recent gyro measurements
    att->gyro_x_dps = gx_dps;
    att->gyro_y_dps = gy_dps;
    att->gyro_z_dps = gz_dps;

    // Save most recent accelerometer measurements
    att->accel_x_g = ax_g;
    att->accel_y_g = ay_g;
    att->accel_z_g = az_g;

    
    // Compute roll from accelerometer
    //
    // Uses gravity direction projected onto Y and Z axes
    // Result is an absolute roll estimate in degrees
    
    roll_acc_deg =
        atan2f(ay_g, az_g) * RAD_TO_DEG;

    
    // Compute pitch from accelerometer
    //
    // Uses X axis and the magnitude of the Y/Z gravity components
    // Result is an absolute pitch estimate in degrees
    
    pitch_acc_deg =
        atan2f(-ax_g, sqrtf((ay_g * ay_g) + (az_g * az_g))) * RAD_TO_DEG;

    
    // Complementary filter for roll
    //
    // 1. Predict new roll using gyro integration
    // 2. Correct slow drift using accelerometer-based roll angle
    
    att->roll_deg =
        alpha * (att->roll_deg + gx_dps * dt) +
        (1.0f - alpha) * roll_acc_deg;

    
    // Complementary filter for pitch
    //
    // 1. Predict new pitch using gyro integration
    // 2. Correct slow drift using accelerometer-based pitch angle
    
    att->pitch_deg =
        alpha * (att->pitch_deg + gy_dps * dt) +
        (1.0f - alpha) * pitch_acc_deg;
}
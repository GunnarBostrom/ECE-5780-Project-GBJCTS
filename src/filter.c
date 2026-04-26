#include "filter.h"

/* ------------------------------------------------------------------ */
/* IMU mounting-tilt bias (measured statically on a level surface).
 * These values are subtracted from the accelerometer angle reference so
 * the filter converges to 0° when the drone is physically level.
 *
 * ROLL_BIAS_CDEG  = 100  : drone reads +1.00° roll  when level
 * PITCH_BIAS_CDEG = -60  : drone reads -0.60° pitch when level
 */
#define ROLL_BIAS_CDEG   100
#define PITCH_BIAS_CDEG  (-60)

/* ------------------------------------------------------------------ */
/* Gyro integration constant.
 * delta_cdeg = gx_mdps * (1/416 s) * (100 cdeg/deg) / (1000 mdps/dps)
 *            = gx_mdps / 4160
 * A Bresenham carry accumulates the sub-centidegree remainder so no
 * angular-rate information is discarded at 416 Hz.                    */
#define GYRO_INTEG_DENOM 4160

/* ------------------------------------------------------------------ */
/* Accelerometer angle constant.
 * Small-angle approximation: atan2(y, z) ≈ y/z [rad]
 * 1 radian = 57.2958° = 5729.58 centidegrees ≈ 5730 cdeg
 * Guard: skip accel correction when az_mg < AZ_MIN_MG (craft near 90°). */
#define RAD_TO_CDEG 5730
#define AZ_MIN_MG    200

/* ------------------------------------------------------------------ */
/* Complementary filter blend: alpha = 49/50 ≈ 0.98
 * roll_new = (49 * gyro_pred + 1 * accel_ref) / 50               */
#define ALPHA_NUM 49
#define ALPHA_DEN 50

void filter_init(Attitude_t *att)
{
    att->roll_cdeg   = 0;
    att->pitch_cdeg  = 0;
    att->roll_carry  = 0;
    att->pitch_carry = 0;
}

void filter_update(Attitude_t *att,
                   int32_t gx_mdps, int32_t gy_mdps,
                   int32_t ax_mg,   int32_t ay_mg, int32_t az_mg)
{
    /* -- Gyro integration (Bresenham) -------------------------------- */
    att->roll_carry  += gx_mdps;
    att->pitch_carry += gy_mdps;

    int32_t delta_roll  = att->roll_carry  / GYRO_INTEG_DENOM;
    int32_t delta_pitch = att->pitch_carry / GYRO_INTEG_DENOM;

    att->roll_carry  %= GYRO_INTEG_DENOM;
    att->pitch_carry %= GYRO_INTEG_DENOM;

    int32_t roll_pred  = att->roll_cdeg  + delta_roll;
    int32_t pitch_pred = att->pitch_cdeg + delta_pitch;

    /* -- Accelerometer angle reference (small-angle approx) ---------- */
    int32_t roll_acc;
    int32_t pitch_acc;

    if (az_mg > AZ_MIN_MG)
    {
        /* roll  ≈  atan2(ay, az) in cdeg, corrected for mounting bias */
        roll_acc  =  ay_mg * RAD_TO_CDEG / az_mg - ROLL_BIAS_CDEG;
        /* pitch ≈  atan2(-ax, az) in cdeg, corrected for mounting bias */
        pitch_acc = -ax_mg * RAD_TO_CDEG / az_mg - PITCH_BIAS_CDEG;
    }
    else
    {
        /* Near 90° bank — trust gyro only, no accel correction */
        roll_acc  = roll_pred;
        pitch_acc = pitch_pred;
    }

    /* -- Complementary filter ---------------------------------------- */
    att->roll_cdeg  = (ALPHA_NUM * roll_pred  + roll_acc)  / ALPHA_DEN;
    att->pitch_cdeg = (ALPHA_NUM * pitch_pred + pitch_acc) / ALPHA_DEN;
}

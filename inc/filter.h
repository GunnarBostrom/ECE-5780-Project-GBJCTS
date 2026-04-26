#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

/* Attitude estimate in centidegrees (1 unit = 0.01°).
 * roll_carry / pitch_carry are Bresenham accumulators for the gyro
 * integration — they hold the sub-centidegree remainder between steps so
 * no angular rate information is lost to truncation at 416 Hz. */
typedef struct {
    int32_t roll_cdeg;
    int32_t pitch_cdeg;
    int32_t roll_carry;   /* mdps units, range (-4160, 4160) */
    int32_t pitch_carry;
} Attitude_t;

void filter_init(Attitude_t *att);

/* Update roll/pitch estimate.
 * gx_mdps / gy_mdps : gyro rates in milli-deg/s (bias already removed).
 * ax_mg / ay_mg / az_mg : accel in milli-g.
 * dt is fixed at 1/416 s — no argument needed. */
void filter_update(Attitude_t *att,
                   int32_t gx_mdps, int32_t gy_mdps,
                   int32_t ax_mg,   int32_t ay_mg, int32_t az_mg);

#endif /* FILTER_H */

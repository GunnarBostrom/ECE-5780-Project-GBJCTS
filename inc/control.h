#ifndef CONTROL_H
#define CONTROL_H

#include "imu.h"
#include "filter.h"

/* Initialise the attitude PID controllers.
 * dt is fixed at 1/416 s — all coefficients are precomputed. */
void control_init(void);

/* Run one attitude-control cycle.
 * Reads radio state internally.  `imu` is reserved for future use.
 * `attitude` provides roll_cdeg / pitch_cdeg in centidegrees. */
void control_update(const IMU_t *imu, const Attitude_t *attitude);

#endif /* CONTROL_H */

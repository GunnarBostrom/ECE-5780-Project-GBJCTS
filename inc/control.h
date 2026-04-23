#ifndef CONTROL_H
#define CONTROL_H

#include "imu.h"
#include "filter.h"

// Initializes the attitude controller with the fixed control-loop timestep.
//
// The timestep should match the rate at which `control_update()` is called so
// the PID controllers use the correct integration and differentiation interval.
void control_init(float dt);

// Runs one attitude-control update using the latest estimated attitude.
//
// The controller reads the current radio state internally, converts throttle
// into a motor baseline, applies roll/pitch stabilization, and sends the mixed
// motor commands to the ESC outputs.
//
// `imu` is accepted for interface consistency and future extensions, but the
// current implementation closes the loop using `attitude`.
void control_update(const IMU_t* imu, const Attitude_t* attitude);

#endif

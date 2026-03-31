#ifndef PID_H
#define PID_H

// PID controller data structure.
//
// This structure stores both the gain values and the internal controller state.
// The internal state includes the integral accumulator, the previous error,
// and the filtered derivative state.

typedef struct
{
    // Proportional gain.
    float kp;

    // Integral gain.
    float ki;

    // Derivative gain.
    float kd;

    // Controller sample period in seconds.
    float dt;

    // Running integral accumulator.
    float integrator;

    // Previous cycle's error value.
    float prev_error;

    // Filtered derivative term state.
    float differentiator;

    // Minimum and maximum allowed controller output.
    float out_min;
    float out_max;

    // Minimum and maximum allowed integrator value.
    //
    // This provides a basic anti-windup mechanism so the integrator
    // does not grow without bound when the actuator saturates.
    float integ_min;
    float integ_max;

    // Derivative low-pass filter time constant.
    //
    // A nonzero value helps prevent the derivative term from becoming
    // overly sensitive to noise or sharp measurement changes.
    float deriv_tau;

} PIDController;

// Initializes the PID controller with gains, timing, and saturation limits.
void PID_Init(PIDController *pid,
              float kp,
              float ki,
              float kd,
              float dt,
              float out_min,
              float out_max,
              float integ_min,
              float integ_max,
              float deriv_tau);

// Resets the internal dynamic state of the controller.
void PID_Reset(PIDController *pid);

// Updates the PID controller for one sample period.
//
// setpoint   = desired target value
// measurement = measured plant output
//
// Returns the saturated controller output.
float PID_Update(PIDController *pid, float setpoint, float measurement);

#endif
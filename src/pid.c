#include "pid.h"

// Helper function to clamp a floating-point value into a valid range.
//
// This is used for both controller output saturation and integrator
// anti-windup limiting.
static float clampf(float value, float min_val, float max_val)
{
    if (value < min_val)
    {
        return min_val;
    }

    if (value > max_val)
    {
        return max_val;
    }

    return value;
}

void PID_Init(PIDController *pid,
              float kp,
              float ki,
              float kd,
              float dt,
              float out_min,
              float out_max,
              float integ_min,
              float integ_max,
              float deriv_tau)
{
    // Store gain values.
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    // Store sample time.
    pid->dt = dt;

    // Initialize internal state to zero so the controller starts cleanly.
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->differentiator = 0.0f;

    // Store output saturation limits.
    pid->out_min = out_min;
    pid->out_max = out_max;

    // Store integrator saturation limits.
    pid->integ_min = integ_min;
    pid->integ_max = integ_max;

    // Store derivative filter time constant.
    pid->deriv_tau = deriv_tau;
}

void PID_Reset(PIDController *pid)
{
    // Clear all dynamic controller state.
    //
    // This does not change gains or limits. It only resets the
    // accumulated terms that depend on runtime history.
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->differentiator = 0.0f;
}

float PID_Update(PIDController *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;
    float proportional = pid->kp * error;

    pid->integrator += 0.5f * pid->ki * pid->dt * (error + pid->prev_error);
    pid->integrator = clampf(pid->integrator, pid->integ_min, pid->integ_max);

    float tau = pid->deriv_tau;
    if (tau < 1.0e-6f)
    {
        tau = 1.0e-6f;
    }

    float error_delta = error - pid->prev_error;

    pid->differentiator =
        ((2.0f * tau - pid->dt) / (2.0f * tau + pid->dt)) * pid->differentiator +
        ((2.0f * pid->kd) / (2.0f * tau + pid->dt)) * error_delta;

    float output = proportional + pid->integrator + pid->differentiator;
    output = clampf(output, pid->out_min, pid->out_max);

    pid->prev_error = error;

    return output;
}

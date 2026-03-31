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
    // Compute the current control error.
    //
    // Positive error means the measured value is below the target.
    float error = setpoint - measurement;

    // -----------------------------
    // Proportional term
    // -----------------------------
    //
    // This provides an immediate correction proportional to the current error.
    float proportional = pid->kp * error;

    // -----------------------------
    // Integral term
    // -----------------------------
    //
    // Use trapezoidal integration for a slightly better discrete approximation:
    //
    // integral += Ki * dt * (e[k] + e[k-1]) / 2
    //
    // Then clamp the integrator to avoid excessive windup.
    pid->integrator += 0.5f * pid->ki * pid->dt * (error + pid->prev_error);
    pid->integrator = clampf(pid->integrator, pid->integ_min, pid->integ_max);

    // -----------------------------
    // Derivative term
    // -----------------------------
    //
    // The raw derivative is based on the error slope. A simple filtered
    // derivative is used rather than a pure finite difference because
    // pure differentiation tends to amplify noise and can make tuning ugly.
    float raw_derivative = (error - pid->prev_error) / pid->dt;

    // First-order filtered derivative update.
    //
    // This discrete-time form is commonly used for a "dirty derivative."
    pid->differentiator =
        ((2.0f * pid->deriv_tau - pid->dt) / (2.0f * pid->deriv_tau + pid->dt)) * pid->differentiator +
        ((2.0f * pid->kd) / (2.0f * pid->deriv_tau + pid->dt)) * raw_derivative;

    // Sum all three terms together.
    float output = proportional + pid->integrator + pid->differentiator;

    // Saturate the final controller output so the actuator demand remains
    // within a realistic range.
    output = clampf(output, pid->out_min, pid->out_max);

    // Store current error for use on the next update cycle.
    pid->prev_error = error;

    return output;
}
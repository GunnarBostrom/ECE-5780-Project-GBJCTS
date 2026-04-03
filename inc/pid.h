#ifndef PID_H_
#define PID_H_

typedef struct
{
    float kp;
    float ki;
    float kd;

    float dt;

    float integrator;
    float prev_error;
    float differentiator;

    float out_min;
    float out_max;

    float integ_min;
    float integ_max;

    float deriv_tau;
} PIDController;

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

void PID_Reset(PIDController *pid);

float PID_Update(PIDController *pid, float setpoint, float measurement);

#endif
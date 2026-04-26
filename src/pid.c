#include "pid.h"

/* ------------------------------------------------------------------ */
/* Integrator Bresenham denominator.
 *
 * The trapezoid-rule increment per step (in cµs) is:
 *   delta = 0.5 × ki × dt × (e + e_prev) × 100   [cµs]
 *         = ki × (e + e_prev) / (416 × 2)          [cµs, cancel 100s]
 *
 * With ki stored as ki_x100:
 *   carry += ki_x100 × (e + e_prev)
 *   integrator_cus += carry / INTEG_DENOM
 *   carry %= INTEG_DENOM
 *
 * INTEG_DENOM = 100 × 416 × 2 / 1 = 83200
 * (the ×100 converts ki_x100 back to ki; the 416×2 is the dt denominator)
 */
#define INTEG_DENOM 83200

static int32_t clamp32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void PID_Init(PIDController *pid,
              int32_t kp_x100, int32_t ki_x100,
              int32_t coeff_d_q12, int32_t coeff_e_x100,
              int32_t out_min_us,  int32_t out_max_us,
              int32_t integ_min_cus, int32_t integ_max_cus)
{
    pid->kp_x100       = kp_x100;
    pid->ki_x100       = ki_x100;
    pid->coeff_d_q12   = coeff_d_q12;
    pid->coeff_e_x100  = coeff_e_x100;
    pid->out_min_us    = out_min_us;
    pid->out_max_us    = out_max_us;
    pid->integ_min_cus = integ_min_cus;
    pid->integ_max_cus = integ_max_cus;

    pid->integrator_cus    = 0;
    pid->integ_carry       = 0;
    pid->prev_error        = 0;
    pid->differentiator_cus = 0;
}

void PID_Reset(PIDController *pid)
{
    pid->integrator_cus    = 0;
    pid->integ_carry       = 0;
    pid->prev_error        = 0;
    pid->differentiator_cus = 0;
}

int32_t PID_Update(PIDController *pid,
                   int32_t setpoint_cdeg, int32_t measurement_cdeg)
{
    int32_t error = setpoint_cdeg - measurement_cdeg;

    /* Proportional [cµs]: kp_x100 × error / 100 */
    int32_t prop_cus = pid->kp_x100 * error / 100;

    /* Integral — Bresenham carry prevents per-step truncation loss.
     * carry unit: ki_x100 × cdeg  →  divided by INTEG_DENOM gives cµs. */
    if (pid->ki_x100 != 0)
    {
        pid->integ_carry += pid->ki_x100 * (error + pid->prev_error);
        pid->integrator_cus += pid->integ_carry / INTEG_DENOM;
        pid->integ_carry    %= INTEG_DENOM;
        pid->integrator_cus  = clamp32(pid->integrator_cus,
                                       pid->integ_min_cus,
                                       pid->integ_max_cus);
    }

    /* Derivative — Tustin bilinear IIR (Q12 coefficient).
     * coeff_d_q12 = (2τ-dt)/(2τ+dt) × 4096 = 3631  for tau=0.02, dt=1/416
     * coeff_e_x100 = 2kd/(2τ+dt) × 100  [cµs/cdeg]
     * diff_new = coeff_d × diff_prev + coeff_e × delta_error            */
    int32_t delta = error - pid->prev_error;
    pid->differentiator_cus =
        pid->coeff_d_q12 * pid->differentiator_cus / 4096
        + pid->coeff_e_x100 * delta / 100;

    pid->prev_error = error;

    int32_t total_cus = prop_cus + pid->integrator_cus + pid->differentiator_cus;

    /* Convert cµs → µs, then clamp */
    int32_t output_us = total_cus / 100;
    return clamp32(output_us, pid->out_min_us, pid->out_max_us);
}

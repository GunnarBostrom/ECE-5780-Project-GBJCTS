#ifndef PID_H
#define PID_H

#include <stdint.h>

/* Fixed-point PID controller for Cortex-M0 (no FPU).
 *
 * Units:
 *   setpoint / measurement : centidegrees (cdeg), 100 = 1°
 *   output                 : microseconds (µs)
 *   internal accumulator   : centi-microseconds (cµs = 0.01 µs)
 *
 * Gains are stored as integer × 100:
 *   kp_x100 = 350  →  kp = 3.5
 *   ki_x100 = 100  →  ki = 1.0  (set to 0 to disable integral)
 *
 * Derivative Tustin coefficients are precomputed for a fixed
 * tau = 0.02 s and dt = 1/416 s:
 *   coeff_d_q12 = 3631   (= 0.8867 × 4096)
 *   coeff_e_x100: per axis (see control.c)
 */
typedef struct {
    int32_t kp_x100;
    int32_t ki_x100;
    int32_t coeff_d_q12;      /* Tustin IIR coefficient, Q12 */
    int32_t coeff_e_x100;     /* derivative error gain × 100 [cµs/cdeg] */

    int32_t integrator_cus;   /* integral accumulator [cµs] */
    int32_t integ_carry;      /* Bresenham sub-step carry */
    int32_t prev_error;       /* [cdeg] */
    int32_t differentiator_cus; /* [cµs] */

    int32_t out_min_us;
    int32_t out_max_us;
    int32_t integ_min_cus;
    int32_t integ_max_cus;
} PIDController;

void    PID_Init(PIDController *pid,
                 int32_t kp_x100, int32_t ki_x100,
                 int32_t coeff_d_q12, int32_t coeff_e_x100,
                 int32_t out_min_us,  int32_t out_max_us,
                 int32_t integ_min_cus, int32_t integ_max_cus);

void    PID_Reset(PIDController *pid);

/* Returns correction in µs. */
int32_t PID_Update(PIDController *pid,
                   int32_t setpoint_cdeg, int32_t measurement_cdeg);

#endif /* PID_H */

#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include <stdint.h>

#define NUM_GAINS 12

typedef struct
{
    float values[NUM_GAINS];
} GainSet;

typedef struct
{
    GainSet lower_bounds;
    GainSet upper_bounds;
    GainSet step_sizes;
    GainSet best_gains;
    float best_cost;
    uint32_t max_iterations;
    float step_tolerance;
} TwiddleOptimizer;

typedef struct
{
    uint8_t enabled[NUM_GAINS];
} GainMask;

// Gain ordering:
// 0  = roll_kp
// 1  = roll_ki
// 2  = roll_kd
// 3  = pitch_kp
// 4  = pitch_ki
// 5  = pitch_kd
// 6  = yaw_kp
// 7  = yaw_ki
// 8  = yaw_kd
// 9  = alt_kp
// 10 = alt_ki
// 11 = alt_kd

void GainSet_Copy(GainSet *dst, const GainSet *src);
void GainSet_Clamp(GainSet *gains, const GainSet *lower, const GainSet *upper);
float GainSet_StepSum(const GainSet *steps, const GainMask *mask);

void GainMask_EnableAll(GainMask *mask);
void GainMask_EnableOnlyAxis(GainMask *mask, uint8_t axis_index);

void Twiddle_Init(TwiddleOptimizer *opt,
                  const GainSet *initial_gains,
                  const GainSet *lower_bounds,
                  const GainSet *upper_bounds,
                  const GainSet *initial_steps,
                  uint32_t max_iterations,
                  float step_tolerance);

void Twiddle_Run(TwiddleOptimizer *opt,
                 const GainMask *mask,
                 float (*evaluate_fn)(const GainSet *gains, int verbose));

#endif
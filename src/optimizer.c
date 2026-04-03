#include <stdio.h>
#include "optimizer.h"

static float clampf(float x, float min_val, float max_val)
{
    if (x < min_val)
    {
        return min_val;
    }

    if (x > max_val)
    {
        return max_val;
    }

    return x;
}

void GainSet_Copy(GainSet *dst, const GainSet *src)
{
    uint32_t i;

    for (i = 0; i < NUM_GAINS; i++)
    {
        dst->values[i] = src->values[i];
    }
}

void GainSet_Clamp(GainSet *gains, const GainSet *lower, const GainSet *upper)
{
    uint32_t i;

    for (i = 0; i < NUM_GAINS; i++)
    {
        gains->values[i] = clampf(gains->values[i],
                                  lower->values[i],
                                  upper->values[i]);
    }
}

float GainSet_StepSum(const GainSet *steps, const GainMask *mask)
{
    uint32_t i;
    float sum = 0.0f;

    for (i = 0; i < NUM_GAINS; i++)
    {
        if (mask->enabled[i])
        {
            sum += steps->values[i];
        }
    }

    return sum;
}

void GainMask_EnableAll(GainMask *mask)
{
    uint32_t i;

    for (i = 0; i < NUM_GAINS; i++)
    {
        mask->enabled[i] = 1;
    }
}

void GainMask_EnableOnlyAxis(GainMask *mask, uint8_t axis_index)
{
    uint32_t i;
    uint32_t start = 3U * axis_index;
    uint32_t end = start + 3U;

    for (i = 0; i < NUM_GAINS; i++)
    {
        mask->enabled[i] = 0;
    }

    for (i = start; i < end && i < NUM_GAINS; i++)
    {
        mask->enabled[i] = 1;
    }
}

void Twiddle_Init(TwiddleOptimizer *opt,
                  const GainSet *initial_gains,
                  const GainSet *lower_bounds,
                  const GainSet *upper_bounds,
                  const GainSet *initial_steps,
                  uint32_t max_iterations,
                  float step_tolerance)
{
    GainSet_Copy(&opt->best_gains, initial_gains);
    GainSet_Copy(&opt->lower_bounds, lower_bounds);
    GainSet_Copy(&opt->upper_bounds, upper_bounds);
    GainSet_Copy(&opt->step_sizes, initial_steps);

    GainSet_Clamp(&opt->best_gains, &opt->lower_bounds, &opt->upper_bounds);

    opt->best_cost = 1.0e30f;
    opt->max_iterations = max_iterations;
    opt->step_tolerance = step_tolerance;
}

void Twiddle_Run(TwiddleOptimizer *opt,
                 const GainMask *mask,
                 float (*evaluate_fn)(const GainSet *gains, int verbose))
{
    uint32_t iter;
    uint32_t i;

    if (evaluate_fn == 0)
    {
        return;
    }

    opt->best_cost = evaluate_fn(&opt->best_gains, 1);

    printf("Initial cost: %.6f\n", opt->best_cost);

    for (iter = 0; iter < opt->max_iterations; iter++)
    {
        float step_sum = GainSet_StepSum(&opt->step_sizes, mask);

        printf("\nIteration %lu, step sum = %.6f, best cost = %.6f\n",
               (unsigned long)iter,
               step_sum,
               opt->best_cost);

        if (step_sum < opt->step_tolerance)
        {
            printf("Stopping because step sum is below tolerance.\n");
            break;
        }

        for (i = 0; i < NUM_GAINS; i++)
        {
            GainSet candidate;
            float trial_cost;

            if (!mask->enabled[i])
            {
                continue;
            }

            GainSet_Copy(&candidate, &opt->best_gains);

            // Try positive perturbation
            candidate.values[i] += opt->step_sizes.values[i];
            GainSet_Clamp(&candidate, &opt->lower_bounds, &opt->upper_bounds);

            trial_cost = evaluate_fn(&candidate, 0);

            if (trial_cost < opt->best_cost)
            {
                opt->best_cost = trial_cost;
                GainSet_Copy(&opt->best_gains, &candidate);
                opt->step_sizes.values[i] *= 1.10f;

                printf("  Gain %lu improved in + direction, cost = %.6f\n",
                       (unsigned long)i,
                       opt->best_cost);
                continue;
            }

            // Try negative perturbation
            GainSet_Copy(&candidate, &opt->best_gains);
            candidate.values[i] -= opt->step_sizes.values[i];
            GainSet_Clamp(&candidate, &opt->lower_bounds, &opt->upper_bounds);

            trial_cost = evaluate_fn(&candidate, 0);

            if (trial_cost < opt->best_cost)
            {
                opt->best_cost = trial_cost;
                GainSet_Copy(&opt->best_gains, &candidate);
                opt->step_sizes.values[i] *= 1.10f;

                printf("  Gain %lu improved in - direction, cost = %.6f\n",
                       (unsigned long)i,
                       opt->best_cost);
            }
            else
            {
                opt->step_sizes.values[i] *= 0.50f;

                printf("  Gain %lu did not improve, shrinking step to %.6f\n",
                       (unsigned long)i,
                       opt->step_sizes.values[i]);
            }
        }
    }

    printf("\nOptimization finished.\n");
    printf("Best cost: %.6f\n", opt->best_cost);
    printf("Best gains:\n");

    for (i = 0; i < NUM_GAINS; i++)
    {
        printf("  g[%lu] = %.6f\n",
               (unsigned long)i,
               opt->best_gains.values[i]);
    }
}
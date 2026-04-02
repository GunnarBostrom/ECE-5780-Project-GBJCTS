/******************************************************************************
 * File: main_opt.c
 * Author: Josh Canada
 * Course: ECE 5780
 *
 * Description:
 * This file runs an automated PID tuning routine for a 4-axis quadrotor
 * simulation (roll, pitch, yaw, altitude) using a Twiddle-based iterative
 * optimization algorithm.
 *
 * The optimizer evaluates controller performance by:
 *   - Running a full closed-loop simulation using:
 *         PID_Update()  +  Plant_Update()
 *   - Applying time-varying reference commands (step inputs)
 *   - Computing a scalar cost based on:
 *         tracking error
 *         control effort
 *         oscillation
 *         cross-axis coupling
 *         actuator saturation
 *
 * The optimizer iteratively adjusts 12 gains:
 *   [Kp, Ki, Kd] for each axis:
 *       roll, pitch, yaw, altitude
 *
 * ---------------------------------------------------------------------------
 * HOW TO BUILD
 * ---------------------------------------------------------------------------
 *
 * From the project root directory:
 *
 *   gcc -Iinc \
 *       src/main_opt.c \
 *       src/optimizer.c \
 *       src/sim_eval.c \
 *       src/pid.c \
 *       src/plant.c \
 *       -lm -o quad_tune
 *
 * ---------------------------------------------------------------------------
 * HOW TO RUN
 * ---------------------------------------------------------------------------
 *
 *   ./quad_tune
 *
 * The program will:
 *   1. Initialize PID gains and step sizes
 *   2. Run iterative optimization (Twiddle)
 *   3. Print progress to the terminal
 *   4. Output the best gain set found
 *
 * ---------------------------------------------------------------------------
 * OUTPUT
 * ---------------------------------------------------------------------------
 *
 * During execution:
 *   - Iteration number
 *   - Current cost
 *   - Best cost
 *   - Gain updates
 *
 * Final output:
 *   - Optimized gains for all 4 axes
 *
 * ---------------------------------------------------------------------------
 * NOTES
 * ---------------------------------------------------------------------------
 *
 * - Simulation runs fully in C (no MATLAB required)
 * - Gains are tuned against the internal plant model (plant.c)
 * - Performance depends on accuracy of plant parameters
 *
 * Recommended workflow:
 *   1. Run optimizer
 *   2. Validate gains in your step-response simulation
 *   3. Adjust cost weights or bounds if behavior is undesirable
 *   4. Re-run optimization
 *
 * ---------------------------------------------------------------------------
 * FUTURE IMPROVEMENTS
 * ---------------------------------------------------------------------------
 *
 * - Add noise/disturbance injection
 * - Add actuator delay or ESC dynamics
 * - Expand cost function (e.g., rise time, overshoot)
 * - Export best gains to file for embedded deployment
 *
 ******************************************************************************/
 
#include <stdio.h>
#include "optimizer.h"

// You must implement this in your simulation code.
float evaluate_gains(const GainSet *gains, int verbose);

static void print_named_gains(const GainSet *g)
{
    printf("\nNamed gains:\n");
    printf("Roll : Kp=%.6f Ki=%.6f Kd=%.6f\n",  g->values[0],  g->values[1],  g->values[2]);
    printf("Pitch: Kp=%.6f Ki=%.6f Kd=%.6f\n",  g->values[3],  g->values[4],  g->values[5]);
    printf("Yaw  : Kp=%.6f Ki=%.6f Kd=%.6f\n",  g->values[6],  g->values[7],  g->values[8]);
    printf("Alt  : Kp=%.6f Ki=%.6f Kd=%.6f\n",  g->values[9],  g->values[10], g->values[11]);
}

int main(void)
{
    TwiddleOptimizer opt;
    GainSet initial_gains;
    GainSet lower_bounds;
    GainSet upper_bounds;
    GainSet initial_steps;
    GainMask mask;

    // Initial gains
    // Replace these with your current best manual values.
    initial_gains.values[0]  = 140.0f;
    initial_gains.values[1]  = 1.0f;
    initial_gains.values[2]  = 0.10f;

    initial_gains.values[3]  = 140.0f;
    initial_gains.values[4]  = 1.0f;
    initial_gains.values[5]  = 0.10f;

    initial_gains.values[6]  = 80.0f;
    initial_gains.values[7]  = 0.5f;
    initial_gains.values[8]  = 0.05f;

    initial_gains.values[9]  = 10.0f;
    initial_gains.values[10] = 1.0f;
    initial_gains.values[11] = 0.20f;

    // Lower bounds
    lower_bounds.values[0]  = 0.0f;
    lower_bounds.values[1]  = 0.0f;
    lower_bounds.values[2]  = 0.0f;
    lower_bounds.values[3]  = 0.0f;
    lower_bounds.values[4]  = 0.0f;
    lower_bounds.values[5]  = 0.0f;
    lower_bounds.values[6]  = 0.0f;
    lower_bounds.values[7]  = 0.0f;
    lower_bounds.values[8]  = 0.0f;
    lower_bounds.values[9]  = 0.0f;
    lower_bounds.values[10] = 0.0f;
    lower_bounds.values[11] = 0.0f;

    // Upper bounds
    upper_bounds.values[0]  = 300.0f;
    upper_bounds.values[1]  = 40.0f;
    upper_bounds.values[2]  = 20.0f;

    upper_bounds.values[3]  = 300.0f;
    upper_bounds.values[4]  = 40.0f;
    upper_bounds.values[5]  = 20.0f;

    upper_bounds.values[6]  = 200.0f;
    upper_bounds.values[7]  = 30.0f;
    upper_bounds.values[8]  = 15.0f;

    upper_bounds.values[9]  = 60.0f;
    upper_bounds.values[10] = 20.0f;
    upper_bounds.values[11] = 10.0f;

    // Initial step sizes
    initial_steps.values[0]  = 10.0f;
    initial_steps.values[1]  = 0.5f;
    initial_steps.values[2]  = 0.05f;

    initial_steps.values[3]  = 10.0f;
    initial_steps.values[4]  = 0.5f;
    initial_steps.values[5]  = 0.05f;

    initial_steps.values[6]  = 8.0f;
    initial_steps.values[7]  = 0.3f;
    initial_steps.values[8]  = 0.03f;

    initial_steps.values[9]  = 2.0f;
    initial_steps.values[10] = 0.2f;
    initial_steps.values[11] = 0.02f;

    Twiddle_Init(&opt,
                 &initial_gains,
                 &lower_bounds,
                 &upper_bounds,
                 &initial_steps,
                 50,
                 0.05f);

    // Stage 1: roll only
    printf("\n===== Tuning roll =====\n");
    GainMask_EnableOnlyAxis(&mask, 0);
    Twiddle_Run(&opt, &mask, evaluate_gains);

    // Stage 2: pitch only
    printf("\n===== Tuning pitch =====\n");
    GainMask_EnableOnlyAxis(&mask, 1);
    Twiddle_Run(&opt, &mask, evaluate_gains);

    // Stage 3: yaw only
    printf("\n===== Tuning yaw =====\n");
    GainMask_EnableOnlyAxis(&mask, 2);
    Twiddle_Run(&opt, &mask, evaluate_gains);

    // Stage 4: altitude only
    printf("\n===== Tuning altitude =====\n");
    GainMask_EnableOnlyAxis(&mask, 3);
    Twiddle_Run(&opt, &mask, evaluate_gains);

    // Stage 5: joint refinement
    printf("\n===== Joint refinement =====\n");
    GainMask_EnableAll(&mask);
    Twiddle_Run(&opt, &mask, evaluate_gains);

    print_named_gains(&opt.best_gains);

    return 0;
}
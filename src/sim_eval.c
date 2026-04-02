// sim_eval.c
// Josh Canada
// 4-axis gain evaluation for autotuning

#include <stdio.h>
#include <math.h>
#include "optimizer.h"
#include "pid.h"
#include "plant.h"

static float absf_local(float x)
{
    if (x < 0.0f)
    {
        return -x;
    }

    return x;
}

static float clampf_local(float x, float min_val, float max_val)
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

static void build_motor_commands(float throttle_cmd,
                                 float roll_cmd,
                                 float pitch_cmd,
                                 float yaw_cmd,
                                 float motor_cmd[4])
{
    // Motor layout from your plant.c:
    //
    //   Front
    //    0   1
    //    3   2
    //
    // Roll torque:
    //   right side minus left side = (1 + 2) - (0 + 3)
    //
    // Pitch torque:
    //   rear minus front = (2 + 3) - (0 + 1)
    //
    // Yaw torque:
    //   0,2 are CCW (+), 1,3 are CW (-)

    motor_cmd[0] = throttle_cmd - roll_cmd - pitch_cmd + yaw_cmd;
    motor_cmd[1] = throttle_cmd + roll_cmd - pitch_cmd - yaw_cmd;
    motor_cmd[2] = throttle_cmd + roll_cmd + pitch_cmd + yaw_cmd;
    motor_cmd[3] = throttle_cmd - roll_cmd + pitch_cmd - yaw_cmd;

    for (int i = 0; i < 4; i++)
    {
        motor_cmd[i] = clampf_local(motor_cmd[i], 0.0f, 2000.0f);
    }
}

float evaluate_gains(const GainSet *gains, int verbose)
{
    const float dt = 0.002f;
    const float sim_time = 10.0f;
    const int steps = (int)(sim_time / dt);

    QuadPlant plant;

    PIDController roll_pid;
    PIDController pitch_pid;
    PIDController yaw_pid;
    PIDController alt_pid;

    float motor_cmd[4];

    float roll_ref = 0.0f;
    float pitch_ref = 0.0f;
    float yaw_ref = 0.0f;
    float alt_ref = 0.0f;

    float prev_roll_err = 0.0f;
    float prev_pitch_err = 0.0f;
    float prev_yaw_err = 0.0f;
    float prev_alt_err = 0.0f;

    float tracking_cost = 0.0f;
    float effort_cost = 0.0f;
    float oscillation_cost = 0.0f;
    float coupling_cost = 0.0f;
    float saturation_cost = 0.0f;
    float terminal_cost = 0.0f;
    float gain_balance_cost = 0.0f;

    // Plant parameters
    const float J_roll = 0.020f;
    const float J_pitch = 0.020f;
    const float J_yaw = 0.035f;

    const float b_roll = 0.080f;
    const float b_pitch = 0.080f;
    const float b_yaw = 0.100f;

    const float mass = 1.20f;
    const float b_altitude = 0.35f;

    const float motor_tau = 0.050f;
    const float k_thrust = 3.5e-6f;
    const float k_yaw = 4.0e-8f;
    const float arm_length = 0.18f;

    const float w_hover =
        sqrtf((mass * 9.81f) / (4.0f * k_thrust));

    for (int i = 0; i < NUM_GAINS; i++)
    {
        if (!isfinite(gains->values[i]) || (gains->values[i] < 0.0f))
        {
            return 1.0e9f;
        }
    }

    // Prevent obviously bad integral-heavy solutions
    if (gains->values[1] > 10.0f ||
        gains->values[4] > 10.0f ||
        gains->values[7] > 10.0f ||
        gains->values[10] > 20.0f)
    {
        return 1.0e6f;
    }

    Plant_Init(&plant,
               J_roll,
               J_pitch,
               J_yaw,
               b_roll,
               b_pitch,
               b_yaw,
               mass,
               b_altitude,
               motor_tau,
               k_thrust,
               k_yaw,
               arm_length);

    PID_Init(&roll_pid,
             gains->values[0],
             gains->values[1],
             gains->values[2],
             dt,
             -250.0f,
             250.0f,
             -80.0f,
             80.0f,
             0.02f);

    PID_Init(&pitch_pid,
             gains->values[3],
             gains->values[4],
             gains->values[5],
             dt,
             -250.0f,
             250.0f,
             -80.0f,
             80.0f,
             0.02f);

    PID_Init(&yaw_pid,
             gains->values[6],
             gains->values[7],
             gains->values[8],
             dt,
             -120.0f,
             120.0f,
             -40.0f,
             40.0f,
             0.02f);

    PID_Init(&alt_pid,
             gains->values[9],
             gains->values[10],
             gains->values[11],
             dt,
             -400.0f,
             400.0f,
             -150.0f,
             150.0f,
             0.05f);

    for (int k = 0; k < steps; k++)
    {
        const float t = k * dt;

        float roll = plant.roll.angle;
        float pitch = plant.pitch.angle;
        float yaw = plant.yaw.angle;
        float alt = plant.altitude.z;

        float roll_cmd;
        float pitch_cmd;
        float yaw_cmd;
        float alt_cmd;
        float throttle_cmd;

        float roll_err;
        float pitch_err;
        float yaw_err;
        float alt_err;

        // Command schedule
        roll_ref = 0.0f;
        pitch_ref = 0.0f;
        yaw_ref = 0.0f;
        alt_ref = 0.0f;

        if ((t >= 0.5f) && (t < 2.0f))
        {
            roll_ref = 0.15f;
        }
        else if ((t >= 2.0f) && (t < 4.0f))
        {
            pitch_ref = 0.15f;
        }
        else if ((t >= 4.0f) && (t < 6.0f))
        {
            yaw_ref = 0.20f;
        }
        else if ((t >= 6.0f) && (t < 8.0f))
        {
            alt_ref = 0.50f;
        }
        else if (t >= 8.0f)
        {
            roll_ref = 0.10f;
            pitch_ref = -0.10f;
            yaw_ref = 0.15f;
            alt_ref = 0.40f;
        }

        roll_err = roll_ref - roll;
        pitch_err = pitch_ref - pitch;
        yaw_err = yaw_ref - yaw;
        alt_err = alt_ref - alt;

        roll_cmd = PID_Update(&roll_pid, roll_ref, roll);
        pitch_cmd = PID_Update(&pitch_pid, pitch_ref, pitch);
        yaw_cmd = PID_Update(&yaw_pid, yaw_ref, yaw);
        alt_cmd = PID_Update(&alt_pid, alt_ref, alt);

        throttle_cmd = w_hover + alt_cmd;

        build_motor_commands(throttle_cmd,
                             roll_cmd,
                             pitch_cmd,
                             yaw_cmd,
                             motor_cmd);

        Plant_Update(&plant, motor_cmd, dt);

        // Blow-up checks
        if (!isfinite(plant.roll.angle) ||
            !isfinite(plant.pitch.angle) ||
            !isfinite(plant.yaw.angle) ||
            !isfinite(plant.altitude.z))
        {
            return 1.0e9f;
        }

        if ((absf_local(plant.roll.angle) > 2.5f) ||
            (absf_local(plant.pitch.angle) > 2.5f) ||
            (absf_local(plant.yaw.angle) > 10.0f) ||
            (plant.altitude.z > 10.0f))
        {
            return 1.0e9f;
        }

        // Tracking cost
        tracking_cost += 8.0f * absf_local(roll_err) * dt;
        tracking_cost += 8.0f * absf_local(pitch_err) * dt;
        tracking_cost += 7.5f * absf_local(yaw_err) * dt;
        tracking_cost += 12.0f * absf_local(alt_err) * dt;

        // Control effort cost
        effort_cost += 0.003f * absf_local(roll_cmd) * dt;
        effort_cost += 0.003f * absf_local(pitch_cmd) * dt;
        effort_cost += 0.003f * absf_local(yaw_cmd) * dt;
        effort_cost += 0.002f * absf_local(alt_cmd) * dt;

        // Oscillation cost from error slope
        oscillation_cost += 2.0f * absf_local(roll_err - prev_roll_err);
        oscillation_cost += 2.0f * absf_local(pitch_err - prev_pitch_err);
        oscillation_cost += 1.5f * absf_local(yaw_err - prev_yaw_err);
        oscillation_cost += 2.0f * absf_local(alt_err - prev_alt_err);

        // Extra oscillation / overshoot penalty from sign flips
        if ((roll_err * prev_roll_err) < 0.0f)
        {
            oscillation_cost += 5.0f;
        }

        if ((pitch_err * prev_pitch_err) < 0.0f)
        {
            oscillation_cost += 5.0f;
        }

        if ((yaw_err * prev_yaw_err) < 0.0f)
        {
            oscillation_cost += 3.0f;
        }

        if ((alt_err * prev_alt_err) < 0.0f)
        {
            oscillation_cost += 5.0f;
        }

        prev_roll_err = roll_err;
        prev_pitch_err = pitch_err;
        prev_yaw_err = yaw_err;
        prev_alt_err = alt_err;

        // Cross-axis coupling cost
        if ((t >= 0.5f) && (t < 2.0f))
        {
            coupling_cost += 6.0f * absf_local(pitch) * dt;
            coupling_cost += 2.0f * absf_local(yaw) * dt;
            coupling_cost += 2.0f * absf_local(alt) * dt;
        }
        else if ((t >= 2.0f) && (t < 4.0f))
        {
            coupling_cost += 6.0f * absf_local(roll) * dt;
            coupling_cost += 2.0f * absf_local(yaw) * dt;
            coupling_cost += 2.0f * absf_local(alt) * dt;
        }
        else if ((t >= 6.0f) && (t < 8.0f))
        {
            coupling_cost += 2.5f * absf_local(roll) * dt;
            coupling_cost += 2.5f * absf_local(pitch) * dt;
            coupling_cost += 1.0f * absf_local(yaw) * dt;
        }

        // Saturation cost
        if (absf_local(roll_cmd) >= 249.0f)
        {
            saturation_cost += 2.0f * dt;
        }

        if (absf_local(pitch_cmd) >= 249.0f)
        {
            saturation_cost += 2.0f * dt;
        }

        if (absf_local(yaw_cmd) >= 119.0f)
        {
            saturation_cost += 2.0f * dt;
        }

        if (absf_local(alt_cmd) >= 399.0f)
        {
            saturation_cost += 3.0f * dt;
        }

        for (int i = 0; i < 4; i++)
        {
            if (motor_cmd[i] <= 1.0f || motor_cmd[i] >= 1999.0f)
            {
                saturation_cost += 0.5f * dt;
            }
        }
    }

    // Terminal error penalty so slow lazy responses lose
    terminal_cost += 20.0f * absf_local(plant.roll.angle - roll_ref);
    terminal_cost += 20.0f * absf_local(plant.pitch.angle - pitch_ref);
    terminal_cost += 15.0f * absf_local(plant.yaw.angle - yaw_ref);
    terminal_cost += 25.0f * absf_local(plant.altitude.z - alt_ref);

    // Light gain-balance penalty to discourage integral-dominated solutions
    gain_balance_cost += 0.5f * (gains->values[1] / (gains->values[0] + 1.0e-3f));
    gain_balance_cost += 0.5f * (gains->values[4] / (gains->values[3] + 1.0e-3f));
    gain_balance_cost += 0.5f * (gains->values[7] / (gains->values[6] + 1.0e-3f));
    gain_balance_cost += 0.25f * (gains->values[10] / (gains->values[9] + 1.0e-3f));

    {
        float total_cost =
            tracking_cost +
            effort_cost +
            1.5f * oscillation_cost +
            2.0f * coupling_cost +
            3.0f * saturation_cost +
            terminal_cost +
            gain_balance_cost;

        if (verbose)
        {
            printf("tracking_cost    = %.6f\n", tracking_cost);
            printf("effort_cost      = %.6f\n", effort_cost);
            printf("oscillation_cost = %.6f\n", oscillation_cost);
            printf("coupling_cost    = %.6f\n", coupling_cost);
            printf("saturation_cost  = %.6f\n", saturation_cost);
            printf("terminal_cost    = %.6f\n", terminal_cost);
            printf("gain_balance_cost= %.6f\n", gain_balance_cost);
            printf("total_cost       = %.6f\n", total_cost);
        }

        return total_cost;
    }
}
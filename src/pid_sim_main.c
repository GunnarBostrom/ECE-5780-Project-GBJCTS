/****************************************************************************************
 * File: pid_sim_main.c
 *
 * Josh Canada
 * Embedded Systems Final Project
 *
 * Description:
 * This file runs a simplified quadrotor hover simulation using:
 *
 * // Roll rotational dynamics
 * // Pitch rotational dynamics
 * // Yaw rotational dynamics
 * // Altitude dynamics
 * // Four first-order motor lag states
 *
 * The current active closed-loop controllers are:
 *
 * // Roll PID
 * // Altitude PID
 *
 * Pitch and yaw are presently held at zero command so that controller development can
 * proceed in stages.
 *
 * --------------------------------------------------------------------------------------
 * MODEL OVERVIEW
 * --------------------------------------------------------------------------------------
 * The plant is not a full nonlinear 6-DOF quadrotor model. It is a reduced-order hover
 * model intended for control framework development and initial PID tuning.
 *
 * The simulation flow is:
 *
 *   setpoint -> PID -> motor mixer -> plant update -> logged response
 *
 * Logged results are written to a CSV file that can be opened in MATLAB.
 *
 * --------------------------------------------------------------------------------------
 * BUILD INSTRUCTIONS (Linux / WSL / macOS)
 * --------------------------------------------------------------------------------------
 * From the root project directory, compile using:
 *
 *     gcc -Iinc src/pid_sim_main.c src/pid.c src/plant.c -lm -o sim
 *
 * Notes:
 * // -Iinc includes the header files in /inc
 * // -lm links the math library for sqrtf()
 * // -o sim creates an executable named sim
 *
 * --------------------------------------------------------------------------------------
 * RUN INSTRUCTIONS
 * --------------------------------------------------------------------------------------
 * Run the simulation:
 *
 *     ./sim
 *
 * The program will automatically save output to the next available file:
 *
 *     /mnt/c/Users/joshc/Documents/MATLAB/Embedded_Final_Project_Tuning/PID_Sim_N.csv
 *
 * where N is the first unused simulation number.
 *
 * --------------------------------------------------------------------------------------
 * IMPORTANT NOTES
 * --------------------------------------------------------------------------------------
 * // Controller outputs in this simulation are not true physical torque commands.
 * // The motor commands are speed-like simulation values used to exercise the mixer
 * // and hover plant.
 * // This model is intended for framework development, not final gain selection.
 *
 ****************************************************************************************/

#include <stdio.h>
#include <math.h>
#include "pid.h"
#include "plant.h"

// Define pi manually to avoid portability issues.
#define PI 3.14159265358979323846f

// Converts degrees to radians.
static float deg_to_rad(float deg)
{
    return deg * (PI / 180.0f);
}

// Converts radians to degrees.
static float rad_to_deg(float rad)
{
    return rad * (180.0f / PI);
}

// Clamp helper for actuator commands.
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

// Opens the next available CSV file in the MATLAB tuning folder.
static FILE *open_next_sim_file(void)
{
    char filename[256];

    for (int sim_num = 1; sim_num <= 10000; sim_num++)
    {
        snprintf(filename,
                 sizeof(filename),
                 "/mnt/c/Users/joshc/Documents/MATLAB/Embedded_Final_Project_Tuning/PID_Sim_%d.csv",
                 sim_num);

        FILE *test_fp = fopen(filename, "r");

        if (test_fp != NULL)
        {
            fclose(test_fp);
            continue;
        }

        FILE *write_fp = fopen(filename, "w");

        if (write_fp == NULL)
        {
            printf("Error: could not create output file:\n%s\n", filename);
            printf("Make sure the MATLAB output folder already exists.\n");
            return NULL;
        }

        printf("Saving simulation results to:\n%s\n", filename);
        return write_fp;
    }

    printf("Error: no available simulation filenames were found.\n");
    return NULL;
}

int main(void)
{
    // Simulation settings
    //
    // dt = 0.002 s corresponds to a 500 Hz control loop.
    const float dt = 0.002f;
    const float sim_time = 8.0f;
    const int total_steps = (int)(sim_time / dt);

    // Controllers
    PIDController roll_pid;
    PIDController pitch_pid;
    PIDController yaw_pid;
    PIDController altitude_pid;

    // Plant
    QuadPlant plant;

    // Open output file
    FILE *fp = open_next_sim_file();

    if (fp == NULL)
    {
        return 1;
    }

    // ----------------------------------------
    // PID initialization
    // ----------------------------------------
    //
    // These are moderate starting values for this simplified simulation.
    // They will almost certainly need tuning as the plant evolves.

    // Roll controller
    PID_Init(&roll_pid,
             140.0f,     // kp
             1.0f,     // ki
             0.1f,     // kd
             dt,
             -30.0f,  // output minimum
             30.0f,   // output maximum
             -5.0f,   // integrator minimum
             5.0f,    // integrator maximum
             0.02f);   // derivative filter time constant

    // Altitude controller
    PID_Init(&altitude_pid,
             400.0f,    // kp
             0.0f,    // ki
             0.4f,    // kd
             dt,
             -300.0f,  // output minimum
             300.0f,   // output maximum
             -80.0f,   // integrator minimum
             80.0f,    // integrator maximum
             0.03f);   // derivative filter time constant
    
    // Pitch controller
    PID_Init(&pitch_pid,
         120.0f,   // kp (start similar to roll)
         0.0f,     // ki
         0.11f,     // kd
         dt,
         -30.0f,
         30.0f,
         -5.0f,
         5.0f,
         0.02f);

    // Yaw controller (smaller gains!)
    PID_Init(&yaw_pid,
         55.0f,        // kp
         2.0f,         // ki
         0.06f,        // kd
         dt,
         -20.0f,
         20.0f,
         -5.0f,
         5.0f,
         0.02f);
    // ----------------------------------------
    // Plant initialization
    // ----------------------------------------
    Plant_Init(&plant,
               0.020f,      // J_roll
               0.020f,      // J_pitch
               0.040f,      // J_yaw
               0.020f,      // b_roll
               0.020f,      // b_pitch
               0.030f,      // b_yaw
               1.00f,       // mass
               0.30f,       // b_altitude
               0.050f,      // motor_tau
               0.00002f,    // k_thrust
               0.000002f,   // k_yaw
               0.10f);      // arm_length

    // Initial conditions
    plant.roll.angle = deg_to_rad(10.0f);
    plant.roll.rate = 0.0f;

    plant.pitch.angle = 0.0f;
    plant.pitch.rate = 0.0f;

    plant.yaw.angle = 0.0f;
    plant.yaw.rate = 0.0f;

    plant.altitude.z = 0.0f;
    plant.altitude.vz = 0.0f;

    // Setpoints
    float roll_setpoint_deg = 0.0f;
    float pitch_setpoint_deg = 0.0f;
    float yaw_setpoint_deg = 0.0f;
    float altitude_setpoint_m = 1.0f;

    // Compute nominal hover motor command
    //
    // 4 * k_thrust * omega_hover^2 = m * g
    //
    // omega_hover = sqrt((m * g) / (4 * k_thrust))
    float omega_hover =
        sqrtf((plant.altitude.mass * plant.g) / (4.0f * plant.k_thrust));

    printf("Computed hover command: %.3f\n", omega_hover);

    // CSV header
    fprintf(fp,"time,roll_sp,roll,pitch_sp,pitch,yaw_sp,yaw,alt_sp,alt,m0,m1,m2,m3\n");

    // Main simulation loop
    for (int step = 0; step < total_steps; step++)
    {
        float time_s = step * dt;

        // Read plant states
        float roll_deg = rad_to_deg(plant.roll.angle);
        float roll_rate_dps = rad_to_deg(plant.roll.rate);

        float pitch_deg = rad_to_deg(plant.pitch.angle);
        float pitch_rate_dps = rad_to_deg(plant.pitch.rate);

        float yaw_deg = rad_to_deg(plant.yaw.angle);
        float yaw_rate_dps = rad_to_deg(plant.yaw.rate);

        float altitude_m = plant.altitude.z;
        float vertical_velocity_mps = plant.altitude.vz;

        if (time_s > 1.0f && time_s < 3.0f)
        {
            roll_setpoint_deg = 10.0f;
        }
        else
        {
        roll_setpoint_deg = 0.0f;
        }

        if (time_s > 3.0f && time_s < 5.0f)
        {
            pitch_setpoint_deg = 10.0f;
        }
        else
        {
            pitch_setpoint_deg = 0.0f;
        }

        if (time_s > 5.0f && time_s < 7.0f)
        {
            yaw_setpoint_deg = 20.0f;
        }
        else
        {
            yaw_setpoint_deg = 0.0f;
        }

        // Controllers
        float roll_cmd = PID_Update(&roll_pid, roll_setpoint_deg, roll_deg);
        float pitch_cmd = PID_Update(&pitch_pid, pitch_setpoint_deg, pitch_deg);
        float yaw_cmd = PID_Update(&yaw_pid, yaw_setpoint_deg, yaw_deg);
        float altitude_cmd = PID_Update(&altitude_pid, altitude_setpoint_m, altitude_m);

        // Base collective command around hover
        float collective = omega_hover + altitude_cmd;

        // ----------------------------------------
        // Attitude command scaling before motor mixing
        // ----------------------------------------
        //
        // Keep roll behavior mostly intact.
        // Reduce pitch and yaw authority so they do not drive the motors into
        // aggressive switching behavior.
        //
        // These scale factors are mixer gains, not PID gains.
        const float roll_mix_scale = 1.00f;
        const float pitch_mix_scale = 0.35f;
        const float yaw_mix_scale = 0.20f;

        float roll_mix = roll_mix_scale * roll_cmd;
        float pitch_mix = pitch_mix_scale * pitch_cmd;
        float yaw_mix = yaw_mix_scale * yaw_cmd;

// ----------------------------------------
// Motor mixing
// ----------------------------------------
//
// Layout assumption:
//
//   Front
//    0   1
//    3   2
//
float motor_cmd[4];

motor_cmd[0] = collective - roll_mix - pitch_mix + yaw_mix;
motor_cmd[1] = collective + roll_mix - pitch_mix - yaw_mix;
motor_cmd[2] = collective + roll_mix + pitch_mix + yaw_mix;
motor_cmd[3] = collective - roll_mix + pitch_mix - yaw_mix;

    // Clamp commands to a reasonable nonnegative simulation range
for (int i = 0; i < 4; i++)
{
    motor_cmd[i] = clampf(motor_cmd[i], 0.0f, 2000.0f);
}

        // Update plant
        Plant_Update(&plant, motor_cmd, dt);

        // Logging
        fprintf(fp,
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
        time_s,
        roll_setpoint_deg,
        roll_deg,
        pitch_setpoint_deg,
        pitch_deg,
        yaw_setpoint_deg,
        yaw_deg,
        altitude_setpoint_m,
        altitude_m,
        motor_cmd[0],
        motor_cmd[1],
        motor_cmd[2],
        motor_cmd[3]);
    }

    fclose(fp);

    printf("Simulation complete.\n");
    return 0;
}
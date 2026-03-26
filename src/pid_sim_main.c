/****************************************************************************************
 * File: pid_sim_main.c
 *
 * Description:
 * This file runs a 1-axis PID control simulation for the quadrotor attitude system.
 * The simulation models the plant dynamics and applies a PID controller to track
 * a desired angle (setpoint). Results are printed to stdout and can be saved to a CSV
 * file for plotting in MATLAB.
 *
 * --------------------------------------------------------------------------------------
 * BUILD INSTRUCTIONS (Linux / WSL / macOS)
 * --------------------------------------------------------------------------------------
 * From the root project directory, compile using gcc:
 *
 *     gcc -Iinc src/pid_sim_main.c src/pid.c src/plant.c -lm -o sim
 *
 * Notes:
 * - -Iinc         : Includes header files from the /inc directory
 * - -lm           : Links the math library (required for sin, cos, etc.)
 * - -o sim        : Outputs executable named "sim"
 *
 * --------------------------------------------------------------------------------------
 * RUN INSTRUCTIONS
 * --------------------------------------------------------------------------------------
 * Run the simulation:
 *
 *     ./sim
 *
 * To save output to a CSV file for analysis:
 *
 *     ./sim > results.csv
 *
 * --------------------------------------------------------------------------------------
 * VIEWING RESULTS
 * --------------------------------------------------------------------------------------
 * MATLAB:
 *     data = readmatrix('results.csv');
 *     t = data(:,1);
 *     setpoint = data(:,2);
 *     angle = data(:,3);
 *     plot(t, setpoint, '--', t, angle);
 *     legend('Setpoint','Measured Angle');
 * --------------------------------------------------------------------------------------
 * NOTES
 * --------------------------------------------------------------------------------------
 * - The simulation timestep (dt) should match the intended embedded control loop rate
 *   (e.g., dt = 0.002 for 500 Hz).
 * - This is a single-axis model used for controller tuning before hardware deployment.
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

// Opens the next available CSV file in the MATLAB tuning folder.
//
// The function checks for files named:
//   PID_Sim_1.csv
//   PID_Sim_2.csv
//   PID_Sim_3.csv
//   ...
//
// It returns a writable FILE pointer for the first filename that does
// not already exist, which prevents old simulation results from being
// overwritten.
static FILE *open_next_sim_file(void)
{
    // Buffer used to build the full output path.
    char filename[256];

    // Try file numbers in ascending order until an unused name is found.
    for (int sim_num = 1; sim_num <= 10000; sim_num++)
    {
        // Build the full WSL path to the output CSV file.
        //
        // This corresponds to the Windows folder:
        // C:\Users\joshc\Documents\MATLAB\Embedded_Final_Project_Tuning
        snprintf(filename,
                 sizeof(filename),
                 "/mnt/c/Users/joshc/Documents/MATLAB/Embedded_Final_Project_Tuning/PID_Sim_%d.csv",
                 sim_num);

        // Attempt to open the file in read mode.
        //
        // If this succeeds, the file already exists, so we close it and
        // continue searching for the next available number.
        FILE *test_fp = fopen(filename, "r");

        if (test_fp != NULL)
        {
            fclose(test_fp);
            continue;
        }

        // If fopen in read mode failed, we assume the file does not exist,
        // so now we open it in write mode and return the handle.
        FILE *write_fp = fopen(filename, "w");

        if (write_fp == NULL)
        {
            printf("Error: could not create output file:\n%s\n", filename);
            return NULL;
        }

        // Print the chosen filename to the terminal so you know which run
        // was just created.
        printf("Saving simulation results to:\n%s\n", filename);

        return write_fp;
    }

    // If we somehow run out of filenames, return NULL.
    printf("Error: no available simulation filenames were found.\n");
    return NULL;
}

int main(void)
{
    // Simulation settings
    //
    // dt should match desired embedded system control rate
    //
    // dt = 1 / 500 = 0.002 seconds
    const float dt = 0.002f;
    const float sim_time = 8.0f;
    const int total_steps = (int)(sim_time / dt);

    // Controller and plant declarations
    PIDController pid;
    AxisPlant plant;

    // Open the next available CSV file so previous simulation runs are
    // preserved automatically.
    FILE *fp = open_next_sim_file();

    // Stop immediately if the output file could not be created.
    if (fp == NULL)
    {
        return 1;
    }

    // PID initialization
    //
    // These are only starting values. You will tune them by observing
    // the angle response and controller output.
    //
    // Suggested starting logic:
    // 1. Start with Ki = 0
    // 2. Tune Kp until it responds strongly
    // 3. Add Kd to reduce overshoot and oscillation
    // 4. Add a small Ki only after the other two look good
    PID_Init(&pid,
             40.0f,     // kp
             3.0f,     // ki
             0.03f,     // kd
             dt,
             -1.5f,    // output minimum
             1.5f,     // output maximum
             -0.5f,    // integrator minimum
             0.5f,     // integrator maximum
             0.02f);   // derivative filter time constant

    // Plant initialization
    //
    // These parameters define how "hard" the simulated axis is to control.
    //
    // J: larger J means more inertia, so the system is slower to accelerate
    // b: larger b means more damping, so oscillations die out faster
    // k: larger k means stronger restoring behavior
    //
    // Reasonable theoretical simulation values for controller development.
    Plant_Init(&plant,
               0.03f,   // J
               0.10f,   // b
               2.5f);   // k

    // Initial condition
    //
    // Start the plant with a nonzero disturbance so the controller has
    // something meaningful to correct.
    //
    // Example: initial pitch error of +12 degrees.
    plant.theta = deg_to_rad(12.0f);
    plant.omega = 0.0f;

    // Desired angle is level, which means zero degrees.
    float setpoint_deg = 0.0f;

    // Write optional metadata at the top of the file.
    //
    // MATLAB readtable will not like these lines unless you skip them, so
    // leave them commented out for now unless you want metadata in the file.
    //
    // fprintf(fp, "# Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pid.kp, pid.ki, pid.kd);
    // fprintf(fp, "# dt=%.6f, sim_time=%.3f\n", dt, sim_time);

    // Write CSV header to the output file.
    fprintf(fp, "time_s,setpoint_deg,theta_deg,omega_dps,control_output\n");

    // Main simulation loop
    for (int step = 0; step < total_steps; step++)
    {
        // Current simulation time.
        float time_s = step * dt;

        // Convert plant states to human-friendly controller units.
        //
        // For tuning, it is often easier to think in degrees and deg/s
        // instead of radians and rad/s.
        float theta_deg = rad_to_deg(plant.theta);
        float omega_dps = rad_to_deg(plant.omega);

        // Update the PID controller.
        //
        // Input:
        //   setpoint = desired angle in degrees
        //   measurement = current angle in degrees
        //
        // Output:
        //   torque-like command to the plant
        float control_output = PID_Update(&pid, setpoint_deg, theta_deg);

        // Update the simulated plant using the controller output.
        Plant_Update(&plant, control_output, dt);

        // Write every simulation step to the CSV file.
        fprintf(fp, "%.6f,%.6f,%.6f,%.6f,%.6f\n",
                time_s,
                setpoint_deg,
                theta_deg,
                omega_dps,
                control_output);
    }

    // Close the output file so all buffered data is saved correctly.
    fclose(fp);

    return 0;
}
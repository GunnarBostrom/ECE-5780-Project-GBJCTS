#include "control.h"
#include "pid.h"
#include "radio.h"
#include "motor.h"
#include <stdint.h>

// ESC PWM bounds 
#define THROTTLE_MIN_US 1000   // minimum throttle (motors idle)
#define THROTTLE_MAX_US 1900   // maximum throttle (full power)
#define THROTTLE_IDLE_DEADBAND_US 50
#define THROTTLE_STABILIZE_START_US 1100
#define THROTTLE_FULL_AUTHORITY_US 1250

// PID controllers for attitude stabilization
static PIDController roll_pid;
static PIDController pitch_pid;


// Clamp a signed value into a valid uint16_t range
// Used to ensure motor commands stay within safe PWM bounds
static uint16_t clamp_u16(int32_t value, uint16_t min_val, uint16_t max_val)
{
    if (value < min_val)
    {
        return min_val;
    }

    if (value > max_val)
    {
        return max_val;
    }

    return (uint16_t)value;
}

static float abs_f(float value)
{
    return (value < 0.0f) ? -value : value;
}


// Map raw radio throttle input to ESC pulse width (microseconds)
//
// Radio input range (empirical):
//   ~230  → minimum stick
//   ~1750 → maximum stick
//
// Output range:
//   1000–1900 us (standard ESC signal)
//
// This ensures consistent motor response regardless of radio scaling
static uint16_t map_throttle_to_us(uint16_t raw)
{
    if (raw <= 230)
    {
        return THROTTLE_MIN_US;
    }

    if (raw >= 1750)
    {
        return THROTTLE_MAX_US;
    }

    return (uint16_t)(((raw - 230) * (THROTTLE_MAX_US - THROTTLE_MIN_US)) / (1750 - 230) + THROTTLE_MIN_US);
}

// Initialize control system
//
// dt = control loop timestep (seconds)
//
// Sets up PID controllers for roll and pitch stabilization
void control_init(float dt)
{
    // Roll PID controller
    PID_Init(&roll_pid,
             6.0f,     // kp  proportional gain (main correction term)
             0.0f,     // ki  integral gain (eliminates steady-state error)
             0.02f,    // kd  derivative gain (damping / smoothing)
             dt,
             -180.0f,  // output min (limits correction authority)
             180.0f,   // output max
             -50.0f,   // integrator min (anti-windup)
             50.0f,    // integrator max
             0.02f);   // derivative filter time constant

    // Pitch PID controller (same tuning as roll for now)
    PID_Init(&pitch_pid,
             6.0f,
             0.0f,
             0.02f,
             dt,
             -180.0f,
             180.0f,
             -50.0f,
             50.0f,
             0.02f);
}

// Main control update (called at fixed loop rate)
//
// Inputs:
//   imu       → raw IMU data (currently unused here estimating at the attitude level from filter.c)
//   attitude  → estimated roll/pitch angles (deg)
//
// Function:
//   - Reads throttle from radio
//   - Runs PID control for roll/pitch
//   - Mixes commands into 4 motor outputs

void control_update(const IMU_t* imu, const Attitude_t* attitude)
{
    uint16_t throttle_us;   // base throttle command (ESC signal)
    float roll_sp_deg;      // desired roll angle (deg)
    float pitch_sp_deg;     // desired pitch angle (deg)

    float roll_cmd;         // PID output for roll correction
    float pitch_cmd;        // PID output for pitch correction

    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
    float authority;
    float correction_sum;
    float correction_headroom;

    (void)imu;  // IMU not used yet (placeholder for future use)

    // SAFETY: If not armed → shut everything down
    if (!radio_data.armed)
    {
        // Reset PID states to avoid integrator buildup
        PID_Reset(&roll_pid);
        PID_Reset(&pitch_pid);

        // Send minimum signal to all motors (disarmed state)
        motor_set_all(1000);
        return;
    }

    // Convert radio throttle → ESC PWM signal
    throttle_us = map_throttle_to_us(radio_data.throttle);

    // Keep motors quiet at zero/idle stick. Without this, PID corrections can
    // still be mixed into a 1000 us throttle command and spin motors hard.
    if (throttle_us <= (THROTTLE_MIN_US + THROTTLE_IDLE_DEADBAND_US))
    {
        PID_Reset(&roll_pid);
        PID_Reset(&pitch_pid);
        motor_set_all(THROTTLE_MIN_US);
        return;
    }

    // Below this point, keep all motors equal. There is not enough headroom for
    // attitude correction without clipping some motors down to 1000 us.
    if (throttle_us < THROTTLE_STABILIZE_START_US)
    {
        PID_Reset(&roll_pid);
        PID_Reset(&pitch_pid);
        motor_set_all(throttle_us);
        return;
    }

    // Self-level mode (no user angle input yet)
    // Forces quad to stay level (0 deg roll/pitch)
    roll_sp_deg = 0.0f;
    pitch_sp_deg = 0.0f;

    // Compute PID corrections based on current attitude
    roll_cmd = PID_Update(&roll_pid, roll_sp_deg, attitude->roll_deg);
    pitch_cmd = PID_Update(&pitch_pid, pitch_sp_deg, attitude->pitch_deg);

    authority = (float)(throttle_us - THROTTLE_STABILIZE_START_US) /
                (float)(THROTTLE_FULL_AUTHORITY_US - THROTTLE_STABILIZE_START_US);
    if (authority > 1.0f)
    {
        authority = 1.0f;
    }

    roll_cmd *= authority;
    pitch_cmd *= authority;

    // Scale corrections before mixing so the requested motor values do not hit
    // the final 1000/1900 us clamps. Clipping hides real control behavior.
    correction_sum = abs_f(roll_cmd) + abs_f(pitch_cmd);
    correction_headroom = (float)(throttle_us - THROTTLE_MIN_US);
    if ((float)(THROTTLE_MAX_US - throttle_us) < correction_headroom)
    {
        correction_headroom = (float)(THROTTLE_MAX_US - throttle_us);
    }
    if (correction_headroom > 10.0f)
    {
        correction_headroom -= 10.0f;
    }
    if ((correction_sum > correction_headroom) && (correction_sum > 1.0f))
    {
        const float scale = correction_headroom / correction_sum;
        roll_cmd *= scale;
        pitch_cmd *= scale;
    }

    
    // MOTOR MIXING (Quad X configuration)
    //
    // Each motor gets:
    //   base throttle ± pitch correction ± roll correction
    //
    // Layout assumption:
    //   m1: front-left
    //   m2: front-right
    //   m3: rear-left
    //   m4: rear-right
    //
    // Signs determine how each motor contributes to rotation
    m1 = clamp_u16((int32_t)(throttle_us + pitch_cmd + roll_cmd), 1000, 1900);
    m2 = clamp_u16((int32_t)(throttle_us + pitch_cmd - roll_cmd), 1000, 1900);
    m3 = clamp_u16((int32_t)(throttle_us - pitch_cmd + roll_cmd), 1000, 1900);
    m4 = clamp_u16((int32_t)(throttle_us - pitch_cmd - roll_cmd), 1000, 1900);

    // Send PWM commands to ESCs
    motor_set_individual(m1, m2, m3, m4);
}

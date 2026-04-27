#include "control.h"
#include "pid.h"
#include "radio.h"
#include "motor.h"
#include <stdint.h>

// ESC PWM bounds
#define THROTTLE_MIN_US 1000
#define THROTTLE_MAX_US 2000
#define THROTTLE_IDLE_DEADBAND_US 50
#define THROTTLE_ATTITUDE_START_US (THROTTLE_MIN_US + THROTTLE_IDLE_DEADBAND_US)
#define THROTTLE_FULL_AUTHORITY_US 1250

static PIDController roll_pid;
static PIDController pitch_pid;

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

static uint16_t map_throttle_to_us(uint16_t raw)
{
    if (raw <= 230U)
    {
        return THROTTLE_MIN_US;
    }

    if (raw >= 1750U)
    {
        return THROTTLE_MAX_US;
    }

    return (uint16_t)(((raw - 230U) * (THROTTLE_MAX_US - THROTTLE_MIN_US)) /
                      (1750U - 230U) + THROTTLE_MIN_US);
}

void control_init(float dt)
{
    PID_Init(&roll_pid,
             2.0f,
             0.0f,
             0.02f,
             dt,
             -180.0f,
             180.0f,
             -50.0f,
             50.0f,
             0.02f);

    PID_Init(&pitch_pid,
             2.2f,
             0.0f,
             0.02f,
             dt,
             -180.0f,
             180.0f,
             -50.0f,
             50.0f,
             0.02f);
}

void control_update(const IMU_t* imu, const Attitude_t* attitude)
{
    uint16_t throttle_us;
    float roll_sp_deg;
    float pitch_sp_deg;
    float roll_cmd;
    float pitch_cmd;
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
    float authority;
    float correction_sum;
    float correction_headroom;

    (void)imu;

    if (!radio_data.armed)
    {
        PID_Reset(&roll_pid);
        PID_Reset(&pitch_pid);
        motor_set_all(THROTTLE_MIN_US);
        return;
    }

    throttle_us = map_throttle_to_us(radio_data.throttle);

    if (throttle_us <= (THROTTLE_MIN_US + THROTTLE_IDLE_DEADBAND_US))
    {
        PID_Reset(&roll_pid);
        PID_Reset(&pitch_pid);
        motor_set_all(THROTTLE_MIN_US);
        return;
    }

    roll_sp_deg = 0.0f;
    pitch_sp_deg = 0.0f;

    roll_cmd = PID_Update(&roll_pid, roll_sp_deg, attitude->roll_deg);
    pitch_cmd = -PID_Update(&pitch_pid, pitch_sp_deg, attitude->pitch_deg);

    authority = (float)(throttle_us - THROTTLE_ATTITUDE_START_US) /
                (float)(THROTTLE_FULL_AUTHORITY_US - THROTTLE_ATTITUDE_START_US);
    if (authority < 0.0f)
    {
        authority = 0.0f;
    }
    if (authority > 1.0f)
    {
        authority = 1.0f;
    }

    roll_cmd *= authority;
    pitch_cmd *= authority;

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
        float scale = correction_headroom / correction_sum;
        roll_cmd *= scale;
        pitch_cmd *= scale;
    }

    m1 = clamp_u16((int32_t)(throttle_us + pitch_cmd + roll_cmd), THROTTLE_MIN_US, THROTTLE_MAX_US);
    m2 = clamp_u16((int32_t)(throttle_us + pitch_cmd - roll_cmd), THROTTLE_MIN_US, THROTTLE_MAX_US);
    m3 = clamp_u16((int32_t)(throttle_us - pitch_cmd + roll_cmd), THROTTLE_MIN_US, THROTTLE_MAX_US);
    m4 = clamp_u16((int32_t)(throttle_us - pitch_cmd - roll_cmd), THROTTLE_MIN_US, THROTTLE_MAX_US);

    motor_set_individual(m1, m2, m3, m4);
}

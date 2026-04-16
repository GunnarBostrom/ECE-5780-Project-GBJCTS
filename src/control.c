#include "control.h"
#include "pid.h"
#include "radio.h"
#include "motor.h"
#include "pid.h"
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include <stdint.h>

#define THROTTLE_MIN_US 1000
#define THROTTLE_MAX_US 1900

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

void control_init(float dt)
{
    PID_Init(&roll_pid,
             5.0f,
             0.0f,
             0.2f,
             dt,
             -250.0f,
             250.0f,
             -50.0f,
             50.0f,
             0.02f);

    PID_Init(&pitch_pid,
             5.0f,
             0.0f,
             0.2f,
             dt,
             -250.0f,
             250.0f,
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

    (void)imu;

    if (!radio_data.armed)
    {
        PID_Reset(&roll_pid);
        PID_Reset(&pitch_pid);
        motor_set_all(1000);
        return;
    }

    throttle_us = map_throttle_to_us(radio_data.throttle);

    // Self-level mode for initial testing
    roll_sp_deg = 0.0f;
    pitch_sp_deg = 0.0f;

    roll_cmd = PID_Update(&roll_pid, roll_sp_deg, attitude->roll_deg);
    pitch_cmd = PID_Update(&pitch_pid, pitch_sp_deg, attitude->pitch_deg);

    m1 = clamp_u16((int32_t)(throttle_us + pitch_cmd + roll_cmd), 1000, 1900);
    m2 = clamp_u16((int32_t)(throttle_us + pitch_cmd - roll_cmd), 1000, 1900);
    m3 = clamp_u16((int32_t)(throttle_us - pitch_cmd - roll_cmd), 1000, 1900);
    m4 = clamp_u16((int32_t)(throttle_us - pitch_cmd + roll_cmd), 1000, 1900);

    motor_set_individual(m1, m2, m3, m4);
}
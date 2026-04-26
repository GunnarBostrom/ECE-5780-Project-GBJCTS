#include "control.h"
#include "motor.h"
#include "pid.h"
#include "radio.h"
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* ESC command limits                                                   */
/* ------------------------------------------------------------------ */
#define THROTTLE_MIN_US            1000
#define THROTTLE_MAX_US            1900
#define THROTTLE_IDLE_DEADBAND_US    50
#define THROTTLE_ATTITUDE_START_US (THROTTLE_MIN_US + THROTTLE_IDLE_DEADBAND_US)
#define THROTTLE_FULL_AUTHORITY_US 1250

/* ------------------------------------------------------------------ */
/* Precomputed Tustin derivative coefficients (tau=0.02s, dt=1/416s)
 *
 *   2τ = 40000 µs,  dt = 2404 µs  (= 1 000 000 / 416)
 *   coeff_d = (2τ-dt)/(2τ+dt) = 37596/42404 = 0.8867
 *   coeff_d_q12 = (int)(0.8867 × 4096) = 3631
 *
 *   coeff_e [µs/deg] = 2kd / (2τ+dt_s)
 *   kd=0.04: 2×0.04/0.042404 = 1.887 → × 100 = 189
 */
#define COEFF_D_Q12            3631
#define COEFF_E_LEVEL_X100      189

/* ------------------------------------------------------------------ */
static PIDController roll_pid;
static PIDController pitch_pid;


static int32_t abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static uint16_t clamp_u16(int32_t v, uint16_t lo, uint16_t hi)
{
    if (v < (int32_t)lo) return lo;
    if (v > (int32_t)hi) return hi;
    return (uint16_t)v;
}

/* Map CRSF 11-bit throttle channel (0-2047) to ESC µs range. */
static uint16_t map_throttle_to_us(uint16_t raw)
{
    if (raw <= 230U)   return THROTTLE_MIN_US;
    if (raw >= 1750U)  return THROTTLE_MAX_US;
    return (uint16_t)(((uint32_t)(raw - 230U)
                        * (THROTTLE_MAX_US - THROTTLE_MIN_US))
                       / (1750U - 230U)
                       + THROTTLE_MIN_US);
}

void control_init(void)
{
    /* Roll/pitch self-level tune:
     * keep it conservative for takeoff and landing, with no integral windup
     * while the craft is on the ground. */
    PID_Init(&roll_pid,
             360,
             0,
             COEFF_D_Q12,
             COEFF_E_LEVEL_X100,
             -160, 
             160,
             -5000, 
             5000);

    PID_Init(&pitch_pid,
             340, 
             0,
             COEFF_D_Q12, 
             COEFF_E_LEVEL_X100,
             -160, 
             160,
             -5000, 5000);
}

void control_update(const IMU_t *imu, const Attitude_t *attitude)
{
    (void)imu;

    if (!radio_data.armed || radio_data.failsafe)
    {
        PID_Reset(&roll_pid);
        PID_Reset(&pitch_pid);
        motor_set_all(THROTTLE_MIN_US);
        return;
    }

    uint16_t throttle_us = map_throttle_to_us(radio_data.throttle);

    if (throttle_us <= THROTTLE_ATTITUDE_START_US)
    {
        PID_Reset(&roll_pid);
        PID_Reset(&pitch_pid);
        motor_set_all(THROTTLE_MIN_US);
        return;
    }

    /* Self-level: setpoint = 0° for both axes */
    int32_t roll_cmd  =  PID_Update(&roll_pid,  0, attitude->roll_cdeg);
    int32_t pitch_cmd = -PID_Update(&pitch_pid, 0, attitude->pitch_cdeg);

    /* Ramp authority 0→100% as throttle rises from ATTITUDE_START to FULL_AUTHORITY */
    int32_t authority = (int32_t)(throttle_us - THROTTLE_ATTITUDE_START_US) * 100
                        / (THROTTLE_FULL_AUTHORITY_US - THROTTLE_ATTITUDE_START_US);
    if (authority < 0)   authority = 0;
    if (authority > 100) authority = 100;

    roll_cmd  = roll_cmd  * authority / 100;
    pitch_cmd = pitch_cmd * authority / 100;

    /* Scale corrections to fit within the headroom around the throttle
     * command so individual motor outputs never clip hard. */
    int32_t sum = abs32(roll_cmd) + abs32(pitch_cmd);
    int32_t headroom = (int32_t)(throttle_us - THROTTLE_MIN_US);
    if ((int32_t)(THROTTLE_MAX_US - throttle_us) < headroom)
    {
        headroom = (int32_t)(THROTTLE_MAX_US - throttle_us);
    }
    if (headroom > 10) headroom -= 10;

    if (sum > headroom && sum > 0)
    {
        roll_cmd  = roll_cmd  * headroom / sum;
        pitch_cmd = pitch_cmd * headroom / sum;
    }

    /* Quad-X motor mix:
     *   m1 front-left,  m2 front-right
     *   m3 rear-left,   m4 rear-right  */
    uint16_t m1 = clamp_u16((int32_t)throttle_us + pitch_cmd + roll_cmd,
                             THROTTLE_MIN_US, THROTTLE_MAX_US);
    uint16_t m2 = clamp_u16((int32_t)throttle_us + pitch_cmd - roll_cmd,
                             THROTTLE_MIN_US, THROTTLE_MAX_US);
    uint16_t m3 = clamp_u16((int32_t)throttle_us - pitch_cmd + roll_cmd,
                             THROTTLE_MIN_US, THROTTLE_MAX_US);
    uint16_t m4 = clamp_u16((int32_t)throttle_us - pitch_cmd - roll_cmd,
                             THROTTLE_MIN_US, THROTTLE_MAX_US);

    motor_set_individual(m1, m2, m3, m4);
}

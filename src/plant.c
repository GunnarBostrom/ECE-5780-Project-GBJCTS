#include "plant.h"

static float clamp_nonnegative(float x)
{
    if (x < 0.0f)
    {
        return 0.0f;
    }

    return x;
}

void Plant_Init(
    QuadPlant *plant,
    float J_roll,
    float J_pitch,
    float J_yaw,
    float b_roll,
    float b_pitch,
    float b_yaw,
    float mass,
    float b_altitude,
    float motor_tau,
    float k_thrust,
    float k_yaw,
    float arm_length)
{
    // Zero rotational states.
    plant->roll.angle = 0.0f;
    plant->roll.rate = 0.0f;
    plant->roll.J = J_roll;
    plant->roll.b = b_roll;

    plant->pitch.angle = 0.0f;
    plant->pitch.rate = 0.0f;
    plant->pitch.J = J_pitch;
    plant->pitch.b = b_pitch;

    plant->yaw.angle = 0.0f;
    plant->yaw.rate = 0.0f;
    plant->yaw.J = J_yaw;
    plant->yaw.b = b_yaw;

    // Zero altitude states.
    plant->altitude.z = 0.0f;
    plant->altitude.vz = 0.0f;
    plant->altitude.mass = mass;
    plant->altitude.b = b_altitude;

    // Zero motor states and store motor dynamics.
    for (int i = 0; i < 4; i++)
    {
        plant->motors[i].omega = 0.0f;
        plant->motors[i].tau = motor_tau;
    }

    // Store physical constants.
    plant->g = 9.81f;
    plant->k_thrust = k_thrust;
    plant->k_yaw = k_yaw;
    plant->arm_length = arm_length;
}

void Plant_Update(QuadPlant *plant, const float motor_cmd[4], float dt)
{
    float thrust[4];
    float yaw_reaction[4];

    // Additional numerical / aerodynamic damping on rotational rates.
    //
    // This helps the simplified hover model behave more like a real quadrotor
    // and prevents persistent undamped oscillation in roll, pitch, and yaw.
    //
    // Values should be close to 1.0. Smaller values mean more damping.
    const float roll_rate_damping = 0.995f;
    const float pitch_rate_damping = 0.995f;
    const float yaw_rate_damping = 0.995f;

    // Motor layout assumption:
    //
    //   Front
    //    0   1
    //    3   2
    //
    // Example spin directions:
    //   Motor 0: CCW
    //   Motor 1: CW
    //   Motor 2: CCW
    //   Motor 3: CW
    //
    // You can flip signs later if your mixer convention changes.

    // Step 1: Update motor first-order lag dynamics.
    for (int i = 0; i < 4; i++)
    {
        float cmd = clamp_nonnegative(motor_cmd[i]);
        float tau = plant->motors[i].tau;

        // Avoid divide-by-zero if someone sets tau badly.
        if (tau < 1.0e-6f)
        {
            tau = 1.0e-6f;
        }

        float domega = (cmd - plant->motors[i].omega) / tau;
        plant->motors[i].omega += domega * dt;

        if (plant->motors[i].omega < 0.0f)
        {
            plant->motors[i].omega = 0.0f;
        }
    }

    // Step 2: Convert motor states to thrust and yaw reaction torques.
    for (int i = 0; i < 4; i++)
    {
        float w = plant->motors[i].omega;
        float w2 = w * w;

        thrust[i] = plant->k_thrust * w2;

        // Alternate sign by motor direction.
        if ((i == 0) || (i == 2))
        {
            // CCW
            yaw_reaction[i] = plant->k_yaw * w2;
        }
        else
        {
            // CW
            yaw_reaction[i] = -plant->k_yaw * w2;
        }
    }

    // Step 3: Compute summed forces and torques.
    float total_thrust =
        thrust[0] + thrust[1] + thrust[2] + thrust[3];

    // Simplified torques from arm geometry.
    //
    // Roll torque:
    // right side thrust minus left side thrust
    float tau_roll =
        plant->arm_length * ((thrust[1] + thrust[2]) - (thrust[0] + thrust[3]));

    // Pitch torque:
    // rear thrust minus front thrust
    float tau_pitch =
        plant->arm_length * ((thrust[2] + thrust[3]) - (thrust[0] + thrust[1]));

    // Yaw torque from drag reaction
    float tau_yaw =
        yaw_reaction[0] + yaw_reaction[1] + yaw_reaction[2] + yaw_reaction[3];

    // Step 4: Update rotational axes.
    {
        float rate_dot =
            (tau_roll - plant->roll.b * plant->roll.rate) / plant->roll.J;

        plant->roll.rate =
            roll_rate_damping * plant->roll.rate + rate_dot * dt;

        plant->roll.angle += plant->roll.rate * dt;
    }

    {
        float rate_dot =
            (tau_pitch - plant->pitch.b * plant->pitch.rate) / plant->pitch.J;

        plant->pitch.rate =
            pitch_rate_damping * plant->pitch.rate + rate_dot * dt;

        plant->pitch.angle += plant->pitch.rate * dt;
    }

    {
        float rate_dot =
            (tau_yaw - plant->yaw.b * plant->yaw.rate) / plant->yaw.J;

        plant->yaw.rate =
            yaw_rate_damping * plant->yaw.rate + rate_dot * dt;

        plant->yaw.angle += plant->yaw.rate * dt;
    }

    // Step 5: Update altitude.
    //
    // Slight realism upgrade:
    // reduce effective vertical thrust when roll/pitch tilt away from level.
    float vertical_thrust =
        total_thrust;

    // Small-angle correction without needing extra trig complexity later.
    // This is still simple enough for your milestone.
    {
        float roll_sq = plant->roll.angle * plant->roll.angle;
        float pitch_sq = plant->pitch.angle * plant->pitch.angle;

        // Approximation of cos(roll)*cos(pitch) near hover:
        // cos(x) ~= 1 - x^2 / 2
        float tilt_scale = (1.0f - 0.5f * roll_sq) * (1.0f - 0.5f * pitch_sq);

        if (tilt_scale < 0.0f)
        {
            tilt_scale = 0.0f;
        }

        vertical_thrust *= tilt_scale;
    }

    {
        float mass = plant->altitude.mass;

        float az =
            (vertical_thrust - mass * plant->g - plant->altitude.b * plant->altitude.vz) / mass;

        plant->altitude.vz += az * dt;
        plant->altitude.z += plant->altitude.vz * dt;

        // Prevent going below ground for simulation convenience.
        if (plant->altitude.z < 0.0f)
        {
            plant->altitude.z = 0.0f;

            if (plant->altitude.vz < 0.0f)
            {
                plant->altitude.vz = 0.0f;
            }
        }
    }
}
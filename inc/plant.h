#ifndef PLANT_H_
#define PLANT_H_

typedef struct
{
    float angle;
    float rate;
    float J;
    float b;
} RotAxis;

typedef struct
{
    float z;
    float vz;
    float mass;
    float b;
} AltitudeAxis;

typedef struct
{
    float omega;
    float tau;
} MotorState;

typedef struct
{
    RotAxis roll;
    RotAxis pitch;
    RotAxis yaw;

    AltitudeAxis altitude;

    MotorState motors[4];

    float g;
    float k_thrust;
    float k_yaw;
    float arm_length;
} QuadPlant;

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
    float arm_length);

void Plant_Update(QuadPlant *plant, const float motor_cmd[4], float dt);

#endif
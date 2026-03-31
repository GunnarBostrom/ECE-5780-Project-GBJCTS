#include "plant.h"

void Plant_Init(AxisPlant *plant, float J, float b, float k)
{
    // Initialize the plant state to rest.
    plant->theta = 0.0f;
    plant->omega = 0.0f;

    // Store physical model parameters.
    plant->J = J;
    plant->b = b;
    plant->k = k;
}

void Plant_Update(AxisPlant *plant, float torque_cmd, float dt)
{
    // Compute angular acceleration from the simplified plant model.
    //
    // omega_dot = (u - b*omega - k*theta) / J
    float omega_dot =
        (torque_cmd - plant->b * plant->omega - plant->k * plant->theta) / plant->J;

    // Integrate angular velocity forward in time.
    plant->omega += omega_dot * dt;

    // Integrate angular position forward in time.
    plant->theta += plant->omega * dt;
}
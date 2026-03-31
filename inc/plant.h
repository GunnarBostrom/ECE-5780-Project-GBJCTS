#ifndef PLANT_H
#define PLANT_H

// Simple one-axis rotational plant used for pid simulation.
//
// This is not a full 6-DOF quadrotor model. It is a reduced-order,
// single-axis rotational model that is good for initial PID tuning.
//
// State variables:
//   theta = angle in radians
//   omega = angular rate in radians per second
//
// Parameters:
//   J = rotational inertia
//   b = rotational damping
//   k = restoring stiffness
//
// The model is:
//
//   theta_dot = omega
//   omega_dot = (u - b*omega - k*theta) / J
//
// For simulation purposes to study controller behavior.
typedef struct
{
    // Angular position in radians.
    float theta;

    // Angular velocity in radians/second.
    float omega;

    // Rotational inertia.
    float J;

    // Damping coefficient.
    float b;

    // Restoring coefficient.
    //
    // This can represent a simplified tendency of the plant to move
    // back toward equilibrium, or just a linearized stabilizing term.
    float k;

} AxisPlant;

// Initializes the plant parameters and zeros the state.
void Plant_Init(AxisPlant *plant, float J, float b, float k);

// Advances the plant by one simulation time step using Euler integration.
//
// torque_cmd is the control input.
// dt is the simulation time step in seconds.
void Plant_Update(AxisPlant *plant, float torque_cmd, float dt);

#endif
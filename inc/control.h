#ifndef CONTROL_H
#define CONTROL_H

#include "imu.h"
#include "filter.h"

//void control_from_radio(void);
void control_init(float dt);
void control_update(const IMU_t* imu, const Attitude_t* attitude);

#endif
/**
LiDAR
STM VL53L1X

*/

#ifndef LIDAR_H
#define LIDAR_H

#include "config.h"
#include "stm32f0xx_hal.h" // Needed for HAL types if you use them later

void lidar_init(void);
void lidar_read(void);

#endif // LIDAR_H
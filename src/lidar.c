/**
LiDAR
STM VL53L1X

VL53L1X can operate at "up to 400 kHz".

WHO_AM_I register: 0x010F
Device ID: 0xEA
I2C slave address: 0x52
*/

#include "lidar.h"


#if USE_LIDAR // use REAL hardware
void lidar_init(void) { /* HAL_I2C_Init... */ }
void lidar_read(void) { /* HAL_I2C_Receive... */ }

#else //use FAKE hardware
void lidar_init(void) { /* do nothing */ }
void lidar_read(void) { /* do nothing */ }
#endif
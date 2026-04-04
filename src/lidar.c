/**
LiDAR

STM VL53L1X
Can operate at "up to 400 kHz".

Data peripheral provides:
    - range
    - range standard deviation
    - range validity
    - signal strength
    - ambient interference
    - reading count
    - active SPAD count

I2C slave address: 0x52
WHO_AM_I register: 0x010F
Device ID: 0xEA
*/

#include "config.h"
#include "lidar.h"

// define device
// defien output regs
// define config regs
// define config vals


#if USE_LIDAR // use REAL hardware

void lidar_init(void) {}
void lidar_read_range(void) {}
void lidar_read_sigma(void) {}
void lidar_read_status(void) {}
void lidar_read_strength(void) {}
void lidar_read_interference(void) {}


#else //use FAKE hardware
void lidar_init(void) { /* do nothing */ }
void lidar_read_range(void) { /* do nothing */ }
void lidar_read_sigma(void) { /* do nothing */ }
void lidar_read_status(void) { /* do nothing */ }
void lidar_read_strength(void) { /* do nothing */ }
void lidar_read_interference(void) { /* do nothing */ }
#endif
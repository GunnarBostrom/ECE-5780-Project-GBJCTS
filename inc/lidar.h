/**
LiDAR driver
STM VL53L1X

*/

#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx_hal.h" // probably not needed

// just what we need
typedef struct {
    uint16_t range_mm;      // range in millimeters
    //uint32_t range_sigma;   // signal standard deviation      (divide by 65536 for real value)
    uint8_t range_status;   // signal validity                (0 = valid)
    uint16_t strength;      // signal rate                    (divide by 65536 for real value)
    uint16_t interference;  // background interference rate   (divide by 65536 for real value)
} VL53L1X_t;

// all data peripheral provides
// typedef struct {
//     uint8_t  stream_count;
//     uint32_t signal_rate;   // divide by 65536 for real value
//     uint32_t ambient_rate;  // divide by 65536 for real value
//     uint16_t spad_count;    // divide by 256 for real value
//     uint32_t sigma_mm;      // divide by 65536 for real value
//     int16_t  range_mm;      // distance in mm
//     uint8_t  range_status;  // 0 = valid
// } TOF_t;

bool lidar_init(VL53L1X_t* lidar);
bool lidar_read(VL53L1X_t* lidar);

// interrupt must be cleared after calling any of the functions below
bool lidar_read_range(VL53L1X_t* lidar);
//bool lidar_read_sigma(VL53L1X_t* lidar); // might only be available through full driver
bool lidar_read_status(VL53L1X_t* lidar);
bool lidar_read_strength(VL53L1X_t* lidar);
bool lidar_read_interference(VL53L1X_t* lidar);

bool lidar_clear_interrupt(void);

#endif // LIDAR_H
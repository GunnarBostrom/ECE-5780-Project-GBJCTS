/**
LiDAR
STM VL53L1X

*/

#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include "stm32f0xx_hal.h" // probably not needed

// just what we need
typedef struct {
    uint8_t slave_addr;
    uint16_t range_mm;      // range in millimeters
    uint32_t range_sigma;   // signal standard deviation (divide by 65536 for real value)
    uint8_t range_status;   // signal validity           (0 = valid)
    uint32_t signal_rate;   // signal strength           (divide by 65536 for real value)
    uint32_t ambient_rate;  // background interference   (divide by 65536 for real value)
} LIDAR_t;

// all data peripheral provides
typedef struct {
    uint8_t  stream_count;
    uint32_t signal_rate;   // divide by 65536 for real value
    uint32_t ambient_rate;  // divide by 65536 for real value
    uint16_t spad_count;    // divide by 256 for real value
    uint32_t sigma_mm;      // divide by 65536 for real value
    int16_t  range_mm;      // distance in mm
    uint8_t  range_status;  // 0 = valid
} TOF_t;

void lidar_init(LIDAR_t *lidar, uint8_t slave_addr);
void lidar_read_range(LIDAR_t *lidar);
void lidar_read_sigma(LIDAR_t *lidar);
void lidar_read_status(LIDAR_t *lidar);
void lidar_read_strength(LIDAR_t *lidar);
void lidar_read_interference(LIDAR_t *lidar);

#endif // LIDAR_H
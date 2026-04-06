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
#include "i2c.h"
#include "VL53L1X_api.h"

// define device
#define WHO_AM_I_REG    0x010F
#define DEVICE_ID       0xEA

// define output regs


// define config regs
// define config vals


#if USE_LIDAR // use REAL hardware

void lidar_init(LIDAR_t *lidar, uint8_t slave_addr) {
    lidar->slave_addr = slave_addr;
    
    // verify device
    uint8_t buf[1];
    i2c_read(lidar->slave_addr, WHO_AM_I_REG, buf, 1);
    if (buf[0] != DEVICE_ID) {
        // wrong device: throw an error
        return;
    }

    // calibration vals (RefSPAD, offset, crosstalk)

    // configs

    // interrupt
}

void lidar_read_range(LIDAR_t *lidar) {}
void lidar_read_sigma(LIDAR_t *lidar) {}
void lidar_read_status(LIDAR_t *lidar) {}
void lidar_read_strength(LIDAR_t *lidar) {}
void lidar_read_interference(LIDAR_t *lidar) {}


#else //use FAKE hardware
void lidar_init(LIDAR_t *lidar, uint8_t slave_addr) { /* do nothing */ }
void lidar_read_range(LIDAR_t *lidar) { /* do nothing */ }
void lidar_read_sigma(LIDAR_t *lidar) { /* do nothing */ }
void lidar_read_status(LIDAR_t *lidar) { /* do nothing */ }
void lidar_read_strength(LIDAR_t *lidar) { /* do nothing */ }
void lidar_read_interference(LIDAR_t *lidar) { /* do nothing */ }
#endif
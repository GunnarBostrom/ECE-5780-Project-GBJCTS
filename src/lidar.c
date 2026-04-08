// /**
// LiDAR

// STM VL53L1X
// Can operate at "up to 400 kHz".

// Data peripheral provides:
//     - range
//     - range standard deviation
//     - range validity
//     - signal strength
//     - ambient interference
//     - reading count
//     - active SPAD count


// lidar.c --> VL53L1X_api.c --> vl53l1x_platform.c --> i2c.c

// I2C slave address: 0x52
// WHO_AM_I register: 0x010F
// Device ID: 0xEA
// */

// #include "config.h"
// #include "lidar.h"
// #include "i2c.h" // remove?
// #include "VL53L1X_api.h"

// // define device
// #define WHO_AM_I_REG    0x010F
// #define DEVICE_ID       0xEA        // or maybe 0xEEAC

// // define output regs


// // define config regs
// // define config vals


// VL53L1X_Result_t lidar_data;
// VL53L1X_ERROR lidar_error;
// uint16_t dev;
// uint16_t lidar_id = DEVICE_ID;
// uint8_t state;

// uint8_t Status;		/*!< ResultStatus */
// uint16_t Distance;	/*!< ResultDistance */
// uint16_t Ambient;	/*!< ResultAmbient */
// uint16_t SigPerSPAD;/*!< ResultSignalPerSPAD */
// uint16_t NumSPADs;	/*!< ResultNumSPADs */


// //#if USE_LIDAR // use REAL hardware

// void lidar_init(LIDAR_t *lidar, uint16_t slave_addr) {
//     lidar->slave_addr = slave_addr;
//     dev = slave_addr;

//     // initialize
//     lidar_error = VL53L1X_SensorInit(slave_addr);
//     //wait for a bit?
//     VL53L1X_BootState(lidar->slave_addr, &state);
    
//     if (VL53L1X_GetSensorId(lidar->slave_addr, &lidar_id) != DEVICE_ID) {
//         // wrong device: throw an error
//         return;
//     }

    



//     // configure
//     VL53L1X_SetTimingBudgetInMs(uint16_t dev, uint16_t TimingBudgetInMs);

//     uint16_t distance_mode = 1; // use short mode
//     VL53L1X_SetDistanceMode(uint16_t dev, uint16_t DistanceMode);

//     VL53L1X_SetInterMeasurementInMs(uint16_t dev, uint32_t InterMeasurementInMs);
    
//     // what is interrupt polarity?
    




//     // do we manage ranging?
//     // can we do it in parallel?   start --> return to cont. --> stop --> read --> return with data
//     VL53L1X_StartRanging(uint16_t dev);
//     VL53L1X_StopRanging(uint16_t dev);


    


//     // // verify device
//     // uint8_t buf[1];
//     // i2c_read(lidar->slave_addr, WHO_AM_I_REG, buf, 1);
//     // if (buf[0] != DEVICE_ID) {
//     //     // wrong device: throw an error
//     //     return;
//     // }

//     // calibration vals (RefSPAD, offset, crosstalk)

// }

// void lidar_read(LIDAR_t *lidar) {
    
//     // VL53L1X_GetDistance(uint16_t dev, uint16_t *distance);
//     // VL53L1X_GetRangeStatus(uint16_t dev, uint8_t *rangeStatus);// if >0, don't trust distance just read?

//     VL53L1X_GetResult(uint16_t dev, VL53L1X_Result_t *pResult); // single shot

//     // update LIDAR_t with data from VL53L1X_Result_t with what we care about?
// }

// // Clear interrupt flag after reading
// void lidar_clear() {

//     VL53L1X_ClearInterrupt(uint16_t dev);
// }


// uint8_t lidar_ready() {
//     uint8_t *isDataReady;
//     VL53L1X_CheckForDataReady(dev, *isDataReady);

//     return &isDataReady;
// }



// void lidar_read_range(LIDAR_t *lidar) {}
// void lidar_read_sigma(LIDAR_t *lidar) {}
// void lidar_read_status(LIDAR_t *lidar) {}
// void lidar_read_strength(LIDAR_t *lidar) {}
// void lidar_read_interference(LIDAR_t *lidar) {}


// // #else //use FAKE hardware
// // void lidar_init(void) { /* do nothing */ }
// // void lidar_read_range(void) { /* do nothing */ }
// // void lidar_read_sigma(void) { /* do nothing */ }
// // void lidar_read_status(void) { /* do nothing */ }
// // void lidar_read_strength(void) { /* do nothing */ }
// // void lidar_read_interference(void) { /* do nothing */ }
// // #endif
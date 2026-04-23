/**
LiDAR driver

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


lidar.c --> VL53L1X_api.c --> vl53l1x_platform.c --> i2c.c

*/

#include "config.h"
#include "lidar.h"
#include "i2c.h" // remove?
#include "VL53L1X_api.h"

// define device
#define LIDAR_ADDR      0x29        // I2C slave address, 0x52 is the 8 bit representation
#define WHO_AM_I_REG    0x010F
#define LIDAR_ID        0xEEAC      // I2C device id, id + type ––– or is it 0xEACC (datasheet) or 0xEEAC (api)?


// define output regs
// define config regs
// define config vals


VL53L1X_Result_t lidar_data;
VL53L1X_ERROR lidar_error;

// uint8_t Status;		/*!< ResultStatus */
// uint16_t Distance;	/*!< ResultDistance */
// uint16_t Ambient;	/*!< ResultAmbient */
// uint16_t SigPerSPAD;/*!< ResultSignalPerSPAD */
// uint16_t NumSPADs;	/*!< ResultNumSPADs */


volatile uint8_t lidar_ready = 0;


#if USE_LIDAR // use REAL hardware

bool lidar_init(VL53L1X_t* lidar) {
    //HAL_Delay(10);
    for (volatile int i = 0; i < 480000; i++);

    uint8_t boot_state = 0;
    while (!boot_state)
        VL53L1X_BootState(LIDAR_ADDR, &boot_state);

    // initialize
    VL53L1X_SensorInit(LIDAR_ADDR);
    //lidar_error = VL53L1X_SensorInit(LIDAR_ADDR); // need to check error?

    uint16_t device_id;
    VL53L1X_GetSensorId(LIDAR_ADDR, &device_id);
    if (device_id != LIDAR_ID) {
        // wrong device: throw an error
        return false;
    }

    VL53L1X_SetDistanceMode(LIDAR_ADDR, 0); // 0 for short mode (~1.3m)
    VL53L1X_SetTimingBudgetInMs(LIDAR_ADDR, 50); // 50ms (20Hz), maybe try 20ms (50Hz)
    VL53L1X_SetInterMeasurementInMs(LIDAR_ADDR, 50); // 50ms (20Hz) - must be longer than TimingBudgetInMs
    VL53L1X_StartRanging(LIDAR_ADDR);
    // these could throw errors and weren't checked

    // what is interrupt polarity?

    return true;
}

// Read all LiDAR data we care about
bool lidar_read(VL53L1X_t* lidar) {
    
    lidar_ready = 0; // set 0 in global?, let interrupt manage?

    while (!lidar_ready) {
        lidar_ready = VL53L1X_CheckForDataReady(LIDAR_ADDR, &lidar_ready);
    }

    VL53L1X_GetDistance(LIDAR_ADDR, &lidar->range_mm);
    //VL53L1X_GetSigma(LIDAR_ADDR, &lidar->sigma_mm);
    VL53L1X_GetRangeStatus(LIDAR_ADDR, &lidar->range_status);
    VL53L1X_GetSignalRate(LIDAR_ADDR, &lidar->strength);
    VL53L1X_GetAmbientRate(LIDAR_ADDR, &lidar->interference);

    VL53L1X_ClearInterrupt(LIDAR_ADDR);

    // check the things above for error
    // if (status != 0) {
    //     // invalid reading: throw some error
    // }

    return true;
}

bool lidar_read_range(VL53L1X_t* lidar) {
    return VL53L1X_GetDistance(LIDAR_ADDR, &lidar->range_mm) == 0;
}

// bool lidar_read_sigma(VL53L1X_t* lidar) {
//     return VL53L1X_GetSigma(LIDAR_ADDR, &lidar->sigma_mm) == 0;
// }

bool lidar_read_status(VL53L1X_t* lidar) {
    return VL53L1X_GetRangeStatus(LIDAR_ADDR, &lidar->range_status) == 0;
}

bool lidar_read_strength(VL53L1X_t* lidar) {
    return VL53L1X_GetSignalRate(LIDAR_ADDR, &lidar->strength) == 0;
}

bool lidar_read_interference(VL53L1X_t* lidar) {
    return VL53L1X_GetAmbientRate(LIDAR_ADDR, &lidar->interference) == 0;
} 

bool lidar_clear_interrupt(void) {
    return VL53L1X_ClearInterrupt(LIDAR_ADDR) == 0;
}


#else //use FAKE hardware
bool lidar_init(VL53L1X_t* lidar) { return true; }
bool lidar_read(VL53L1X_t* lidar) { return true; }
bool lidar_read_range(VL53L1X_t* lidar) { return true; }
bool lidar_read_sigma(VL53L1X_t* lidar) { return true; }
bool lidar_read_status(VL53L1X_t* lidar) { return true; }
bool lidar_read_strength(VL53L1X_t* lidar) { return true; }
bool lidar_read_interference(VL53L1X_t* lidar) { return true; }
bool lidar_clear_interrupt(void) { return true; }
#endif



//     // do we manage ranging?
//     // can we do it in parallel?   start --> return to cont. --> stop --> read --> return with data
//     VL53L1X_StartRanging(uint16_t dev);
//     VL53L1X_StopRanging(uint16_t dev);


    



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
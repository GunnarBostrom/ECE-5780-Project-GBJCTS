#include "imu.h"
#include "config_local.h"

#if USE_IMU
// Put your REAL hardware code here
void imu_init(void) { /* HAL_I2C_Init... */ }
void imu_read(void) { /* HAL_I2C_Receive... */ }

#else 
// Put your FAKE hardware code here (inside the same file!)
void imu_init(void) { /* do nothing */ }
void imu_read(void) { /* do nothing */ }
#endif
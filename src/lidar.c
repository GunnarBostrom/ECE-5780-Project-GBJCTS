#include "lidar.h"
#include "config_local.h"

#if USE_LIDAR
// Put your REAL hardware code here
void lidar_init(void) { /* HAL_I2C_Init... */ }
void lidar_read(void) { /* HAL_I2C_Receive... */ }

#else 
// Put your FAKE hardware code here (inside the same file!)
void lidar_init(void) { /* do nothing */ }
void lidar_read(void) { /* do nothing */ }
#endif
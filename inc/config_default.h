// This config file assumes that all peripherals are used unless overridden

#ifndef USE_RADIO
    #define USE_RADIO 1
#endif
#ifndef USE_IMU
    #define USE_IMU 1
#endif
#ifndef USE_LIDAR
    #define USE_LIDAR 1
#endif
#ifndef USE_MOTOR
    #define USE_MOTOR 1
#endif
#ifndef USE_DEBUGGER
    #define USE_DEBUGGER 1
#endif

// Route only one data-ready source to the control-loop interrupt by default.
//
// Enabling both accel and gyro DRDY on the same pin can create two interrupt
// pulses per sensor period, which breaks the fixed control-loop timestep.
#ifndef IMU_INT_USE_ACCEL_DRDY
    #define IMU_INT_USE_ACCEL_DRDY 0
#endif

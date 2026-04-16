#include "control.h"
#include "radio.h"
#include "motor.h"
#include "stm32f072xb.h"
#include <stdint.h>

#define THROTTLE_MIN 1000
#define THROTTLE_MAX 1900


void control_from_radio() {
    
    uint16_t throttle = 1000;
    if (radio_data.throttle < 230) throttle = 1000;
    else if (radio_data.throttle > 1750) throttle = 1900;
    else throttle = ((radio_data.throttle - 230) * (THROTTLE_MAX - THROTTLE_MIN)) / (1750 - 230) + THROTTLE_MIN;
    if (radio_data.armed) {
        // For simplicity, set all motors to the same throttle value
        motor_set_all(throttle);
    } else {
        // Disarmed, set motors to minimum
        motor_set_all(1000);
    }

}
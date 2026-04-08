#include "control.h"
#include "radio.h"
#include "motor.h"
#include <stdint.h>


void control_from_radio() {
    uint16_t throttle = radio_data.throttle > 992 ? 1200 : 1000; // simple threshold for throttle, can be improved with scaling and deadband

    if (radio_data.armed) {
        // For simplicity, set all motors to the same throttle value
        motor_set_all(throttle);
    } else {
        // Disarmed, set motors to minimum
        motor_set_all(1000);
    }

}
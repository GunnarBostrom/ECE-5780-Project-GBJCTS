#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>

typedef struct
{
    uint16_t throttle;
    uint8_t armed;
    uint8_t failsafe;
} Radio_t;

extern volatile Radio_t radio_data;

void radio_init(void);
void radio_read(void);

#endif // RADIO_H

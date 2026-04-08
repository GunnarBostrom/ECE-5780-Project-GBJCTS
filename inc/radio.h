#include <stdint.h>

void radio_init(void);
void radio_read(void);

typedef struct {
    uint16_t throttle;  // channel 1, range 172-1811
    uint8_t armed;      // channel 5, 0 or 1
} Radio_t;

extern volatile Radio_t radio_data;
extern volatile short int new_radio_data_flag;
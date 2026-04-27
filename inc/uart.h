#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init(void);
void uart_write_str(const char *str);
void uart_write_control_commands(float roll_cmd,
                                 float pitch_cmd,
                                 uint16_t m1,
                                 uint16_t m2,
                                 uint16_t m3,
                                 uint16_t m4);

#endif

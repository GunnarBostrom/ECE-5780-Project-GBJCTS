/*
Serial debugger to read out sensor values.

Set up on USART1
PA9 - STM32 TX
PA10 - STM32 RX
*/

#ifndef DEBUGGER_H
#define DEBUGGER_H

#include <stdint.h>

void debug_init(void);
void accept_character(uint8_t character);
void accept_string(const char *str);
void USART1_IRQHandler(void);

#endif

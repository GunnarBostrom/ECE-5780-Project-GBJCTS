#ifndef UART_TEST_DATA_H
#define UART_TEST_DATA_H

void uart_test_init(void);
void accept_character(uint8_t character);
void accept_string(const char *str);
void USART3_4_IRQHandler(void);

#endif // UART_TEST_DATA_H
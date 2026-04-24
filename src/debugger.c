/*
UART debugger to read out sensor values.

Set up on USART1
PA9 - STM32 TX -- gray -- Serial RX
PA10 - STM32 RX -- purple -- Serial TX
*/

#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "config.h"
#include "debugger.h"

volatile char usart_read_register;
volatile short int new_data = 0;

#if USE_DEBUGGER // use REAL hardware

void debug_init(void) {
    
  GPIO_InitTypeDef initStr = {0};
  initStr.Pin = GPIO_PIN_9 | GPIO_PIN_10; // PA9 - stm tx, PA10 - stm rx
  initStr.Mode = GPIO_MODE_AF_PP;
  initStr.Speed = GPIO_SPEED_FREQ_HIGH;
  initStr.Pull = GPIO_NOPULL;
  initStr.Alternate = GPIO_AF1_USART1;
  HAL_GPIO_Init(GPIOA, &initStr);

  HAL_RCC_GetHCLKFreq();
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  (void)RCC->APB2ENR;

  USART1->CR1 = 0;
  USART1->BRR = HAL_RCC_GetHCLKFreq() / 115200;
  USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
  USART1->CR1 |= USART_CR1_UE;

  NVIC_EnableIRQ(USART1_IRQn);
  NVIC_SetPriority(USART1_IRQn, 3U);

}

void accept_character(uint8_t character) {
  while (!(USART1->ISR & USART_ISR_TXE)) {} // wait
  USART1->TDR = character;                  // write
}

void accept_string(const char *str) {
  while (*str) {
    accept_character((uint8_t)*str);
    str++;
  }
}

void USART1_IRQHandler(void) {
  if (USART1->ISR & USART_ISR_RXNE) {
    usart_read_register = (char)USART1->RDR;
    new_data = 1;
  }
}

#else // use FAKE hardware
void debug_init(void) { /* do nothing */ }
void accept_character(uint8_t character) { /* do nothing */ }
void accept_string(const char *str) { /* do nothing */ }
void USART1_IRQHandler(void) { /* do nothing */ }
#endif
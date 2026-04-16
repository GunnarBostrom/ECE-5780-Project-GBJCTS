#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"

volatile char USART_read_test_register;
volatile short int new_data_test_flag = 0;


void uart_test_init(void) {
    
  GPIO_InitTypeDef initStr = {0};
  initStr.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  initStr.Mode = GPIO_MODE_AF_PP;      // Set to Alternate Function
  initStr.Speed = GPIO_SPEED_FREQ_HIGH;
  initStr.Pull = GPIO_NOPULL;
  initStr.Alternate = GPIO_AF1_USART1;   // Connects the pins to TIM3
  HAL_GPIO_Init(GPIOA, &initStr);

  HAL_RCC_GetHCLKFreq();
  // Enable USART3
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  (void)RCC->APB2ENR;

  USART1->CR1 = 0; // Reset USART3 control register
  USART1->BRR = HAL_RCC_GetHCLKFreq() / 115200; // Set baud rate to 115200
  USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // Enable Transmitter and Receiver
  USART1->CR1 |= USART_CR1_UE; // Enable USART3

  NVIC_EnableIRQ(USART1_IRQn);
  NVIC_SetPriority(USART1_IRQn, 3U);

}


void accept_character(uint8_t character)
{
  // Wait until the transmit data register is empty
  while (!(USART1->ISR & USART_ISR_TXE))
  {
  }
  // Write the character to the transmit data register
  USART1->TDR = character;
}

void accept_string(const char *str)
{
  while (*str)
  {
    accept_character((uint8_t)*str);
    str++;
  }
}


void USART1_IRQHandler(void)
{
  if (USART1->ISR & USART_ISR_RXNE)
  {
    USART_read_test_register = (char)USART1->RDR;
    new_data_test_flag = 1;
  }
}
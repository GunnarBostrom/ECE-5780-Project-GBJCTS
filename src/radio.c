#include "radio.h"
#include "config_local.h"
#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

volatile char USART_read_register;
volatile short int new_data_flag = 0;

#if USE_RADIO


void radio_init(void) { 


    // Enable GPIOC clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    (void)RCC->AHBENR;

    // Configure PC4 and PC5 for USART3 (TX and RX)
    GPIO_InitTypeDef initStr = {0};
    initStr.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    initStr.Mode = GPIO_MODE_AF_PP;      // Set to Alternate Function
    initStr.Speed = GPIO_SPEED_FREQ_HIGH;
    initStr.Pull = GPIO_NOPULL;
    initStr.Alternate = GPIO_AF1_USART3;   // Connects the pins to TIM3
    HAL_GPIO_Init(GPIOC, &initStr);

    // Configure PC6, PC7, PC8, and PC9 as outputs for debugging
    GPIO_InitTypeDef initStr2 = { GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7,
                                GPIO_MODE_OUTPUT_PP,
                                GPIO_SPEED_FREQ_LOW,
                                GPIO_NOPULL};
    HAL_GPIO_Init(GPIOC, &initStr2);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);

    // Enable USART3
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    (void)RCC->APB1ENR;

    USART3->CR1 = 0; // Reset USART3 control register
    USART3->BRR = HAL_RCC_GetHCLKFreq() / 416666; // Set baud rate to 115
    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // Enable Transmitter and Receiver and Interrupt on RXNE
    USART3->CR1 |= USART_CR1_UE; // Enable USART3

    // Enable USART3 interrupt in NVIC and set priority
    NVIC_EnableIRQ(USART3_4_IRQn);
    NVIC_SetPriority(USART3_4_IRQn, 0U);

}


void radio_read(void) { 
    if (new_data_flag) {
        // Process the received data in USART_read_register
        if(USART_read_register == 0xC8) {
            // Example: Toggle PC6 if 'A' is received
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
        } else {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7); // Toggle PC7 for any other character
        }

        // Clear the flag after processing
        new_data_flag = 0;
    }

 }
 /**
  * @brief This function handles USART3 and USART4 interrupt.
  */
void USART3_4_IRQHandler(void)
{
  if (USART3->ISR & USART_ISR_RXNE)
  {
    GPIOC->ODR ^= GPIO_ODR_8;           // Toggle orange LED


    USART_read_register = (char)USART3->RDR;
    new_data_flag = 1;
  }

    // ERROR CLEARING: If the UART gets too much data too fast, it locks up.
    // This clears the Overrun, Noise, and Framing error flags.
    if (USART3->ISR & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE)) {
        USART3->ICR |= (USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF);
    }
}

#else 
// Put your FAKE hardware code here (inside the same file!)
void radio_init(void) { /* do nothing */ }
void radio_read(void) { /* do nothing */ }
void USART3_4_IRQHandler(void) { /* do nothing */ }
#endif
#include "radio.h"
#include "config_local.h"
#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

volatile char USART_read_register;
volatile short int new_data_flag = 0;

volatile char radio_buffer[26];
volatile uint8_t radio_buffer_index = 0;
volatile Radio_t radio_data = {0};

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
    initStr.Alternate = GPIO_AF1_USART3;   // Set alternate function for USART3
    HAL_GPIO_Init(GPIOC, &initStr);


    // Enable USART3
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    (void)RCC->APB1ENR;

    USART3->CR1 = 0; // Reset USART3 control register
    // USART3->CR2 = 0; // Reset USART3 control register 2
    // USART3->CR2 |= USART_CR2_RXINV | USART_CR2_TXINV; // Invert RX and TX line (active low)
    USART3->BRR = HAL_RCC_GetHCLKFreq() / 420000; // Set baud rate to 115
    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // Enable Transmitter and Receiver and Interrupt on RXNE

    USART3->CR1 |= USART_CR1_UE; // Enable USART3

    // Enable USART3 interrupt in NVIC and set priority
    NVIC_EnableIRQ(USART3_4_IRQn);
    NVIC_SetPriority(USART3_4_IRQn, 0U);

}


#define CHANNEL_TO_TEST 1  // Change this 0-15 to test each channel

void radio_read(void) {
    if (new_data_flag) {
        new_data_flag = 0;

        const volatile uint8_t *p = (const volatile uint8_t *)&radio_buffer[3];
        if (radio_buffer[0] != 0xC8) {
            // Invalid packet, ignore
            return;
        }
        if (radio_buffer[1] == 0x00) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);   // red on for valid packet
        }

        uint16_t ch[16];
        ch[0]  = ((p[0])       | (p[1]  << 8))  & 0x07FF;
        ch[1]  = ((p[1]  >> 3) | (p[2]  << 5))  & 0x07FF;
        ch[2]  = ((p[2]  >> 6) | (p[3]  << 2) | (p[4] << 10)) & 0x07FF;
        ch[3]  = ((p[4]  >> 1) | (p[5]  << 7))  & 0x07FF;
        ch[4]  = ((p[5]  >> 4) | (p[6]  << 4))  & 0x07FF;
        ch[5]  = ((p[6]  >> 7) | (p[7]  << 1) | (p[8] << 9))  & 0x07FF;
        ch[6]  = ((p[8]  >> 2) | (p[9]  << 6))  & 0x07FF;
        ch[7]  = ((p[9]  >> 5) | (p[10] << 3))  & 0x07FF;
        ch[8]  = ((p[11])      | (p[12] << 8))  & 0x07FF;
        ch[9]  = ((p[12] >> 3) | (p[13] << 5))  & 0x07FF;
        ch[10] = ((p[13] >> 6) | (p[14] << 2) | (p[15] << 10)) & 0x07FF;
        ch[11] = ((p[15] >> 1) | (p[16] << 7))  & 0x07FF;
        ch[12] = ((p[16] >> 4) | (p[17] << 4))  & 0x07FF;
        ch[13] = ((p[17] >> 7) | (p[18] << 1) | (p[19] << 9)) & 0x07FF;
        ch[14] = ((p[19] >> 2) | (p[20] << 6))  & 0x07FF;
        ch[15] = ((p[20] >> 5) | (p[21] << 3))  & 0x07FF;

        // Test: orange LED = above center, blue LED = below center
        if (ch[CHANNEL_TO_TEST] > 992) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);   // orange on
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // blue off
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // orange off
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);   // blue on
        }
    }
}
 
 /**
  * @brief This function handles USART3 and USART4 interrupt.
  */
void USART3_4_IRQHandler(void)
{
    if (USART3->ISR & USART_ISR_RXNE)
    {
        
        USART_read_register = (char)USART3->RDR;
        
        if (USART_read_register == 0xC8 && radio_buffer_index == 0) {
            radio_buffer[0] = USART_read_register;
            radio_buffer_index = 1;
        } else if (radio_buffer_index == 1) {
            if (USART_read_register > 0 && USART_read_register <= 62) {
                radio_buffer[1] = USART_read_register;
                radio_buffer_index = 2;
                // store length so we know when packet ends
            } else {
                radio_buffer_index = 0;
            }
        } else if (radio_buffer_index > 1) {
            radio_buffer[radio_buffer_index] = USART_read_register;
            radio_buffer_index++;
            if (radio_buffer_index > 15) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);   // blue on
            }
            uint8_t packet_len = radio_buffer[1] + 2; // length byte + sync + payload
            if (radio_buffer_index >= packet_len) {
                radio_buffer_index = 0;
                if (radio_buffer[2] == 0x16) { // only flag RC channels packets
                    new_data_flag = 1;
                }
            }
        }
    }
}

#else 
// Put your FAKE hardware code here (inside the same file!)
void radio_init(void) { /* do nothing */ }
void radio_read(void) { /* do nothing */ }
void USART3_4_IRQHandler(void) { /* do nothing */ }
#endif
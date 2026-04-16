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
    USART3->BRR = HAL_RCC_GetHCLKFreq() / 420000; // Set baud rate to 420000
    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // Enable Transmitter and Receiver and Interrupt on RXNE

    USART3->CR1 |= USART_CR1_UE; // Enable USART3

    // Enable USART3 interrupt in NVIC and set priority 3
    NVIC_EnableIRQ(USART3_4_IRQn);
    NVIC_SetPriority(USART3_4_IRQn, 3U);

}


void radio_read(void) {
    if (new_data_flag) {
        new_data_flag = 0;

        const volatile uint8_t *p = (const volatile uint8_t *)&radio_buffer[3];

        uint16_t ch[16];
        ch[0]  = ((p[0])       | (p[1]  << 8))  & 0x07FF;
        //ch[1]  = ((p[1]  >> 3) | (p[2]  << 5))  & 0x07FF;
        // ch[2]  = ((p[2]  >> 6) | (p[3]  << 2) | (p[4] << 10)) & 0x07FF;
        // ch[3]  = ((p[4]  >> 1) | (p[5]  << 7))  & 0x07FF;
        ch[4]  = ((p[5]  >> 4) | (p[6]  << 4))  & 0x07FF;
        // ch[5]  = ((p[6]  >> 7) | (p[7]  << 1) | (p[8] << 9))  & 0x07FF;
        // ch[6]  = ((p[8]  >> 2) | (p[9]  << 6))  & 0x07FF;
        // ch[7]  = ((p[9]  >> 5) | (p[10] << 3))  & 0x07FF;
        // ch[8]  = ((p[11])      | (p[12] << 8))  & 0x07FF;
        // ch[9]  = ((p[12] >> 3) | (p[13] << 5))  & 0x07FF;
        // ch[10] = ((p[13] >> 6) | (p[14] << 2) | (p[15] << 10)) & 0x07FF;
        // ch[11] = ((p[15] >> 1) | (p[16] << 7))  & 0x07FF;
        // ch[12] = ((p[16] >> 4) | (p[17] << 4))  & 0x07FF;
        // ch[13] = ((p[17] >> 7) | (p[18] << 1) | (p[19] << 9)) & 0x07FF;
        // ch[14] = ((p[19] >> 2) | (p[20] << 6))  & 0x07FF;
        // ch[15] = ((p[20] >> 5) | (p[21] << 3))  & 0x07FF;

        radio_data.throttle = ch[0]; // channel 1 is throttle
        radio_data.armed = (ch[4] > 992) ? 1 : 0; // channel 5 above center is armed, below center is disarmed
    } else {
        // lost signal? set to safe values
        radio_data.throttle = 50;
        radio_data.armed = 0;
    }

}
 
 /**
  * @brief This function handles USART3 and USART4 interrupt.
  */
#define CRSF_SYNC       0xC8
#define CRSF_FRAME_SIZE 26  // sync + length + type + 22 payload + 1 CRC

void USART3_4_IRQHandler(void)
{
    if (!(USART3->ISR & USART_ISR_RXNE)) return;

    // Clear error flags if set
    if (USART3->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;
        return;
    }

    uint8_t byte = USART3->RDR;

    // Reset on sync byte
    if (byte == CRSF_SYNC) {
        radio_buffer_index = 0;
    }

    // Store byte if within frame
    if (radio_buffer_index < CRSF_FRAME_SIZE) {
        radio_buffer[radio_buffer_index++] = byte;
    }

    // Complete frame received
    if (radio_buffer_index == CRSF_FRAME_SIZE) {
        radio_buffer_index = 0;
        new_data_flag = 1;
    }
}

#else 
// Put your FAKE hardware code here (inside the same file!)
void radio_init(void) { /* do nothing */ }
void radio_read(void) { /* do nothing */ }
void USART3_4_IRQHandler(void) { /* do nothing */ }
#endif
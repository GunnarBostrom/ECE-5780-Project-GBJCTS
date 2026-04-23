#include "radio.h"
#include "config_local.h"
#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define RADIO_FAILSAFE_MS 250U

volatile char USART_read_register;
volatile short int new_data_flag = 0;

volatile char radio_buffer[26];
volatile uint8_t radio_buffer_index = 0;
volatile Radio_t radio_data = {0};

#if USE_RADIO

void radio_init(void)
{
    // Enable GPIOC clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    (void)RCC->AHBENR;

    // Configure PC4 and PC5 for USART3 (TX and RX)
    GPIO_InitTypeDef initStr = {0};
    initStr.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    initStr.Mode = GPIO_MODE_AF_PP;
    initStr.Speed = GPIO_SPEED_FREQ_HIGH;
    initStr.Pull = GPIO_NOPULL;
    initStr.Alternate = GPIO_AF1_USART3;
    HAL_GPIO_Init(GPIOC, &initStr);

    // Enable USART3
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    (void)RCC->APB1ENR;

    USART3->CR1 = 0;
    USART3->BRR = HAL_RCC_GetHCLKFreq() / 420000;
    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART3->CR1 |= USART_CR1_UE;

    NVIC_EnableIRQ(USART3_4_IRQn);
    NVIC_SetPriority(USART3_4_IRQn, 3U);
}

void radio_read(void)
{
    static uint32_t last_frame_ms = 0;

    if (new_data_flag)
    {
        new_data_flag = 0;
        last_frame_ms = HAL_GetTick();

        const volatile uint8_t *p = (const volatile uint8_t *)&radio_buffer[3];

        uint16_t ch[16];
        ch[0] = ((p[0]) | (p[1] << 8)) & 0x07FF;
        ch[4] = ((p[5] >> 4) | (p[6] << 4)) & 0x07FF;

        radio_data.throttle = ch[0];
        radio_data.armed = (ch[4] > 992) ? 1 : 0;
        radio_data.failsafe = 0;
    }
    else if ((HAL_GetTick() - last_frame_ms) > RADIO_FAILSAFE_MS)
    {
        radio_data.throttle = 0;
        radio_data.armed = 0;
        radio_data.failsafe = 1;
    }
}

#define CRSF_SYNC       0xC8
#define CRSF_FRAME_SIZE 26

void USART3_4_IRQHandler(void)
{
    if (!(USART3->ISR & USART_ISR_RXNE))
    {
        return;
    }

    if (USART3->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE))
    {
        USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;
        return;
    }

    uint8_t byte = USART3->RDR;

    if (byte == CRSF_SYNC)
    {
        radio_buffer_index = 0;
    }

    if (radio_buffer_index < CRSF_FRAME_SIZE)
    {
        radio_buffer[radio_buffer_index++] = byte;
    }

    if (radio_buffer_index == CRSF_FRAME_SIZE)
    {
        radio_buffer_index = 0;
        new_data_flag = 1;
    }
}

#else
void radio_init(void) { /* do nothing */ }
void radio_read(void) { /* do nothing */ }
void USART3_4_IRQHandler(void) { /* do nothing */ }
#endif

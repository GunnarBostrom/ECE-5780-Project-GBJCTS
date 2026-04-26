#include "radio.h"
#include "config.h"
#include "stm32f0xx_hal.h"
#include <stdint.h>

#define RADIO_FAILSAFE_MS 250U
#define CRSF_SYNC 0xC8
#define CRSF_FRAME_SIZE 26
#define CRSF_LENGTH_RC_CHANNELS 24U
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16U

volatile char USART_read_register;
volatile short int new_data_flag = 0;

volatile char radio_buffer[26];
volatile uint8_t radio_buffer_index = 0;
volatile Radio_t radio_data = {0};

#if USE_RADIO

static void decode_crsf_channels(const volatile uint8_t *payload, uint16_t *channels)
{
    channels[0]  = ((payload[0]      | (payload[1]  << 8))) & 0x07FF;
    channels[1]  = (((payload[1] >> 3) | (payload[2]  << 5))) & 0x07FF;
    channels[2]  = (((payload[2] >> 6) | (payload[3]  << 2) | (payload[4]  << 10))) & 0x07FF;
    channels[3]  = (((payload[4] >> 1) | (payload[5]  << 7))) & 0x07FF;
    channels[4]  = (((payload[5] >> 4) | (payload[6]  << 4))) & 0x07FF;
    channels[5]  = (((payload[6] >> 7) | (payload[7]  << 1) | (payload[8]  << 9))) & 0x07FF;
    channels[6]  = (((payload[8] >> 2) | (payload[9]  << 6))) & 0x07FF;
    channels[7]  = (((payload[9] >> 5) | (payload[10] << 3))) & 0x07FF;
    channels[8]  = ((payload[11]      | (payload[12] << 8))) & 0x07FF;
    channels[9]  = (((payload[12] >> 3) | (payload[13] << 5))) & 0x07FF;
    channels[10] = (((payload[13] >> 6) | (payload[14] << 2) | (payload[15] << 10))) & 0x07FF;
    channels[11] = (((payload[15] >> 1) | (payload[16] << 7))) & 0x07FF;
    channels[12] = (((payload[16] >> 4) | (payload[17] << 4))) & 0x07FF;
    channels[13] = (((payload[17] >> 7) | (payload[18] << 1) | (payload[19] << 9))) & 0x07FF;
    channels[14] = (((payload[19] >> 2) | (payload[20] << 6))) & 0x07FF;
    channels[15] = (((payload[20] >> 5) | (payload[21] << 3))) & 0x07FF;
}

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

        if (((uint8_t)radio_buffer[0] == CRSF_SYNC) &&
            ((uint8_t)radio_buffer[1] == CRSF_LENGTH_RC_CHANNELS) &&
            ((uint8_t)radio_buffer[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED))
        {
            const volatile uint8_t *payload = (const volatile uint8_t *)&radio_buffer[3];
            uint16_t channels[16];

            decode_crsf_channels(payload, channels);

            last_frame_ms = HAL_GetTick();
            radio_data.throttle = channels[0];
            radio_data.armed = (channels[4] > 992U) ? 1U : 0U;
            radio_data.failsafe = 0;
        }
    }
    else if ((HAL_GetTick() - last_frame_ms) > RADIO_FAILSAFE_MS)
    {
        radio_data.throttle = 0;
        radio_data.armed = 0;
        radio_data.failsafe = 1;
    }
}

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

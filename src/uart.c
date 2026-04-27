#include "uart.h"

#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"

#include <stdint.h>
#include <stdio.h>

#define UART_DEBUG_BAUD 115200U

void uart_init(void)
{
    GPIO_InitTypeDef init = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    init.Pin = GPIO_PIN_9;
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    init.Alternate = GPIO_AF1_USART1;
    HAL_GPIO_Init(GPIOA, &init);

    USART1->CR1 = 0;
    USART1->BRR = HAL_RCC_GetHCLKFreq() / UART_DEBUG_BAUD;
    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_UE;
}

void uart_write_str(const char *str)
{
    while (*str != '\0')
    {
        while ((USART1->ISR & USART_ISR_TXE) == 0U)
        {
        }

        USART1->TDR = (uint8_t)(*str);
        str++;
    }

    while ((USART1->ISR & USART_ISR_TC) == 0U)
    {
    }
}

void uart_write_control_commands(float roll_cmd,
                                 float pitch_cmd,
                                 uint16_t m1,
                                 uint16_t m2,
                                 uint16_t m3,
                                 uint16_t m4)
{
    char buf[80];
    int32_t roll_cmd_tenths = (int32_t)(roll_cmd * 10.0f);
    int32_t pitch_cmd_tenths = (int32_t)(pitch_cmd * 10.0f);

    snprintf(buf,
             sizeof(buf),
             "roll=%ld pitch=%ld m1=%u m2=%u m3=%u m4=%u\r\n",
             (long)roll_cmd_tenths,
             (long)pitch_cmd_tenths,
             (unsigned)m1,
             (unsigned)m2,
             (unsigned)m3,
             (unsigned)m4);

    uart_write_str(buf);
}

/**
 * Flight Controller
 * STM32F072x Discovery Board
 */

#include "main.h"
#include "config.h"
#include "control.h"
#include "filter.h"
#include "i2c.h"
#include "imu.h"
#include "lidar.h"
#include "motor.h"
#include "radio.h"
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_it.h"
#include <stdint.h>

void Error_Handler(void);
void SystemClock_Config(void);

static void LED_init(void);
static void imu_interrupt_init(void);

static LSM6DS3_t imu;
static LIDAR_t vl53l1x;
static Attitude_t attitude;

int main(void)
{
    const float dt = 1.0f / 416.0f;

    HAL_Init();
    SystemClock_Config();

    LED_init();
    i2c_init(400);
    imu_interrupt_init();

    if (!imu_init(&imu))
    {
        Error_Handler();
    }

    if (!imu_calibrate_gyro(&imu, 512U))
    {
        Error_Handler();
    }

    lidar_init(&vl53l1x, 0x52);
    radio_init();
    motor_init();

    filter_init(&attitude);
    control_init(dt);
    motor_set_all(1000);

    while (1)
    {
        radio_read();

        if (imu_ready)
        {
            imu_ready = 0;

            imu_read(&imu);

            filter_update(&attitude,
                          imu.gx_dps,
                          imu.gy_dps,
                          imu.gz_dps,
                          imu.ax_g,
                          imu.ay_g,
                          imu.az_g,
                          dt);

            control_update(&imu, &attitude);
        }
    }
}

static void LED_init(void)
{
    GPIO_InitTypeDef initStr = {0};

    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    (void)RCC->AHBENR;

    initStr.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    initStr.Mode = GPIO_MODE_OUTPUT_PP;
    initStr.Speed = GPIO_SPEED_FREQ_LOW;
    initStr.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &initStr);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
}

static void imu_interrupt_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);

    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC;

    EXTI->IMR |= EXTI_IMR_MR0;
    EXTI->RTSR |= EXTI_RTSR_TR0;
    EXTI->FTSR &= ~EXTI_FTSR_TR0;

    NVIC_SetPriority(EXTI0_1_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
        for (volatile uint32_t delay = 0; delay < 200000U; ++delay)
        {
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        static uint16_t imu_led_divider = 0;

        imu_ready = 1;

        if (++imu_led_divider >= 208U)
        {
            imu_led_divider = 0;
            GPIOC->ODR ^= GPIO_ODR_8;
        }
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif

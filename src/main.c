/**
 * Flight Controller
 * STM32F072x Discovery Board
 */

#include "main.h"
#include "config.h"
#include "control.h"
#include "debugger.h"
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
#include <stdio.h>
#include <stdint.h>

void Error_Handler(void);
void SystemClock_Config(void);

static void LED_init(void);
static void debug_send_attitude(const Attitude_t* attitude);
static void imu_interrupt_init(void);
static void busy_delay(volatile uint32_t cycles);

static LSM6DS3_t imu;
static LIDAR_t vl53l1x;
static Attitude_t attitude;

int main(void)
{
    const float dt = 1.0f / 416.0f;

    HAL_Init();
    SystemClock_Config();

    LED_init();
    debug_init();
    accept_string("debug boot\r\n");
    i2c_init(400);
    imu_interrupt_init();

    if (!imu_init(&imu))
    {
        Error_Handler();
    }
    accept_string("imu init ok\r\n");

    if (!imu_calibrate_gyro(&imu, 512U))
    {
        Error_Handler();
    }
    accept_string("gyro cal ok\r\n");

    lidar_init(&vl53l1x, 0x52);
    radio_init();
    motor_init();

    filter_init(&attitude);
    control_init(dt);
    motor_set_all(1000);

    while (1)
    {
        radio_read();

        if (imu_data_ready())
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

            debug_send_attitude(&attitude);
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

static void debug_send_attitude(const Attitude_t* attitude)
{
#if USE_DEBUGGER
    static uint16_t debug_divider = 0U;
    char msg[96];
    MotorMixDebug_t mix;
    int32_t roll_cdeg;
    int32_t pitch_cdeg;
    char roll_sign;
    char pitch_sign;
    uint32_t roll_abs_cdeg;
    uint32_t pitch_abs_cdeg;

    if (++debug_divider < 83U)
    {
        return;
    }

    debug_divider = 0U;

    roll_cdeg = (attitude->roll_deg >= 0.0f) ?
                (int32_t)(attitude->roll_deg * 100.0f + 0.5f) :
                (int32_t)(attitude->roll_deg * 100.0f - 0.5f);
    pitch_cdeg = (attitude->pitch_deg >= 0.0f) ?
                 (int32_t)(attitude->pitch_deg * 100.0f + 0.5f) :
                 (int32_t)(attitude->pitch_deg * 100.0f - 0.5f);

    roll_sign = (roll_cdeg < 0) ? '-' : '+';
    pitch_sign = (pitch_cdeg < 0) ? '-' : '+';
    roll_abs_cdeg = (roll_cdeg < 0) ? (uint32_t)(-roll_cdeg) : (uint32_t)roll_cdeg;
    pitch_abs_cdeg = (pitch_cdeg < 0) ? (uint32_t)(-pitch_cdeg) : (uint32_t)pitch_cdeg;
    mix = control_get_last_mix();

    snprintf(msg,
             sizeof(msg),
             "roll=%c%lu.%02lu pitch=%c%lu.%02lu m=[%u %u %u %u]\r\n",
             roll_sign,
             (unsigned long)(roll_abs_cdeg / 100U),
             (unsigned long)(roll_abs_cdeg % 100U),
             pitch_sign,
             (unsigned long)(pitch_abs_cdeg / 100U),
             (unsigned long)(pitch_abs_cdeg % 100U),
             (unsigned int)mix.m1,
             (unsigned int)mix.m2,
             (unsigned int)mix.m3,
             (unsigned int)mix.m4);
    accept_string(msg);
#else
    (void)attitude;
#endif
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
        uint8_t blink_count = 1U;

        if (imu_error_code == IMU_ERROR_CAL_TIMEOUT)
        {
            blink_count = 2U;
        }
        else if (imu_error_code == IMU_ERROR_CAL_MOTION)
        {
            blink_count = 3U;
        }

        for (uint8_t blink_index = 0; blink_index < blink_count; ++blink_index)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
            busy_delay(4000000U);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
            busy_delay(4000000U);
        }

        busy_delay(12000000U);
    }
}

static void busy_delay(volatile uint32_t cycles)
{
    while (cycles-- > 0U)
    {
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

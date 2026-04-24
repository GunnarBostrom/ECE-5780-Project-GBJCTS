/**
 * Flight Controller
 * STM32F072x Discovery Board
 *
 * LED pinout:
 *   PC6 - red
 *   PC7 - blue
 *   PC8 - orange  (control-loop heartbeat blink)
 *   PC9 - green
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
#include <stdint.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* Forward declarations                                                 */
/* ------------------------------------------------------------------ */
void Error_Handler(void);
void SystemClock_Config(void);
static void LED_init(void);
void imu_interrupt_init(void);
static void print_imu(const LSM6DS3_t *imu);

/* ------------------------------------------------------------------ */
/* Globals                                                              */
/* ------------------------------------------------------------------ */
LSM6DS3_t  imu;
LIDAR_t    vl53l1x;
Attitude_t attitude;

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    LED_init();

    /* i2c_init already calls i2c_bus_reset() internally (9-clock recovery) */
    i2c_init(400);

    /* IMU — verify WHO_AM_I, configure ODR/FS, enable INT1 */
    if (!imu_init(&imu)) {
        Error_Handler();
    }
    imu_interrupt_init();   /* arm EXTI *after* IMU is configured */

    /* LiDAR */
    lidar_init(&vl53l1x, 0x52);

    /* UART debug output — 115200 baud on PA9/PA10 */
    debug_init();
    accept_string("\r\nFlight controller booted.\r\n");

    /* Radio */
    radio_init();

    /* Motors — hold ESCs at minimum on boot */
    motor_init();
    motor_set_all(1000);

    /* Estimation / control */
    const float dt = 1.0f / 416.0f;
    filter_init(&attitude);
    control_init(dt);

    uint32_t last_print_ms = 0;

    /* ---------------------------------------------------------------- */
    /* Main loop                                                         */
    /* ---------------------------------------------------------------- */
    while (1)
    {
        radio_read();

        if (imu_ready)
        {
            imu_ready = 0;

            imu_read(&imu);

            /*
             * imu_convert_units() stores integer mg and mdps.
             * Convert to float g and dps for the complementary filter.
             */
            float ax_g   =  (float)imu.ax_mg   / 1000.0f;
            float ay_g   =  (float)imu.ay_mg   / 1000.0f;
            float az_g   =  (float)imu.az_mg   / 1000.0f;
            float gx_dps =  (float)imu.gx_mdps / 1000.0f;
            float gy_dps =  (float)imu.gy_mdps / 1000.0f;
            float gz_dps =  (float)imu.gz_mdps / 1000.0f;

            filter_update(&attitude,
                          gx_dps, gy_dps, gz_dps,
                          ax_g,   ay_g,   az_g,
                          dt);

            control_update((const IMU_t *)&imu, &attitude);
        }

        /* Print one IMU batch every 3 seconds */
        if ((HAL_GetTick() - last_print_ms) >= 3000U)
        {
            last_print_ms = HAL_GetTick();
            print_imu(&imu);
        }
    }
}

/* ------------------------------------------------------------------ */
/* print_imu — formatted UART output for PuTTY                         */
/* ------------------------------------------------------------------ */
static void print_imu(const LSM6DS3_t *d)
{
    char buf[128];

    accept_string("\r\n=== IMU Data ===\r\n");

    sprintf(buf, "Accel  (mg):   ax=%7ld  ay=%7ld  az=%7ld\r\n",
            (long)d->ax_mg, (long)d->ay_mg, (long)d->az_mg);
    accept_string(buf);

    sprintf(buf, "Gyro  (mdps):  gx=%7ld  gy=%7ld  gz=%7ld\r\n",
            (long)d->gx_mdps, (long)d->gy_mdps, (long)d->gz_mdps);
    accept_string(buf);

    sprintf(buf, "Raw accel:     ax=%7d  ay=%7d  az=%7d\r\n",
            d->ax, d->ay, d->az);
    accept_string(buf);

    sprintf(buf, "Raw gyro:      gx=%7d  gy=%7d  gz=%7d\r\n",
            d->gx, d->gy, d->gz);
    accept_string(buf);

    accept_string("================\r\n");
}

/* ------------------------------------------------------------------ */
/* LED init — brief startup blink so you know the board booted         */
/* ------------------------------------------------------------------ */
static void LED_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    (void)RCC->AHBENR;

    GPIO_InitTypeDef initStr = {
        GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
        GPIO_MODE_OUTPUT_PP,
        GPIO_SPEED_FREQ_LOW,
        GPIO_NOPULL
    };
    HAL_GPIO_Init(GPIOC, &initStr);

    /* All on */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(500);
    /* Chase off one by one */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); HAL_Delay(150);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7); HAL_Delay(150);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); HAL_Delay(150);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); HAL_Delay(150);
    /* All off */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);
}

/* ------------------------------------------------------------------ */
/* IMU data-ready interrupt — PC0, rising edge                          */
/* ------------------------------------------------------------------ */
void imu_interrupt_init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);

    __HAL_RCC_SYSCFG_CLK_ENABLE();

    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI0_PC;

    EXTI->IMR  |=  EXTI_IMR_MR0;
    EXTI->RTSR |=  EXTI_RTSR_TR0;
    EXTI->FTSR &= ~EXTI_FTSR_TR0;

    NVIC_SetPriority(EXTI0_1_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/* ------------------------------------------------------------------ */
/* EXTI callback — sets imu_ready flag, blinks orange at ~2 Hz         */
/* ------------------------------------------------------------------ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        imu_ready = 1;

        static uint16_t divider = 0;
        if (++divider >= 208)   /* 416 Hz / 208 = 2 Hz toggle -> 1 Hz blink */
        {
            divider = 0;
            GPIOC->ODR ^= GPIO_ODR_8;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Clock — 48 MHz via HSI PLL                                           */
/* ------------------------------------------------------------------ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLMUL          = RCC_PLL_MUL6;
    osc.PLL.PREDIV          = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) { Error_Handler(); }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }
}

/* ------------------------------------------------------------------ */
/* Error handler                                                        */
/* ------------------------------------------------------------------ */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif

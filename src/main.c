/**
 * Flight Controller — STM32F072RB Discovery Board
 *
 * LED assignments:
 *   PC6 - red   (error indicator)
 *   PC7 - blue
 *   PC8 - orange (IMU heartbeat, toggles at ~1 Hz via EXTI ISR)
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
static void imu_interrupt_init(void);
static void print_imu(const IMU_t *imu, const Attitude_t *att);
static void busy_delay(volatile uint32_t cycles);

/* ------------------------------------------------------------------ */
/* Globals                                                              */
/* ------------------------------------------------------------------ */
static IMU_t      imu;
static LIDAR_t    vl53l1x;
static Attitude_t attitude;

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    LED_init();
    debug_init();
    accept_string("\r\nFlight controller booting...\r\n");

    i2c_init(400);

    /* IMU — verify WHO_AM_I, configure ODR/FS, enable INT1 */
    if (!imu_init(&imu, 0x6B))
    {
        Error_Handler();
    }
    accept_string("IMU init OK\r\n");

    /* Gyro bias calibration — keep drone still for ~1.5 s */
    accept_string("Calibrating gyro (keep still)...\r\n");
    imu_interrupt_init();   /* arm EXTI so calibration can use imu_ready flag */
    if (!imu_calibrate_gyro(&imu, 512U))
    {
        imu.gx_bias = 0;
        imu.gy_bias = 0;
        imu.gz_bias = 0;
        accept_string("Gyro cal warning, continuing with zero bias\r\n");
    }
    else
    {
        accept_string("Gyro cal OK\r\n");
    }

    /* LiDAR */
    lidar_init(&vl53l1x, 0x52);

    /* Radio */
    radio_init();

    /* Motors — hold ESCs at minimum on boot */
    motor_init();
    motor_set_all(1000);

    /* Attitude estimation + control */
    filter_init(&attitude);
    control_init();

    uint32_t last_print_ms = 0;

    /* ---------------------------------------------------------------- */
    /* Main loop — runs at up to 416 Hz, gated by IMU data-ready        */
    /* ---------------------------------------------------------------- */
    while (1)
    {
        radio_read();

        if (imu_data_ready())
        {
            imu_ready = 0;

            imu_read(&imu);

            /* All values already in integer mg / mdps — no float needed */
            filter_update(&attitude,
                          imu.gx_mdps, imu.gy_mdps,
                          imu.ax_mg,   imu.ay_mg, imu.az_mg);

            control_update(&imu, &attitude);
        }

        /* Print IMU + attitude over UART every 3 s */
        if ((HAL_GetTick() - last_print_ms) >= 3000U)
        {
            last_print_ms = HAL_GetTick();
            print_imu(&imu, &attitude);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Debug UART output                                                    */
/* ------------------------------------------------------------------ */
static void print_imu(const IMU_t *d, const Attitude_t *att)
{
    char buf[128];

    accept_string("\r\n=== IMU / Attitude ===\r\n");

    snprintf(buf, sizeof(buf),
             "Accel  (mg):  ax=%7ld  ay=%7ld  az=%7ld\r\n",
             (long)d->ax_mg, (long)d->ay_mg, (long)d->az_mg);
    accept_string(buf);

    snprintf(buf, sizeof(buf),
             "Gyro  (mdps): gx=%7ld  gy=%7ld  gz=%7ld\r\n",
             (long)d->gx_mdps, (long)d->gy_mdps, (long)d->gz_mdps);
    accept_string(buf);

    snprintf(buf, sizeof(buf),
             "Gyro bias:    bx=%6d   by=%6d   bz=%6d\r\n",
             (int)d->gx_bias, (int)d->gy_bias, (int)d->gz_bias);
    accept_string(buf);

    /* Print attitude as signed X.XX degrees */
    int32_t r = att->roll_cdeg;
    int32_t p = att->pitch_cdeg;
    snprintf(buf, sizeof(buf),
             "Attitude: roll=%c%ld.%02ld  pitch=%c%ld.%02ld\r\n",
             (r < 0 ? '-' : '+'), (long)(r < 0 ? -r : r) / 100,
                                  (long)(r < 0 ? -r : r) % 100,
             (p < 0 ? '-' : '+'), (long)(p < 0 ? -p : p) / 100,
                                  (long)(p < 0 ? -p : p) % 100);
    accept_string(buf);

    accept_string("======================\r\n");
}

/* ------------------------------------------------------------------ */
/* LED init                                                             */
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

    /* Chase pattern to confirm boot */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); HAL_Delay(150);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7); HAL_Delay(150);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); HAL_Delay(150);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); HAL_Delay(150);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);
}

/* ------------------------------------------------------------------ */
/* IMU data-ready interrupt — PC0, rising edge                          */
/* ------------------------------------------------------------------ */
static void imu_interrupt_init(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* Belt-and-suspenders: explicitly route EXTI line 0 to port C.
     * HAL_GPIO_Init should handle this, but SYSCFG can be misconfigured
     * after a warm reset without a full power cycle. */
    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0)
                        | SYSCFG_EXTICR1_EXTI0_PC;

    /* Clear any pending flag that accumulated before the EXTI was armed */
    EXTI->PR = EXTI_PR_PR0;

    NVIC_SetPriority(EXTI0_1_IRQn, 3);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/* ------------------------------------------------------------------ */
/* EXTI ISR — sets imu_ready, blinks orange LED at ~1 Hz               */
/* CRITICAL: EXTI->PR must be cleared (W1C) before returning or the
 * Cortex-M0 tail-chains straight back into this ISR forever.          */
/* ------------------------------------------------------------------ */
void EXTI0_1_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0)
    {
        EXTI->PR = EXTI_PR_PR0;   /* W1C — must clear before returning or ISR re-fires */
        imu_ready = 1;

        static uint16_t divider = 0;
        if (++divider >= 208)     /* 416 Hz / 208 = 2 Hz toggle -> 1 Hz blink */
        {
            divider = 0;
            GPIOC->ODR ^= GPIO_ODR_8;  /* orange LED heartbeat */
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
/* Error handler — blink red LED; blink count indicates error code      */
/* ------------------------------------------------------------------ */
void Error_Handler(void)
{
    __disable_irq();

    uint8_t blink_count = 1U;
    if      (imu_error_code == IMU_ERROR_CAL_TIMEOUT) blink_count = 2U;
    else if (imu_error_code == IMU_ERROR_CAL_MOTION)  blink_count = 3U;

    while (1)
    {
        for (uint8_t i = 0; i < blink_count; ++i)
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
    while (cycles-- > 0U) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif

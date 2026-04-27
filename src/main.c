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
#include "pid.h"
#include "radio.h"
#include "uart.h"
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_it.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/_intsup.h>
#include <sys/types.h>
#include "imu.h"
#include "filter.h"
#include "control.h"
#include "radio.h"
#include "motor.h"

void Error_Handler(void);
void SystemClock_Config(void);

static void LED_init(void);
void imu_interrupt_init(void);


/* –––––––––– globals –––––––––– */
LSM6DS3_t imu;
LIDAR_t vl53l1x;
Attitude_t attitude;

int main(void) {

    // Core HAL and clock setup
    HAL_Init();
    SystemClock_Config();

    // Optional LED setup for heartbeat / debug indication
    LED_init();

    // Dedicated debug UART on USART1 TX (PA9)
    uart_init();

    // Peripheral initialization

    // Initialize I2C bus at 400 kHz for IMU and lidar communication
    i2c_init(400);

    // Initialize IMU, then enable its interrupt so new samples can trigger updates
    imu_init(&imu);
    imu_interrupt_init();

    // Initialize lidar sensor
    lidar_init(&vl53l1x, 0x52);    // VL53L1X I2C address = 0x52

    // Initialize radio receiver interface
    radio_init();

    // Initialize ESC / motor PWM outputs
    motor_init();

    // Estimation and control initialization

    // Fixed timestep used by the filter and controller
    //
    // This assumes the IMU data-ready interrupt occurs at 416 Hz and that each
    // interrupt corresponds to one fresh sensor sample
    //
    // Because the control loop only runs when imu_ready is asserted, this is far
    // better than using the same dt inside a free-running while loop. Motor
    // outputs are refreshed independently by TIM2 at a higher ESC update rate.
    const float dt = 1.0f / 416.0f;

    // Initialize attitude estimate state
    filter_init(&attitude);

    // Initialize roll/pitch controllers using the same fixed timestep
    control_init(dt);

    // Start with motors at minimum command for safety
    motor_set_all(1000);

    /*
     * CONTROL LOOP STRUCTURE
     *
     * 1. Wait for a new IMU sample
     * 2. Read IMU data
     * 3. Update roll/pitch estimate
     * 4. Run controller
     * 5. Update motors
     *
     * The loop is event-driven by the IMU interrupt rather than running
     * continuously at full speed.
     */
    while (1)
    {
        // Read and process radio input in the background
        //
        // This keeps the latest throttle / arm state updated even while waiting
        // for the next IMU sample.
        radio_read();

        // Run one control update per fresh IMU interrupt
        if (imu_ready)
        {
            // Clear the flag immediately so the next sensor interrupt can be captured
            imu_ready = 0;

            // Read the newest IMU sample
            imu_read(&imu);

            // Update attitude estimate from gyro + accelerometer data
            filter_update(&attitude,
                          imu.gx_dps,
                          imu.gy_dps,
                          imu.gz_dps,
                          imu.ax_g,
                          imu.ay_g,
                          imu.az_g,
                          dt);

            // Run the controller using the estimated attitude
            control_update(&imu, &attitude);
          }

        // Background tasks can be added here later as long as they stay short
        //
        // Examples:
        //   - heartbeat LED
        //   - telemetry output
        //   - lidar processing
        //   - safety / fault checks
    }

    // Nothing should execute below the main control loop
}



/**
 * @brief LED initialization for heartbeat and debug.
 * 
 * PC6 - red
 * PC7 - blue
 * PC8 - orange
 * PC9 - green
 */
static void LED_init(void) {
  // GPIO_InitTypeDef init = {0};

  // __HAL_RCC_GPIOC_CLK_ENABLE();


  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  (void)RCC->AHBENR;

  GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
                              GPIO_MODE_OUTPUT_PP,
                              GPIO_SPEED_FREQ_LOW,
                              GPIO_NOPULL};

  HAL_GPIO_Init(GPIOC, &initStr);
  
  // a real cute power on sequence
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET); // leds on
  HAL_Delay(1000);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9); // leds off
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET); // leds on
  HAL_Delay(500);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // red off
  HAL_Delay(500);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // green off
  HAL_Delay(500);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7); // blue off
  HAL_Delay(500);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // orange off
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET); // leds on
  HAL_Delay(1000);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9); // leds off
  HAL_Delay(500);

}

void imu_interrupt_init(void) {
    // Enable GPIOC clock
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // Configure PC0 as input, no pull (LSM6DS3 drives actively)
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_IT_RISING;  // LSM6DS3 INT1 is active-high pulse
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);
    
    // Enable SYSCFG clock (required for EXTI mux)
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    
    // Connect EXTI line 0 to Port C via SYSCFG
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI0_PC;
    
    // Configure EXTI line 0
    EXTI->IMR  |= EXTI_IMR_MR0;    // unmask line 0
    EXTI->RTSR |= EXTI_RTSR_TR0;   // rising edge trigger
    EXTI->FTSR &= ~EXTI_FTSR_TR0;  // not falling edge
    
    // Configure and enable NVIC
    NVIC_SetPriority(EXTI0_1_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Configure HSI with PLL to reach 48MHz
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL6;   // 8MHz * 6 = 48MHz
  RCC_OscInitStruct.PLL.PREDIV          = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  // Set PLL as system clock source
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK; // Use PLL instead of HSI
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  // 48MHz requires FLASH_LATENCY_1 (1 wait state)
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* User can add their own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)   // PC0 = IMU INT1
    {
        static uint16_t imu_led_divider = 0;

        imu_ready = 1;

        // Debug: make the 416 Hz IMU interrupt visible as a 1 Hz blink.
        if (++imu_led_divider >= 208)
        {
            imu_led_divider = 0;
            GPIOC->ODR ^= GPIO_ODR_8;
        }
    }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
  /* User can add their own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

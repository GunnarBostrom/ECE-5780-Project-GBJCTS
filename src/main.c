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
#include <stdio.h>
#include <sys/_intsup.h>
#include <sys/types.h>
#include <string.h>
#include "debugger.h"

typedef enum {
    SENSOR_IMU,
    SENSOR_LIDAR,
    SENSOR_RADIO,
} SensorType_t;

void Error_Handler(void);
void SystemClock_Config(void);

static void LED_init(void);
static void imu_interrupt_init(void);
static void SendSensorData(SensorType_t sensor);


/* –––––––––– globals –––––––––– */
LSM6DS3_t imu;
VL53L1X_t lidar;
Attitude_t attitude;

int main(void)
{
    HAL_Init();
    SystemClock_Config();

  LED_init();
  
  // I2C peripheral initialization
  i2c_init(400);

  imu_init(&imu);
  imu_interrupt_init();        // then arm the EXTI
  
  lidar_init(&lidar);
  

  // UART peripheral initialization
  debug_init();
  static uint32_t last_print = 0;

  radio_init();


  // // PWM peripheral initialization
  // motor_init();

  const float dt = 1.0f / 416.0f;

  filter_init(&attitude);
  control_init(dt);
  motor_set_all(1000);

  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
  //HAL_Delay(500);
  
  while (1)
  {
    radio_read();

    if (imu_ready)
    {
      imu_ready = 0;
      imu_read(&imu);
      
      filter_update(&attitude,
                    imu.gx_mdps,
                    imu.gy_mdps,
                    imu.gz_mdps,
                    imu.ax_mg,
                    imu.ay_mg,
                    imu.az_mg,
                    dt);

      control_update(&imu, &attitude);
    }

    
    if (HAL_GetTick() - last_print >= 500) {  // print at ~10 Hz
      last_print = HAL_GetTick();
      SendSensorData(SENSOR_IMU);
    }

    
    

  }
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

  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  (void)RCC->AHBENR;

  GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
                              GPIO_MODE_OUTPUT_PP,
                              GPIO_SPEED_FREQ_LOW,
                              GPIO_NOPULL};

  HAL_GPIO_Init(GPIOC, &initStr);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  // HAL_Delay(1000);
  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
  // HAL_Delay(500);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  // HAL_Delay(500);
  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // led red
  // HAL_Delay(500);
  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // led green
  // HAL_Delay(500);
  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7); // led blue
  // HAL_Delay(500);
  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // led orange
  // HAL_Delay(500);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  // HAL_Delay(1000);
  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
  // HAL_Delay(500);
}

void imu_interrupt_init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_0;
  gpio.Mode = GPIO_MODE_IT_RISING;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &gpio);

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC;

  EXTI->IMR |= EXTI_IMR_MR0;
  EXTI->RTSR |= EXTI_RTSR_TR0;
  EXTI->FTSR &= ~EXTI_FTSR_TR0;

  NVIC_SetPriority(EXTI0_1_IRQn, 1);
  NVIC_EnableIRQ(EXTI0_1_IRQn);
}

static void SendSensorData(SensorType_t sensor) {

  char buffer[100];

  switch (sensor) {
    
    case SENSOR_IMU:
      accept_string("IMU data received\r\n");
      sprintf(buffer, "ax: %d \n\ray: %d\n\raz: %d\n\r\n\rgx: %d\n\rgy: %d\n\rgz: %d\n\r", imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz);
      accept_string(buffer);
      break;

    case SENSOR_LIDAR:
      break;
    case SENSOR_RADIO:
      break;
    default:
      break;
  }

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
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        static uint16_t imu_led_divider = 0;

        imu_ready = 1;

        if (++imu_led_divider >= 208)
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

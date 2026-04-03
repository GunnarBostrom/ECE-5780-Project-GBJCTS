#include "main.h"
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_it.h"
#include <stdint.h>
#include <sys/_intsup.h>
#include <sys/types.h>
#include "motor.h"
#include "config.h"

// parallel logic for lidar and other peripherals
#if USE_IMU
    #include "imu.h"
#else
    #include "imu_fake.h"
#endif

void Error_Handler(void);
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */


// LED INIT for main loop heartbeat
static void LED_init(void)
{
    GPIO_InitTypeDef init = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    // PC8 = orange, PC9 = green
    init.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOC, &init);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
}

int main(void)
{

  //imu_init();
  HAL_Init();
  SystemClock_Config();
  LED_init();
  motor_init();

  uint32_t last_heartbeat = HAL_GetTick();
  uint32_t last_toggle = HAL_GetTick();

  uint8_t motor_state = 0;

  // Arm all ESCs at minimum throttle
  motor_set_all(1000);
  // motor_set_individual(1000, 1000, 1000, 1000);
   HAL_Delay(8000);

  // Hold Motors 1-4 at low throttle
  // Note motor minimum value for all 4 motors to spin at min throttle is 1200
  // motor_set_individual(1200, 1200, 1200, 1200);
  motor_set_all(1200);

  while (1)
  {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // heartbeat
      HAL_Delay(500);
    }
}
  /*loop structure:*/
  //read sensors
  //compute error
  //run PID
  //mix motors
  //update PWM
  // while (1)
  //   {

  // END DRONE CODE










  // // BEGIN BLINKY CODE

  // /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  // HAL_Init();
  // /* Configure the system clock */
  // SystemClock_Config();

  // RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  // (void)RCC->AHBENR;
  
  // GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
  //                             GPIO_MODE_OUTPUT_PP,
  //                             GPIO_SPEED_FREQ_LOW,
  //                             GPIO_NOPULL};

  // HAL_GPIO_Init(GPIOC, &initStr);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);


  // while (1){
  //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
  //   HAL_Delay(500);
  // }
//}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add their own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add their own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

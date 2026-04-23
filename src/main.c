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
#include "pwm.h"
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
void imu_interrupt_init(void);
void SendSensorData(SensorType_t sensor);


/* –––––––––– globals –––––––––– */
//IMU_t lsm6ds3;
LSM6DS3_t imu;
LIDAR_t vl53l1x;



int main(void) {
  HAL_Init();
  SystemClock_Config();

  LED_init();
  
  // I2C peripheral initialization
  i2c_init(400);
  
  //imu_init(&lsm6ds3);     // i2c addr: 0x6B
  imu_init(&imu);
  imu_interrupt_init();        // then arm the EXTI
  
  lidar_init(&vl53l1x, 0x52); // i2c addr: 0x52
  

  // UART peripheral initialization
  debug_init();
  static uint32_t last_print = 0;

  radio_init();


  // PWM peripheral initialization
  motor_init();

  uint32_t last_heartbeat = HAL_GetTick();
  uint32_t last_toggle = HAL_GetTick();
  uint8_t motor_state = 0;

  // Arm all ESCs at minimum throttle
  motor_set_all(1000);
  motor_set_individual(1000, 1000, 1000, 1000);

  // Hold Motors 1-4 at low throttle
  // Note motor minimum value for all 4 motors to spin at min throttle is 1200
  //motor_set_all(1040);
  
  

  /*
   CONTROL LOOP STRUCTURE
      - read sensors
      - compute error
      - run PID
      - mix motors
      - update PWM
  */
  while(1) {
    // need to think about precedence and data frequency

    
    if (imu_ready) {
      imu_ready = 0;
      //imu_read(&lsm6ds3); // highest priority - interrupt with flag
      imu_read(&imu);
      
      HAL_Delay(1000);
      GPIOC->ODR ^= GPIO_PIN_6; // red toggle on IMU read
      
    }
    
    // update motors as soon as IMU data ready

    radio_read();       // medium priority - interrupt with flag

    control_from_radio();
    lidar_read(&vl53l1x);

    if (HAL_GetTick() - last_print >= 500) {  // print at ~10 Hz
      last_print = HAL_GetTick();
      SendSensorData(SENSOR_IMU);
    }

    /* tentative loop structure */

    // if (imu_ready) {
    //   imu_ready = 0;
    //   imu_read(&lsm6ds3);

    //   lidar_counter++;
    //   if (lidar_counter > some_threshold){
    //     lidar_counter = 0;
    //     lidar_read(&vl53l1x);
    //   }

    //   run_pid();
    //   update_motors();
    // }

    // if (radio_ready) {
    //   radio_ready = 0;
    //   radio_read();
    // }


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
    NVIC_SetPriority(EXTI0_1_IRQn, 3U);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void SendSensorData(SensorType_t sensor) {

  char buffer[100];

  switch (sensor) {
    
    case SENSOR_IMU:
      accept_string("IMU data received\r\n");
      // sprintf(buffer, "ax:%d ay:%d az:%d gx:%d gy:%d gz:%d\r\n",
      //   lsm6ds3.ax, lsm6ds3.ay, lsm6ds3.az,
      //   lsm6ds3.gx, lsm6ds3.gy, lsm6ds3.gz);
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

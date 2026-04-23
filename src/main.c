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
void imu_interrupt_init(void);
void SendSensorData(SensorType_t sensor);


/* –––––––––– globals –––––––––– */
//IMU_t lsm6ds3;
LSM6DS3_t imu;
LIDAR_t vl53l1x;
Attitude_t attitude;

int main(void)
{
    HAL_Init();
    SystemClock_Config();

  LED_init();
  
  // I2C peripheral initialization
  i2c_init(400);
  
  //imu_init(&lsm6ds3);     // i2c addr: 0x6B
  imu_init(&imu);
  imu_interrupt_init();        // then arm the EXTI
  
  lidar_init(&lidar);

  while(1) { 
    lidar_read(&lidar);
    //imu_read(&imu); 
  }
  

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

    while (1)
    {
        radio_read();

        if (imu_ready)
        {
            imu_ready = 0;

            imu_read(&lsm6ds3);

            filter_update(&attitude,
                          lsm6ds3.gx_dps,
                          lsm6ds3.gy_dps,
                          lsm6ds3.gz_dps,
                          lsm6ds3.ax_g,
                          lsm6ds3.ay_g,
                          lsm6ds3.az_g,
                          dt);

            control_update(&lsm6ds3, &attitude);
        }
    }
}

static void LED_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    (void)RCC->AHBENR;
  // Arm all ESCs at minimum throttle
  motor_set_all(1000);
  motor_set_individual(1000, 1000, 1000, 1000);

  // Hold Motors 1-4 at low throttle
  // Note motor minimum value for all 4 motors to spin at min throttle is 1200
  //motor_set_all(1040);
  
  

  // /*
  //  CONTROL LOOP STRUCTURE
  //     - read sensors
  //     - compute error
  //     - run PID
  //     - mix motors
  //     - update PWM
  // */
  // while(1) {
  //   // need to think about precedence and data frequency

    
    if (imu_ready) {
      imu_ready = 0;
      //imu_read(&lsm6ds3); // highest priority - interrupt with flag
      imu_read(&imu);
      
  //     HAL_Delay(1000);
  //     GPIOC->ODR ^= GPIO_PIN_6; // red toggle on IMU read
      
  //   }
    
  //   // update motors as soon as IMU data ready

  //   radio_read();       // medium priority - interrupt with flag

  //   control_from_radio();
  //   lidar_read(&lidar);

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


//   }
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
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
    HAL_Delay(500);
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

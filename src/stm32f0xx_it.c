#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_it.h"

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
   while (1)
  {
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}


/**
  * @brief Handles EXTI line 0/1 interrupts.
  *
  * The blue LED (PC7) toggles here — before the HAL dispatch — so a
  * scope or visual check on PC7 confirms whether the raw ISR fires.
  * If PC7 blinks but the orange LED (PC8, toggled in the callback)
  * does not, the HAL dispatch chain is the problem.
  * Remove the PC7 toggle once the interrupt is confirmed working.
  */
// void EXTI0_1_IRQHandler(void) {
//   GPIOC->ODR ^= GPIO_PIN_7;            /* blue LED — ISR diagnostic */
//   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
// }
/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/


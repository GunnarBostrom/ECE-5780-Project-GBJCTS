// motor.c
// Quad ESC PWM driver
// Josh Canada

#include "motor.h"
#include "stm32f0xx.h"

static uint16_t clamp_us(uint16_t val)
{
    if (val < 1000) return 1000;
    if (val > 2000) return 2000;
    return val;
}

void motor_init(void)
{
    // Enable clocks
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // -------------------------------------------------------------------------
    // GPIO CONFIG
    //
    // M1 -> PA0  = TIM2_CH1 (AF2)
    // M2 -> PA1  = TIM2_CH2 (AF2)
    // M3 -> PB10 = TIM2_CH3 (AF2)
    // M4 -> PB11 = TIM2_CH4 (AF2)
    // -------------------------------------------------------------------------

    // Clear mode bits
    GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOB->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);

    // Set alternate function mode
    GPIOA->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);
    GPIOB->MODER |= (GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);

    // Push-pull
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1);
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11);

    // No pull-up / pull-down
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR11);

    // High speed
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR0 | GPIO_OSPEEDR_OSPEEDR1);
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR10 | GPIO_OSPEEDR_OSPEEDR11);

    // AF2 on PA0, PA1
    GPIOA->AFR[0] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)));
    GPIOA->AFR[0] |=  ((0x2 << (0 * 4)) | (0x2 << (1 * 4)));

    // AF2 on PB10, PB11
    GPIOB->AFR[1] &= ~((0xF << ((10 - 8) * 4)) | (0xF << ((11 - 8) * 4)));
    GPIOB->AFR[1] |=  ((0x2 << ((10 - 8) * 4)) | (0x2 << ((11 - 8) * 4)));

    // -------------------------------------------------------------------------
    // TIMER CONFIG
    //
    // Assuming timer clock = 8 MHz
    // PSC = 7 -> 1 MHz timer tick -> 1 us per tick
    // ARR = 19999 -> 20 ms period -> 50 Hz ESC PWM
    // -------------------------------------------------------------------------
    TIM2->PSC = 7;
    TIM2->ARR = 20000 - 1;

    // Safe minimum throttle to start
    TIM2->CCR1 = 1000;
    TIM2->CCR2 = 1000;
    TIM2->CCR3 = 1000;
    TIM2->CCR4 = 1000;

    // Clear channel mode bits first
    TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M);

    // PWM mode 1 + preload enable
    TIM2->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM2->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM2->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM2->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

    // Enable outputs
    TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P | TIM_CCER_CC4P);
    TIM2->CCER |=  (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);

    // Enable ARR preload
    TIM2->CR1 |= TIM_CR1_ARPE;

    // Load registers immediately
    TIM2->EGR |= TIM_EGR_UG;

    // Start timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

void motor_set_all(uint16_t pulse_us)
{
    pulse_us = clamp_us(pulse_us);

    TIM2->CCR1 = pulse_us;
    TIM2->CCR2 = pulse_us;
    TIM2->CCR3 = pulse_us;
    TIM2->CCR4 = pulse_us;
}
//Note motor minimum value for all 4 motors at min throttle is 1200
void motor_set_individual(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    TIM2->CCR1 = clamp_us(m1);
    TIM2->CCR2 = clamp_us(m2);
    TIM2->CCR3 = clamp_us(m3);
    TIM2->CCR4 = clamp_us(m4);
}
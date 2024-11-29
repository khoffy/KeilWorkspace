#include <stm32f429xx.h>

void configure_timer2_with_IT(void) {
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    // Configure for 100ms interval at 90MHz clock
    TIM2->ARR = 999;    // Auto-reload value
    TIM2->PSC = 8999;   // Prescaler for 90MHz clock
    
    // Enable update interrupt
    TIM2->DIER |= TIM_DIER_UIE;
    
    // Enable TIM2 interrupt in NVIC (TIM2 is IRQ 28)
    NVIC->ISER[0] |= (1 << 28);
    
    // Set priority to 7 (bits 4-7 in IPR register)
    NVIC->IPR[28] = (7 << 4);
}

void configure_gpio_pa6_alternate_push_pull(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    // Configure PA6 for alternate function
    // Clear bits [13:12] and set bit 13 for alternate function mode (0b10)
    GPIOA->MODER &= ~(3U << (6 * 2));     // Clear bits
    GPIOA->MODER |= (2U << (6 * 2));      // Set for alternate function
    
    // Set alternate function 2 (TIM3_CH1) for pin 6
    // AFR[0] handles pins 0-7, AFR[1] handles pins 8-15
    // Each pin takes 4 bits
    GPIOA->AFR[0] &= ~(0xFU << (6 * 4));  // Clear AF bits
    GPIOA->AFR[0] |= (2U << (6 * 4));     // Set AF2
    
    // Configure as push-pull (bit=0)
    GPIOA->OTYPER &= ~(1U << 6);
    
    // Set speed to high (0b11)
    GPIOA->OSPEEDR |= (3U << (6 * 2));
}

void configure_pwm_ch1_20khz(TIM_TypeDef *TIMER) {
    // Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    
    // Configure for 20kHz PWM frequency at 90MHz clock
    TIMER->PSC = 0;         // No prescaling
    TIMER->ARR = 4499;      // Auto-reload value for 20kHz
    
    // Configure channel 1 as PWM mode 1 (110 in OC1M bits)
    TIMER->CCMR1 &= ~(7U << 4);           // Clear OC1M bits
    TIMER->CCMR1 |= (6U << 4);            // Set PWM mode 1 (110)
    
    // Enable preload register
    TIMER->CCMR1 |= TIM_CCMR1_OC1PE;
    
    // Enable channel 1 output
    TIMER->CCER |= TIM_CCER_CC1E;
}

void start_timer(TIM_TypeDef *TIMER) {
    TIMER->CR1 |= TIM_CR1_CEN;
}

void set_pulse_percentage(TIM_TypeDef *TIMER, int pulse) {
    TIMER->CCR1 = (TIMER->ARR * pulse) / 100;
}

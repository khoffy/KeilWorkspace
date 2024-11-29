#include <stm32f4xx.h>

void configure_gpio_pa6_alternate_push_pull(void) {
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock
	 // Configure PA6 for alternate function
	 // Clear bits [13:12] & set bit 13 for alternate function mode (0b10)
	 GPIOA->MODER &= ~(3U << (6 * 2));
	 GPIOA->MODER |= (2U << (6 * 2));
	 // Set alternate function 2 (TIM3_CH1) for PA6
	 // AF[0] handles pins 0-7, AFR[1] handles pins 8-15
	 // Each pin takes 4 bits
	 GPIOA->AFR[0] &= ~(0xFU << (6 * 2));
	 GPIOA->AFR[0] |= (2U << (6 * 2));
	 // Configure as push-pull
	 GPIOA->OTYPER &= ~(1U << 6);
	 // Set speed to high (0b11)
	 GPIOA->OSPEEDR |= (3U << (6 * 2));
}

void configure_pwm_ch1_20khz(TIM_TypeDef *TIMER) {
   // Enable TIM3 clock
	 RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	 // Configure for 20Khz PWM frequency at 90Mhz clock
	 TIMER->PSC = 0;
	 TIMER->ARR = 4499;
	 // Configure channel 1 as PWM mode 1 (110 in OC1M bits)
	 TIMER->CCMR1 &= ~(7U << 4); // Clear OC1M bits
	 TIMER->CCMR1 |= (6U << 4); // Set PWM mode 1 (110)
	 // Enable preload register
	 TIMER->CCMR1 |= TIM_CCMR1_OC1PE;
	 // Enable channel 1 output
	 TIMER->CCER |= TIM_CCER_CC1E;
}

void start_timer(TIM_TypeDef *TIMER) {
   TIMER->CR1 |= TIM_CR1_CEN;
}

void set_pulse_percentage(TIM_TypeDef *TIMER, int pulse) {
   TIMER->CCR1 = (TIMER->ARR * pulse)/100;
}

void GPIOB0_Analog_Input_config(void) {
   // No speed or output type config needed for analog input
	 // 1. Enable GPIOB clock
	 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	 // Configure PB0 as analog input
	 // MODER: Set bits [1:0] to 11 (analog mode)
	 GPIOB->MODER &= ~(3U << (1 * 2));
	 GPIOB->MODER |= (3U << (1 * 2));
	 // 3. Optional: Disable pull-p/pull-down
	 GPIOB->PUPDR &= ~(3U << (1 * 2));
}

void ADC1_In8_config(void) {
   // 1. ADC clock activation
	 RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	 // 2. ADC Common Configuration
	 ADC->CCR = 0; 	// Clear CR1
	 ADC->CCR |= ADC_CCR_ADCPRE_0; 	// Set ADC clock to APB2/4
	 // 3. ADC1 Configuration
	 ADC1->CR1 = 0; 	// Clear CR1
	 ADC1->CR2 = 0; 	// Clear CR2
	 // 4. Resolution & conversion configuration
	 ADC1->CR1 &= ~ADC_CR1_RES; 	// 12-bit resolution
	 ADC1->CR2 |= ADC_CR2_ALIGN; 	// right alignment
	 // 5. Channel Selection
	 ADC1->SQR1 &= ~ADC_SQR1_L;
	 ADC1->SQR3 |= 9;
	 // 6. Sampling Time Configuration for Channel 8 (SMPR2 register)
	 // Channel 8 uses bits [26:24] in SMPR2
	 ADC1->SMPR2 &= ~(7 << (3 * 9));
	 ADC1->SMPR2 |= (7 << (3 * 9));
	 // 7. Activate the ADC, set to 1 the ADON bit of CR2 register
	 // Enable ADC
	 ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t ADC1_GetConversionValue(void) {
   // 1. Start the conversion
	 ADC1->CR2 |= ADC_CR2_SWSTART;
	 // 2. Wait for the end of conversion (EOC) flag
	 while(!(ADC1->SR & ADC_SR_EOC));
	 // 3. Read and return the converted value
	 return ADC1->DR;
}

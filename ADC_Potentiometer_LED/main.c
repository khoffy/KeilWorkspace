#include <stm32f4xx.h>
//#include "functions.h"

//int main(void) {
//   uint16_t adc_value = 0;
	 
	 // PWM configuration
//	 configure_gpio_pa6_alternate_push_pull();
//	 configure_pwm_ch1_20khz(TIM3);
//	 set_pulse_percentage(TIM3, 0);
	 // Configure ADC
//	 GPIOB0_Analog_Input_config();
//	 ADC1_In8_config();
	 // Start PWM
//	 start_timer(TIM3);
	
//	 for(;;) {
		  // Trigger conversion & get value
//	    adc_value = ADC1_GetConversionValue();
		  // Convert to voltage (assuming 3.3V reference)
//		  float voltage = (adc_value * 3.3f) / 4096.0f;
		  // Update of LED intensity
//		  set_pulse_percentage(TIM3, 100 * voltage / 0xFFF);
//	 }
	 
//	 return 0;
//}

// Configure ADC1 for potentiometer on PB1 (Channel 9)
//void ADC1_Init(void) {
    // Enable GPIOB and ADC1 clock
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
//    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    
    // Configure PB1 as analog input
//    GPIOB->MODER |= GPIO_MODER_MODER1;
    
    // ADC configuration
//    ADC1->CR1 = ADC_CR1_SCAN; // Scan mode
//    ADC1->CR2 = ADC_CR2_ADON; // Turn on ADC
    
    // Select channel 9
//    ADC1->SQR3 = 9; // Channel 9 for first conversion
//    ADC1->SQR1 = 0; // 1 conversion
//}

// Read ADC value from potentiometer
//uint16_t ADC1_Read(void) {
    // Start conversion
//    ADC1->CR2 |= ADC_CR2_SWSTART;
    
    // Wait for conversion complete
//    while(!(ADC1->SR & ADC_SR_EOC));
    
    // Return converted value
//    return ADC1->DR;
//}

// Configure LED on PA6 with Timer 3 CH1 for PWM
//void LED_PWM_Init(void) {
    // Enable GPIOA and TIM3 clock
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
//    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    
    // Configure PA6 as alternate function (TIM3 CH1)
//    GPIOA->MODER |= GPIO_MODER_MODER6_1; // Alternate function mode
//    GPIOA->AFR[0] |= (2 << 24); // AF2 for TIM3
    
    // Configure Timer 3 for PWM
//    TIM3->PSC = 84 - 1; // Assuming 84MHz clock
//    TIM3->ARR = 100 - 1; // PWM period
//    TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // PWM mode 1
//    TIM3->CCER |= TIM_CCER_CC1E; // Enable output compare
//    TIM3->CR1 |= TIM_CR1_CEN; // Enable timer
//}

// Set LED brightness based on ADC value
//void Set_LED_Brightness(uint16_t adc_value) {
    // Map ADC value (0-4095) to PWM duty cycle (0-100)
//    uint16_t brightness = (adc_value * 100) / 4095;
//    TIM3->CCR1 = brightness;
//}

//int main(void) {
    // Initialize peripherals
//    ADC1_Init();
//    LED_PWM_Init();
    
//    while(1) {
        // Read potentiometer
//        uint16_t pot_value = ADC1_Read();
        
        // Set LED brightness
//        Set_LED_Brightness(pot_value);
//    }
//}



//#include "stm32f4xx.h"

void GPIO_Init(void);
void ADC_Init(void);
void TIM3_PWM_Init(void);
uint16_t ADC_Read(void);
void Set_PWM_Duty(uint16_t duty);

int main(void) {
    uint16_t adc_value;
    uint16_t pwm_duty;

    // Initialize GPIO, ADC, and TIM3
    GPIO_Init();
    ADC_Init();
    TIM3_PWM_Init();

    while (1) {
        // Read the ADC value from PB1
        adc_value = ADC_Read();

        // Map ADC value (0–4095) to PWM duty cycle (0–100%)
        pwm_duty = (adc_value * 100) / 4095;

        // Set the PWM duty cycle
        Set_PWM_Duty(pwm_duty);
    }
}

void GPIO_Init(void) {
    // Enable clocks for GPIOA and GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // Configure PA6 as alternate function (TIM3_CH1)
    GPIOA->MODER |= (2 << (6 * 2)); // Alternate function
    GPIOA->AFR[0] |= (2 << (6 * 4)); // AF2 (TIM3)

    // Configure PB1 as analog mode (ADC1_IN9)
    GPIOB->MODER |= (3 << (1 * 2)); // Analog mode
}

void ADC_Init(void) {
    // Enable ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Configure ADC1
    ADC1->SQR3 = 9;                 // Set channel 9 (PB1) as first in sequence
    ADC1->SMPR2 |= (7 << (3 * 9));  // Sampling time: 480 cycles
    ADC1->CR2 |= ADC_CR2_ADON;      // Enable ADC1
}

uint16_t ADC_Read(void) {
    // Start conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Wait for conversion to complete
    while (!(ADC1->SR & ADC_SR_EOC));

    // Return ADC value
    return ADC1->DR;
}

void TIM3_PWM_Init(void) {
    // Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Configure TIM3 for PWM
    TIM3->PSC = 15;                 // Prescaler to get 1 MHz timer clock (16 MHz / 16)
    TIM3->ARR = 999;                // Auto-reload value for 1 kHz frequency (1 MHz / 1000)
    TIM3->CCR1 = 0;                 // Start with 0% duty cycle
    TIM3->CCMR1 |= (6 << 4);        // PWM mode 1
    TIM3->CCER |= TIM_CCER_CC1E;    // Enable output on channel 1
    TIM3->CR1 |= TIM_CR1_CEN;       // Enable the timer
}

void Set_PWM_Duty(uint16_t duty) {
    // Set CCR1 based on duty cycle percentage
    TIM3->CCR1 = (duty * (TIM3->ARR + 1)) / 100;
}

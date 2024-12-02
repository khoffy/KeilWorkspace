/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stm32f4xx.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void GPIO_Init(void);
void ADC_Init(void);
void TIM3_PWM_Init(void);
uint16_t ADC_Read(void);
void Set_PWM_Duty(uint16_t duty);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint16_t adc_value;
  uint16_t pwm_duty;

  // Initialize GPIO, ADC, and TIM3
  GPIO_Init();
  ADC_Init();
  TIM3_PWM_Init();


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		// Read the ADC value from PB1
        adc_value = ADC_Read();

        // Map ADC value (0�4095) to PWM duty cycle (0�100%)
        pwm_duty = (adc_value * 100) / 4095;

        // Set the PWM duty cycle
        Set_PWM_Duty(pwm_duty);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
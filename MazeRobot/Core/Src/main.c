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
#include "motor.h"

/* USER CODE BEGIN PV */
extern volatile uint16_t rawDistanceValue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void TurnRight90(void);
void TurnLeft90(void);
void GoForwardOne(void);
void Transmit(int, char);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Transmits one character over the USART interface
void TransmitChar(char c) {
	while(!(USART3->ISR & 0x80));	// wait until TXE (bit 7 of ISR) is 1
	USART3->TDR = c;	// write the character to the USART
}

// Transmits a string
void TransmitString(char s[]) {
	int i = 0;
	while(s[i]) {		// Loop until a null character is reached
		TransmitChar(s[i++]);
	}
}

// Transmits a number on USART
void TransmitNumber(int32_t n) {
	// Max integer is 10 digits (plus sign maybe) so create buffer of length 12
	char buffer[12];
	
	// Calculate sign
	uint8_t sign = (n < 0);
	if (sign) {
		n = -n;
	}
	
	// Convert to chars
	uint8_t i = 0;
	while(n > 9) {
		buffer[i++] = (n % 10) + 48;	// Convert digit to char
		n /= 10;	// Shift to next digit
	}
	buffer[i++] = (n % 10) + 48;	// Convert last digit to char
	
	if(sign) {
		buffer[i++] = '-';
	}
	uint8_t numDigits = i;
	
	// Flip the characters since we wrote them backwards
	for(i = 0; i < (numDigits + 1) / 2; ++i) {
		char temp = buffer[i];
		buffer[i] = buffer[numDigits - i - 1];
		buffer[numDigits - i - 1] = temp;
	}
	
	// Add null terminator
	buffer[numDigits] = 0;
	
	// Print to screen
	TransmitString(buffer);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	SystemClock_Config(); //Configure the system clock
	
	// PB4 = trigger, PA8 = echo, PC5 = TX on UART board, PC4 = RX on UART board, PA4 PA5 PA9 motor 1 control, PB3 PB5 PB6 motor 1 control
	
	// Enable GPIO and Timer 2
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_USART3EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
	
	// Configure LEDs and USART
	GPIOC->MODER = 0x55A00;
	GPIOC->AFR[0] |= 0x110000;
	
	// Initialize the USART
	USART3->BRR = 8000000 / 115200;	// Set baud rate
	USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;	// 8 data bit, 1 start bit, 1 stop bit, no parity, reception and transmission mode
	
	// Enable interrupt
	NVIC_EnableIRQ(TIM2_IRQn);
	
	// Configure alternate function of PB4
	GPIOB->MODER = 0x200;	// PB4 = AF
	GPIOB->AFR[0] = 0x10000;	//AF 2
	
	// Timer 3 setup for PWM on trigger
	TIM3->CCMR1 |= (0x1 << 3);	// Enable output compare preload
	TIM3->PSC = 7;		// Set prescaler to 8
	TIM3->ARR = 10000;	// Set auto-reload to 10000
	TIM3->CCMR1 |= (0x6 << 4);	// Set channel 1 to PWM mode 1
	TIM3->CCER |= 0x1;		// Enable CC
	TIM3->CCR1 = 10;		// Set duty cycle to 0.1%
	TIM3->CR1 |= 0x1;		// Enable timer 3
	
	// GPIO setup for echo input capture	
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER8)) | (GPIO_MODER_MODER8_1);
  GPIOA->AFR[1] |= 0x02;
  
	// Timer 1 setup for echo input capture
	NVIC_EnableIRQ(TIM1_CC_IRQn);
  NVIC_SetPriority(TIM1_CC_IRQn,0);
  TIM1->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1;
  TIM1->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_0 | TIM_SMCR_SMS_2;
  TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;  
  TIM1->DIER |= TIM_DIER_CC1IE;
  TIM1->CR1 |= TIM_CR1_CEN;
	
	// Initialize both motor drivers
	motor_init();
	
	// Enable pull down resistor for button
	GPIOA->PUPDR |= 0x2;
	
	uint32_t debouncer = 0;
	uint8_t buttonState = 0;
	while (debouncer != 0x7FFFFFFF) {
		
		debouncer = (debouncer << 1); // Always shift every loop iteration
		
		if (GPIOA->IDR & 0x1) { // If input signal is set/high
			debouncer |= 0x01; // Set lowest bit of bit-vector
		}
	}
	HAL_Delay(1500);
	
	char originalDirection = 5; //Original Direction of bot
	char arr[102]; //Value chosen for bit map lenght is assuming no more than 34 cells
	char solved = 0; //boolean to check if maze is solved
	
	int n=0; //Integer to go through array
	int maxN = 0; //Integer to record maximum length of array
	
	while(1){
		debouncer = 0;
		
			if(solved){
				HAL_Delay(2000);
				while (debouncer != 0x7FFFFFFF) {
					debouncer = (debouncer << 1); // Always shift every loop iteration
					if (GPIOA->IDR & 0x1) { // If input signal is set/high
							debouncer |= 0x01; // Set lowest bit of bit-vector
					}
				}
				for(n=0; n<=maxN; n+=3){
					GoForwardOne();
					if(arr[n] == 0){
						TurnLeft90();
					}
					else if(arr[n+2] == 0){
						TurnRight90();
					}
				}
				GoForwardOne();	//Exit the maze
				continue;
			}
			
			// Main solve loop
			while(!(GPIOA->IDR & 0x1)){
			GoForwardOne();
			if(GPIOA->IDR &0x1) {
				break;
			}
			//0 = open, 1 equals not checked, 2 = wall

			//Mark All Directions as "not checked"
			arr[n]=1;
			arr[n+1]=1;
			arr[n+2]=1;

			//Check Left Wall, follow if open
			TurnLeft90();
			if(rawDistanceValue/480 > 15){
				arr[n] = 0;
				n+=3;
				continue;
			}

			//Close Left wall, check Forward Wall, follow if open
			arr[n] = 2;
			TurnRight90();
			if(rawDistanceValue/480 > 15){
				arr[n+1] = 0;
				n+=3;
				continue;
			}

			//Close Foward  Wall, check right wall, follow if open
			arr[n+1] = 2;
			TurnRight90();
			if(rawDistanceValue/480 > 15){
				arr[n+2] = 0;
				n+=3;
				continue;
			}	
		
			arr[n+2] = 2;
			TurnRight90();

			//Begin Backtracking
			while(arr[n] == 2 && arr[n+1] == 2 && arr[n+2] == 2){
				originalDirection = 5; //Reset originalDirection
		
				//Reset arr[n through n+2]
				arr[n]=1;
				arr[n+1]=1;
				arr[n+2]=1;
				n-=3;

				//Go back one cell
				GoForwardOne();
		
				//Check for incorrect open path -> close it
				if(arr[n] == 0){
					arr[n] = 2;
					//Transmit(n, 2);
					originalDirection = 0;
				}
				else if(arr[n+1] == 0){
					arr[n+1] = 2;
					//Transmit(n+1, 2);
					originalDirection = 1;
				}
				else if(arr[n+2] == 0){
					arr[n+2] = 2;
					//Transmit(n+2, 2);
					originalDirection = 2;
				}

				//Check (original) straight or right directions, go there OR iterate through backtrack loop again
				if(arr[n+1] == 1){
					TurnLeft90();
					if(rawDistanceValue/480 < 15){
						arr[n+1] = 2;
						//Transmit(n+1, 2);
					}
					else{
						arr[n+1] = 0;
						//Transmit(n+1, 0);
						break;
					}
				}
				if(arr[n+2] == 1){
					if(originalDirection == 0){
						TurnRight90();
					}
					else{
						TurnLeft90();
					}
					if(rawDistanceValue/480 < 15){
						arr[n+2] = 2;
						//Transmit(n+2, 2);
					}
					else{	
						arr[n+2] = 0;
						//Transmit(n+2, 0);
						break;
					}
				}
			}
			
			// Move to next cell now that backtracking is done
			n+=3;
		}
		maxN=n;
		solved = 1;
	}
}

/**
	* @brief Transmits index and value to UART
	*	@retval None
	*/
void Transmit(int n, char c){
	TransmitNumber(n);
	TransmitString(" : ");
	TransmitNumber(c);
}

/**
	* @brief Turns the bot left 90 degrees
	*	@retval None
	*/
void TurnLeft90(void){
	TurnLeft();
	HAL_Delay(425);
	Stop();
	HAL_Delay(1000);
}

/**
	* @brief Turns the bot right 90 degrees
	*	@retval None
	*/
void TurnRight90(void){
	TurnRight();
	HAL_Delay(425);
	Stop();
	HAL_Delay(1000);
}

/**
	* @brief Moves the bot Forward one cell
	*	@retval None
	*/
void GoForwardOne(void){
	GoForward();
	HAL_Delay(1100);
	Stop();
	HAL_Delay(1000);
}


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

/* USER CODE BEGIN 4 */

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

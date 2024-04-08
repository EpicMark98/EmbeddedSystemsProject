/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */
#include "../inc/motor.h"

volatile int16_t error_integral_prev = 0;
volatile int16_t error_integral = 0;    // Integrated error signal
volatile uint8_t duty_cycle = 0;    	// Output PWM duty cycle
volatile int16_t target_rpm = 0;    	// Desired speed target
volatile int16_t motor_speed = 0;   	// Measured motor speed
volatile int8_t adc_value = 0;      	// ADC measured motor current
volatile int16_t error = 0;         	// Speed error signal
volatile uint8_t Kp = 1;            	// Proportional gain
volatile uint8_t Ki = 1;            	// Integral gain

// Sets up the entire motor drive system
void motor_init(void) {
    pwm_init_left();
		pwm_init_right();
}

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init_left(void) {
    
    // Set up pin PA4 (enable) for H-bridge PWM output (TIMER 14 CH1)
    GPIOA->MODER |= (1 << 9);
    GPIOA->MODER &= ~(1 << 8);

    // Set PA4 to AF4,
    GPIOA->AFR[0] &= 0xFFF0FFFF; // clear PA4 bits,
    GPIOA->AFR[0] |= (1 << 18);

    // Set up a PA5 (dir A?), PA6 (dir b?) as GPIO output pins for motor direction control
    GPIOA->MODER &= 0xFFFFC3FF; // clear PA5, PA6 bits,
    GPIOA->MODER |= (1 << 10) | (1 << 12);
    
    //Initialize one direction pin to high, the other low
    GPIOA->ODR |= (1 << 5);
    GPIOA->ODR &= ~(1 << 6);

    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->CR1 = 0;                         // Clear control registers
    TIM14->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM14->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM14->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    TIM14->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    TIM14->PSC = 1;                         // Run timer on 24Mhz
    TIM14->ARR = 1200;                      // PWM at 20kHz
    TIM14->CCR1 = 0;                        // Start PWM at 0% duty cycle
    
    TIM14->CR1 |= TIM_CR1_CEN;              // Enable timer
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle_left(uint8_t duty) {
    if(duty <= 100) {
        TIM14->CCR1 = ((uint32_t)duty*TIM14->ARR)/100;  // Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init_right(void) {
    
    // Set up pin PB3 (enable) for H-bridge PWM output (TIMER 2 CH2)
    GPIOB->MODER |= 0x80;

    // Set PB3 to AF2,
    GPIOB->AFR[0] |= 0x2000;

    // Set up a PB5 (dir A), PB6 (dir b) as GPIO output pins for motor direction control
    GPIOB->MODER |= (1 << 10) | (1 << 12);
    
    //Initialize one direction pin to high, the other low
    GPIOB->ODR |= (1 << 5);
    GPIOB->ODR &= ~(1 << 6);

    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1 = 0;                         // Clear control registers
    TIM2->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM2->CCER = 0;

    // Set output-compare CH2 to PWM1 mode and enable CCR1 preload buffer
    TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE);
    TIM2->CCER |= TIM_CCER_CC2E;           // Enable capture-compare channel 2
    TIM2->PSC = 1;                         // Run timer on 24Mhz
    TIM2->ARR = 1200;                      // PWM at 20kHz
    TIM2->CCR2 = 0;                        // Start PWM at 0% duty cycle
    
    TIM2->CR1 |= TIM_CR1_CEN;              // Enable timer
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle_right(uint8_t duty) {
    if(duty <= 100) {
        TIM2->CCR2 = ((uint32_t)duty*TIM2->ARR)/100;  // Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}
//Left: DIRA - PA5 DIRB - PA6		Right: DIRA - PB5 DIRB - PB6
void GoForward() {
	// Set DIRA high
	GPIOA->ODR |= (1 << 5);
	GPIOB->ODR |= (1 << 5);
	
	// Set DIRB low
	GPIOA->ODR &= ~(1 << 6);
	GPIOB->ODR &= ~(1 << 6);

	pwm_setDutyCycle_left(50);
	pwm_setDutyCycle_right(50);
}

void GoBackwards() {
	// Set DIRA low
	GPIOA->ODR &= ~(1 << 5);
	GPIOB->ODR &= ~(1 << 5);
	
	// Set DIRB high
	GPIOA->ODR |= (1 << 6);
	GPIOB->ODR |= (1 << 6);
	
	pwm_setDutyCycle_left(50);
	pwm_setDutyCycle_right(50);
}

void Stop() {
	pwm_setDutyCycle_left(0);
	pwm_setDutyCycle_right(0);
}

void TurnLeft() {
	// Left wheel backward
	GPIOA->ODR &= ~(1 << 5);
	GPIOA->ODR |= (1 << 6);
	
	// Right wheel forward
	GPIOB->ODR |= (1 << 5);
	GPIOB->ODR &= ~(1 << 6);
	
	pwm_setDutyCycle_left(50);
	pwm_setDutyCycle_right(50);
}

void TurnRight() {
	// Left wheel forward
	GPIOA->ODR |= (1 << 5);
	GPIOA->ODR &= ~(1 << 6);
	
	// Right wheel backward
	GPIOB->ODR &= ~(1 << 5);
	GPIOB->ODR |= (1 << 6);
	
	pwm_setDutyCycle_left(50);
	pwm_setDutyCycle_right(50);
}

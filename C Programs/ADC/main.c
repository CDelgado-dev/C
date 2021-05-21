#include "stm32f4xx.h"
#include "stdio.h"

void USART2_Init(void);
void Timer2_Init(void);
void Button_Init(void);
void Thermal_Init(void);
void Potentiometer_Init(void);
void ADC1_Init(void);
void LED_Output_Init(void);
void ADC1_Potentiometer(void);
void Set_Delay_us(int val);
void myPrint(char msg[]);
void USART2_read(int character);
char txt[256];


int main(void){
	__disable_irq();

	USART2_Init();							// Initialize UART2 communication registers
	Button_Init();						// Configure on-board button as input signal
	NVIC_EnableIRQ(EXTI15_10_IRQn);			// Enable Interrupt in NVIC
	NVIC_SetPriority(EXTI15_10_IRQn, 25);	// Change the priority of Interrupt to take precedence over ADC interrupt
	Thermal_Init();				// Configure A0 for analog input of Thermal sensor.
	Potentiometer_Init();				// Configure A1 for analog input of Potentiometer
	ADC1_Init();						// Configure ADC to read Thermmal sensor and Potentiometer inputs
	NVIC_EnableIRQ(ADC_IRQn);			// Enable ADC Interrupt in NVIC
	NVIC_SetPriority(ADC_IRQn, 47);		// Change the priority of Interrupt to be lower importance than Button-Press interrupt
	LED_Output_Init();				// Configure on-board LED to check timings
	Timer2_Init();					// Configure TIM2 to be the trigger for ADC
	NVIC_EnableIRQ(TIM2_IRQn);			// Enable TIM2 Interrupt in NVIC

	__enable_irq();

	while (1){

	}
}

void EXTI15_10_IRQHandler(void){	// Handler for On-Board Button press Interrupt
	ADC1_Potentiometer();		// Read Potentiometer and reset TIM2 delay
	EXTI->PR = (1<<13);				// Clear interrupt pending flag
}


void TIM2_IRQHandler(void){			// Handler for TIM2 interrupt (Toggle LED for testing)
	GPIOA->ODR ^= (1<<5);			// Toggle On-Board LED (pin A5) on timer interrupt
	TIM2->SR &= ~(1<<0);			// Clear the Update Interrupt Flag (bit 0)
}


void ADC_IRQHandler(void){			// Handler for ADC interrupt thermal sensor
	float voltage = (float)(ADC1->DR) / 4095 * 3300 ;						// Convert reading to mV
	float tempCel = ((voltage - 50) / 10) ;									// Convert mV to Celcius
    float tempFer = (float)tempCel * 1.8 + 32;								// Convert tempC to F degrees
	sprintf(txt, "$%.02f %.02f %.02f;",voltage, tempCel, tempFer);			//print to console Voltage(mV), C, F
	myPrint(txt);
}

void ADC1_Potentiometer(void){
	ADC1->CR2 |= (1<<22);													// Start conversion of Injected channels
	while (!(ADC1->SR & (1<<2))) {}											// Wait for ADC
	float read = (((float)(ADC1->JDR1 & 0xFFF) / 0xFFF) * 1999999) + 1;		// Scale ADC reading to 1-2,000,000
	Set_Delay_us((int)read);												// Change the TIM2 delay according to Potentiometer reading
	ADC1->SR &= ~(1<<2);													// Clear Injected Channel
}

void USART2_Init(void){
	RCC->AHB1ENR |= 1;					// GPIOA
	RCC->APB1ENR |= (1<<17);			// UART2
	GPIOA->MODER  &= ~(0x3<<4); 	// Clear the two bits for PA2
	GPIOA->MODER  |=  (0x2<<4); 	// Set the mode to AF
	GPIOA->AFR[0] &= ~(0xF<<8); 	// Clear the 4 bits for PA2
	GPIOA->AFR[0] |=  (0x7<<8); 	// Set the 4 bits to 7

	USART2->BRR= 0x8B;					// Set baud rate register to 0x8B (115200 baud @ 16MHz)
	USART2->CR1 |= (1<<3);				// Transmit Enable
	USART2->CR1 |= (1<<2);				// Receive Enable
	USART2->CR2 = 0x0000;
	USART2->CR3 = 0x0000;
	USART2->CR1 |= (1<<13);				// USART Enable

}

void Timer2_Init(void){
	RCC->APB1ENR |= 1;              	// Enable TIM2 clock
	TIM2->PSC = 16-1;					// Divided by 16
	TIM2->ARR = 1000000;				// Divided by 1,000,000 to produce 1,000,000us (1sec)
	TIM2->CNT = 0;                  	// Clear timer counter
	TIM2->CR2 |= (2<<4);				// Set Master Mode Select to Update
	TIM2->DIER |= (1<<0);				// Enable Update Interrupt
	TIM2->CR1 |= (1<<0);				// Enable TIM2
}


void Button_Init(void){
	RCC->AHB1ENR |= (1<<2);					// Enable GPIOC clock
	RCC->APB2ENR |= (1<<14);				// Enable SYSCFG clock
	GPIOC->MODER &= ~(3<<26);				// Clear bits 26 and 27
	SYSCFG->EXTICR[3] &= ~(0xF<<4);			// Clear port selection for EXTI 13
	SYSCFG->EXTICR[3] |= (2<<4);			// Select port C for EXTI 13
	EXTI->IMR |= (1<<13);					// Unmask EXTI 13
	EXTI->FTSR |= (1<<13);					// Select Falling-Edge trigger
}


void ADC1_Init(void){
	RCC->APB2ENR |= (1<<8);				// Enable ADC1 clock (bit 8)

	ADC1->CR1 = 0;						// Disable everything in ADC1 Control Register 1
	ADC1->CR1 |= (1<<12);				// Enable discontinuous mode on the Injected channels of a group
	ADC1->CR1 |= (1<<11);				// Enable discontinuous mode on Regular channels
	ADC1->CR1 |= (1<<5);				// Enable End Of Conversion Interrupt Flag

	ADC1->CR2 |= (1<<28);				// Enable External Trigger for Regular channels
	ADC1->CR2 |= (6<<24);				// Set Timer 2 TRGO event as trigger to start conversion for Regular channels


	ADC1->SMPR2 |= (1<<3);				// Set Sampling Time of A1 to 15 cycles
	ADC1->SMPR2 |= (1<<0);				// Set Sampling Time of A0 to 15 cycles

	ADC1->SQR1 = 0;						// Conversion Sequence length 1 for Regular channels
	ADC1->SQR3 = 0;						// Set first conversion in Regular sequence to PA0
	ADC1->SQR3 |= (1<<5);				// Set second conversion in Regular sequence to PA1

	ADC1->JSQR = 0;						// Set Injected channels sequence length to 1
	ADC1->JSQR |= (1<<15);				// Set Injected conversion to PA1
	ADC1->CR2 |= 1;						// Enable A/D Converter On/Off
}

void LED_Output_Init(void){
	RCC->AHB1ENR |= 1;				// Enable GPIOA clock
	GPIOA->MODER &= ~(3<<10);		// Clear bits 10 and 11
	GPIOA->MODER |= (1<<10);		// Set bits 10 and 11 to binary 10
	GPIOA->ODR |= (1<<5);			// Turn on on-board LED
}

void Thermal_Init(void){
	// Analog input PA0
	RCC->AHB1ENR |= 1;					// Enable GPIOA Clock
	GPIOA->MODER &= ~(3);				// Clear bits 1:0 for GPIOA-0
	GPIOA->MODER |= 3;					// Set bits 1:0 to analog input
}

void Potentiometer_Init(void){
	// Analog input PA1
	RCC->AHB1ENR |= 1;					// Enable GPIOA Clock
	GPIOA->MODER &= ~(3<<2);		// Clear bits 3:2 for GPIOA-1
	GPIOA->MODER |= (3<<2);			// Set bits 3:2 to analog input
}


void myPrint(char msg[]){
	uint8_t index = 0;
	while( msg[index] != 0){
		USART2_read(msg[index++]);
	}
}

void USART2_read(int character){
	while (!(USART2->SR & (1<<7))) {};				// Wait until Tx buffer is empty
	USART2->DR = (character & 0xFF);				// Send 'character' input to USART Data Register
}

void Set_Delay_us(int val){
	TIM2->CR1 &= ~(1<<0);			// Disable the counter
    TIM2->ARR = val;				// Set the reload value to new input
    TIM2->CNT = 0;                  // Clear timer counter
    TIM2->CR1 |= (1<<0);			// Enable TIM2 counter
}

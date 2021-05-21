// IMPORTANT NOTE
// TO SEE WAVELENGTH AN RC FILTER WAS USED TO CLEAN UP SIGNAL
// USED AN 100 Ohm Resister and a ceramic capacitor (0.1uF)
#include "stm32f4xx.h"
#include "stdio.h"
#include <math.h>

void myPrint(char msg[]);
void USART2_Tx(int c);
void Timer1_Init(void);
void SysTick_Timer_Init(void);
void Waveform_Outputs(void);
void ADC1_Init(void);
void USART2_Init(void);
void WaveformCalculations(void);
void Read_Send_ADC1(void);
void SendStorage(void);
void delayMs(int);
#define Cycles 100
#define Wave_length 100
#define PI 3.1415926535

int Sine[100];											// Sine array
int Square[100];										// Squate array
int Triangle[100];										// Triangle array
int Sawtooth[100];										// Sawtooth array
int Waveindex = 1;										// Index value to step through the waveform arrays
int Storage[1000];										// ADC reading array
int WaveIndexPostition = 0;								// Index values to step through Storage array
char txt[256];											// Array to save ADC results then transmit via USART2

int main(void){
	__disable_irq();

	WaveformCalculations();					// Calculate all values for the 4 waveforms and populate arrays
	Waveform_Outputs();						// Configure GPIOA 8-11 to be outputs for TIM1 waveforms (A8-Sine, A10-Triangle, A11-Square, A12-Saw)
	ADC1_Init();							// Configure ADC1 to read input GPIO_A0
	NVIC_EnableIRQ(ADC_IRQn);				// Enable ADC Interrupt in NVIC
	USART2_Init();							// Configure USART2
	Timer1_Init();							// Configure TIM1 to produce 4 waveforms
	SysTick_Timer_Init();					// Configure SysTick

	__enable_irq();
	while(1){}
}

void SendStorage(void){
	for (int i=0 ; i<1000 ; i++){
		float Volt = (float)Storage[i] * ((float)3300 / 0xFFF);		 					// Convert ADC reading to mV
		sprintf(txt, "$%.02f;", Volt);													// Format the Voltage value and save in txt buffer
		myPrint(txt);																	// Send the Voltage value in txt buffer to serial monitor
		//delayMs(10);
	}
}

void SysTick_Handler(void){
	ADC1->CR2 |= (1<<30);							// Start conversion
	TIM1->CCR1 = Sine[Waveindex];					// Change CCR1 Value
	TIM1->CCR2 = Square[Waveindex];					// Change CCR2 Value
	TIM1->CCR3 = Triangle[Waveindex];				// Change CCR3 Value
	TIM1->CCR4 = Sawtooth[Waveindex++];				// Change CCR4 Value

	if (Waveindex > 99){
		Waveindex = 0;                              //reset
	}
}

void ADC_IRQHandler(void) {
	Storage[WaveIndexPostition++] = (ADC1->DR & 0xFFF);			// Save ADC value to Storage buffer
	if (WaveIndexPostition >= 1000) {							// If Storage buffer is full
		SendStorage();											// Function to convert each ADC value and send to serial monitor
		WaveIndexPostition = 0;									// Reset Index value for Storage buffer
	}

}

void Timer1_Init(void){
	RCC->APB2ENR |= 1;              	// Enable TIM1 clock
	TIM1->ARR = Cycles-1;				// Divided by 100
	TIM1->CNT = 0;                  	// Clear timer counter
	TIM1->CCER |= 1<<0;					// Enable Capture/Compare 1
	TIM1->CCMR1 |= 6<<4;				// Set Output Compare 1 mode to PWM1
	TIM1->CCMR1 |= 1<<3;				// Enable Output Compare 1 preload
	TIM1->CCR1 = Sine[0];				// Pre-load CCR1 value to first value of Sine

	TIM1->CCER |= 1<<4;					// Enable Capture/Compare 2
	TIM1->CCMR1 |= 6<<12;				// Set Output Compare 2 mode to PWM1
	TIM1->CCMR1 |= 1<<11;				// Enable Output Compare 1 preload
	TIM1->CCR2 = Square[0];				// Preload CCR2 value to first value of Square

	TIM1->CCER |= 1<<8;					// Enable Capture/Compare 3
	TIM1->CCMR2 |= 6<<4;				// Set Output Compare 3 mode to PWM1
	TIM1->CCMR2 |= 1<<3;				// Enable Output Compare 3 preload
	TIM1->CCR3 = Triangle[0];			// Preload CCR3 value to first value of Triangle

	TIM1->CCER |= 1<<12;				// Enable Capture/Compare 4
	TIM1->CCMR2 |= 6<<12;				// Set Output Compare 4 mode to PWM1
	TIM1->CCMR2 |= 1<<11;				// Enable Output Compare 4 preload
	TIM1->CCR4 = Sawtooth[0];			// Preload CCR4 value to first value of Sawtooth
	TIM1->BDTR |= 1<<15;				// Enable Main Output Enable
	TIM1->CR1 |= 1<<0;					// Enable TIM1
}

void Waveform_Outputs(void){
	RCC->AHB1ENR |= 1;					// Enable GPIOA Clock
	GPIOA->MODER &= ~(3<<16);			// Clear bits
	GPIOA->MODER |= 2<<16;				// Set GPIO-A8 Mode to AF
	GPIOA->AFR[1] |= 1<<0;				// Set GPIO-A8 to AF-1

	GPIOA->MODER &= ~(3<<18);			// Clear bits
	GPIOA->MODER |= 2<<18;				// Set GPIO-A9 Mode to AF
	GPIOA->AFR[1] |= 1<<4;				// Set GPIOA-9 to AF-1 (TIM1 CH2)

	GPIOA->MODER &= ~(3<<20);			// Clear bits
	GPIOA->MODER |= 2<<20;				// Set GPIO-A10 Mode to AF
	GPIOA->AFR[1] |= 1<<8;				// Set GPIO-A10 to AF-1 (TIM1 CH3)

	GPIOA->MODER &= ~(3<<22);			// Clear bits
	GPIOA->MODER |= 2<<22;				// Set GPIO-A11 Mode to AF
	GPIOA->AFR[1] |= 1<<12;				// Set GPIO-A11 to AF-1 (TIM1 CH4)
}

void SysTick_Timer_Init(void){
	SysTick->LOAD = 160-1;				// Load with 160 to occur 100,000 times per sec so each waveform is broken into 100 values, for each to cycle at 1KHz
	SysTick->VAL = 0;					// Set count value to 0
	SysTick->CTRL = 0b111;				// Enable SysTick counter, Enable interrupt, and System Clock
}

void ADC1_Init(void){
	RCC->AHB1ENR |= 1;					// Enable GPIOA Clock
	GPIOA->MODER &= ~(3);				// Clear bits
	GPIOA->MODER |= 3;					// Set analog input GPIOA0
	RCC->APB2ENR |= (1<<8);				// Enable ADC1 clock
	ADC1->CR1 = 0;						// SW trigger
	ADC1->CR1 |= (1<<11);				// Enable discontinuous mode on Regular channels
	ADC1->CR1 |= (1<<5);				// Enable EOC Interrupt Flag
	ADC1->SQR1 = 0;						// Conversion Sequence length 1 for Regular channels
	ADC1->SQR3 = 0;						// Set first conversion in Regular sequence to PA0
	ADC1->CR2 |= 1;						// Enable A/D Converter
}


void USART2_Init(void){
	RCC->AHB1ENR |= 1;					// Enable the clock of the GPIOA
	RCC->APB1ENR |= (1<<17);			// Enable the clock for the UART2
	GPIOA->MODER  &= ~(0x3<<4); 		// Clear the two bits for PA2
	GPIOA->MODER  |=  (0x2<<4); 		// Set the mode to AF
	GPIOA->AFR[0] &= ~(0xF<<8); 		// Clear the 4 bits for PA2
	GPIOA->AFR[0] |=  (0x7<<8);
	USART2->BRR= 0x8B;					// Set baud rate register to 0x8B (115200 baud @ 16MHz)
	USART2->CR1 |= (1<<3);				// Enable Transmit
	USART2->CR1 |= (1<<2);				// Enable Receive
	USART2->CR1 |= (1<<13);				// USART Enable = bit 13 -> enable = 0b1
}

void WaveformCalculations(void){
	double radian = 2 * PI / Wave_length;											// Calculate a Radian value based on Wave_length
	int halfCycles = Cycles / 2;										 			// Calculate Half_Cycles for other calculations
	float TriangleSlope = Cycles / (Wave_length / 2);								// Calculate slope of Triangle
	float SawSlope = Cycles / Wave_length;											// Calculate slope of Sawtooth

	for (int i=0 ; i<Wave_length ; i++){
		Sine[i] = ((sin(radian * i)) * halfCycles) + halfCycles;					// Calculate Sine Wave values
		Sawtooth[i] = SawSlope * i;													// Calculate Sawtooth values

		if (i<=25){
			Square[i] = 0;															// Calculate Square Wave values
			Triangle[i] = (TriangleSlope * i) + halfCycles;							// Calculate rising part of Triangle
		} else if (i<=50){
			Square[i] = 0;															// Calculate Square Wave values
			Triangle[i] = ((-TriangleSlope) * i) + (3 * halfCycles);				// Calculate falling part of Triangle
		} else if (i<=75){
			Square[i] = Cycles;														// Calculate Square Wave values
			Triangle[i] = ((-TriangleSlope) * i) + (3 * halfCycles);				// Calculate falling part of Triangle
		} else {
			Square[i] = Cycles;														// Calculate Square Wave values
			Triangle[i] = (TriangleSlope * i) - (3 * halfCycles);					// Calculate rising part of Triangle
		}
	}
}

void myPrint(char msg[]){
	uint8_t index = 0;
	while(msg[index] != 0){			// While there is a char in msg buffer
		USART2_Tx(msg[index++]);	// Send the char via USART2_Tx
	}
}

void USART2_Tx(int c){
	while (!(USART2->SR & (1<<7))) {};		// Wait until Tx buffer is empty
	USART2->DR = (c & 0xFF);		// Send 'character' input to USART Data Register
}

void delayMs(int n) {
    int i;

    /* Configure SysTick */
    SysTick->LOAD = 16000;  /* reload with number of clocks per millisecond */
    SysTick->VAL = 0;       /* clear current value register */
    SysTick->CTRL = 0x5;    /* Enable the timer */

    for(i = 0; i < n; i++) {
        while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUNTFLAG is set */
            { }
    }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}

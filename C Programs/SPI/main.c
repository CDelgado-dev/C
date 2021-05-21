//NOTE PINS ARE PC10 = SCK ,PC11 = ~LDAC, PC12 = MOSI, PD2 = SS
//
//
#include "stm32f4xx.h"
#include <stdio.h>
#include <math.h>

void GPIOA_init(void);
void TIM2_init(int a);
void SysTick_init(void);
void button_init(void);
void ADC1_init(void);
void myprint(char msg[]);
void USART2_init(void);
void USART2_Tx(char c);
void SPI3_init(void);
void SP3_send(unsigned short a);
void WaveformCalculations(void);

char txt[1000];
int WaveformArrays[4][1000];
int counter = 0;
int count = 0;
int threshold = 500;
int SampleSize = 1000;
int waveform = 0;

int main(void){

	__disable_irq();

	WaveformCalculations(); // Calculations for calculating arrays
	GPIOA_init();//enable A0 as analog input
	TIM2_init(1000);//1000us = 1ms cycle of each wavelength
	SysTick_init(); //Set up Systick for SPI interfacing with waveform arrays
	button_init();//Button init to cycle between the 4 waveforms
	ADC1_init();//enables clock, sets SQR
	USART2_init();//Init Usart for later use in myprint function
	SPI3_init();//

	__enable_irq();

	while(1){
	}

}

void SysTick_Handler(void){
	SP3_send(WaveformArrays[waveform][count++]);
	count = count%1000;
	//GPIOA->ODR |= (1<<5);			// Turn on on-board LED *Used to check if handler was being called
}

void ADC_IRQHandler(void){
	float result = (ADC1->DR & 0xfff)*3.3/1024;//Vout in Volts
		sprintf(txt, "$%.02f;",  result);
		myprint(txt);
		ADC1->SR &= ~2;//clear interrupt flag
}

void EXTI15_10_IRQHandler(void){

	counter = 0;				//reset counter
	count=0;					//reset count
	waveform++;					//increase position
	waveform = waveform%4;     //reset after reaching 4
	EXTI->PR = 0x2000;         //clear interrupt pending flag
}

void myprint(char msg[]){
	uint8_t idx=0;
	while(msg[idx]!='\0' ){
		USART2_Tx(msg[idx++]);
	}
}

void button_init(void){

	RCC->APB2ENR|=1<<14;//enable communication with SYSCFG
	SYSCFG->EXTICR[3] &= ~0x00F0;//clear port selection for EXTI3
	SYSCFG->EXTICR[3] |= 0x0020;//select port C for EXTI13

	EXTI->IMR|=0x2000;//unmask EXTI13
	EXTI->RTSR|=0x2000;//select rising edge trigger
}

void SysTick_init(void){
	SysTick->LOAD = 0xFFF;//using 16MHz clock from AHB gives 1 microsecond
	SysTick->VAL = 0;/* clear current value register */
	SysTick->CTRL = 0x7;/* Enable the timer, set clock source to system clock, clear count flag, and enable interrupt*/
}

void TIM2_init(int a){

	RCC->APB1ENR |= 1;//enable communication with TIM2
	TIM2->CR1&= ~1;//1. disable timer
	TIM2->PSC = 16-1;//no clock division
	TIM2->ARR = a;//random initial value
	TIM2->CNT = 0;//clear counter
	TIM2->CR2 |= 2<<4;//set MMS bits to UPDATE MODE

	TIM2->CR1|=1;//enable timer
	NVIC_SetPriority(EXTI15_10_IRQn, 43);//edge must take precedence over ADC
	NVIC_SetPriority(ADC_IRQn, 44); //ADC set priority after EXTI
	NVIC_EnableIRQ(ADC_IRQn); //Enable ADC interrupt
	NVIC_EnableIRQ(EXTI15_10_IRQn);//enable interrupt handler for PC13
}

void SPI3_init(void){
	RCC->APB1ENR |= 1<<15; //enable SPI comm
	RCC->AHB1ENR |= 3<<2;//enable comm with GPIOC and GPIOD

	//set PC10 and PC12 to AF6
	GPIOC->MODER &= ~((63)<<20);//clear pin modes of PC10,11,12
	GPIOC->MODER |= 0x22<<20;//set PC10/PC12 to alt func

	GPIOC->AFR[1] |= 0x606<<8;//set alt func of PC10/PC12 to AF6

	//set PD2 and PC11 to output
	GPIOD->MODER &= ~(3<<4);//clear pinmode of PD2
	GPIOD->MODER|= (1<<4);//set PD2 to output
	GPIOC->MODER |= (1<<22);

	SPI3->CR1|=1<<2;//set to master mode
	SPI3->CR2|=1<<2;//set to single master mode

	SPI3->CR1|=1<<11;//set DFF to 16 bit frame
	SPI3->CR1 &= ~(3);//clock polarity = 0, clock phase = 0
	SPI3->CR1 |= 1<<6;//enable SPI3

	GPIOC->ODR |= (1<<11);//set ~LDAC high
}

void SP3_send(unsigned short a){

	while(!(SPI3->SR & 2)){}//wait for transmit buffer empty
	GPIOD->ODR&=~(1<<2);//set slave select low
	SPI3->DR = (0x3000)|(a<<2);
	while(!(SPI3->SR & 2)){}
	while(SPI3->SR & 0x80){}
	GPIOD->ODR |=(1<<2);//pull slave select back high
	GPIOC->ODR &=~(1<<11);//pull ~LDAC low
}


void GPIOA_init(void){
	RCC->AHB1ENR|=1;//enable communication w/GPIOA
	GPIOA->MODER |= (1<<10);// Enable A5 for led
	GPIOA->MODER &= ~3;//clear pinmode of pin 0
	GPIOA->MODER|=3;//set pinmode to analog

}

void USART2_init (void){
	//enable peripheral clocks
	RCC->AHB1ENR|=1;//enable GPIOA clock
	RCC->APB1ENR|=0x20000;//enable USART2 clock

	//configure PA3 for USART2 RX
	GPIOA->AFR[0] &= ~0xF000;//clear mode of PA3
	GPIOA->AFR[0]|=0x7000;//alternate function 7 for USART2 on PA3
	GPIOA->MODER &= ~0x00C0;//clear pin mode of PA3
	GPIOA->MODER|=0x0080;//set mode of PA3 to alternate function


	//configure PA2 for USART2 TX
	GPIOA->AFR[0]&= ~0x0F00;//clear mode of PA2
	GPIOA->AFR[0]|= 0x0700;//alternate function 7 for USART2 on PA2
	GPIOA->MODER &= ~0x0030;//clear pin mode of PA2
	GPIOA->MODER|= 0x0020;//set mode of PA2 to alternate function

	//configure USART2 receiving
	USART2->BRR = 0x008B;//115200 baud at 16 MHz, oversampling 16
	USART2->CR1= 0x0004;//enable Rx, 8-bit data
	USART2->CR2 = 0x0000;//one stop bit
	USART2->CR3 = 0x0000;//no flow control

	//configure USART2 transmission
	USART2->CR1|=8;//enable Tx, still 8 bit data
	USART2->CR1|=0x2000;//enable USART2
}

void USART2_Tx(char c){
	while(!(USART2->SR & 0x0080)){}//wait until Tx buffer is empty
	USART2->DR = c;
}

void ADC1_init(){

	RCC->APB2ENR|= 1<<8;//enable communication to ADC1

	ADC1->SQR1 &=~(15<<20);//one channel on regular group
	ADC1->SQR3 &= ~(31);//PA0 in regular group

	ADC1->CR2 &= ~2;//single mode
	ADC1->CR2 &= ~(3<<28);//clear bits
	ADC1->CR2 |= 1<<28;	// enable external trigger on rising edge for regular group
	ADC1->CR2 &= ~(15<<24);//clear bits
	ADC1->CR2 |= 6<<24;//set EXTSEL to TRGO mode from TIM2

	ADC1->CR1|=1<<5;//enable interrupt on EOC flag
	ADC1->CR1 |= 1<<24;//10 bits of resolution
	ADC1->CR1&= ~(1<<10);//clear JAUTO bit

	ADC1->CR2 |=1;//enable ADC

}

void WaveformCalculations(void){

	int j = 0;
	int i =1;
	while(j<SampleSize){//Sine calculations
		WaveformArrays[0][j] = ((1.65*(1 + sin(2*3.14*j/SampleSize)))/3.3)*1024;
		j++;
	}

	j = 0;
	while(j<SampleSize){//Square calculations
		if(j<SampleSize/2){
			WaveformArrays[1][j] = 1023;
		}else{
			WaveformArrays[1][j] = 0;
		}
		j++;
	}
	j=0; //Triangle calculations
	while(j<SampleSize){
	if(j < SampleSize/2){
		WaveformArrays[2][j]= j*2;
		j++;
	}else{
		WaveformArrays[2][j]= (j*2) - (4*i) ;
		i++;
		j++;
	}
}
	j = 0;
	while(j<SampleSize){//SawTooth calculations
		WaveformArrays[3][j] = j;
		j++;

	}
}

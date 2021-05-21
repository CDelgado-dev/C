#include "stm32f4xx.h"

void sysTick_init(void);//
void GPIOB_init(void);//
void SysTick_Handler(void);//
void USART2_IRQHandle(void);//
void USART2_init(void);//
void USART2_write(char c);//
char USART2_read(void);//
void LED_init(void);//
void LED_Blink(void);//
void button_push(void);//
char Name[14]="Cesar Delgado ";
int counter = 0;
int main(void) {
    __disable_irq();                /* global disable IRQs */

    USART2_init();//Set up USART2
    LED_init();//Set up LED
    sysTick_init();//Set up SysTick
    GPIOB_init(); // set up GPIOB

    RCC->AHB1ENR |= 4; //Enable GPIOC
    RCC->APB2ENR |=0x4000; //enable SYSCFG clock
    GPIOC->MODER &= ~(3<<26);//clear bits

    button_push(); // set up button
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
    __enable_irq();                 /* global enable IRQs */

    while(1) {
    }
}

void button_push(void){
	SYSCFG->EXTICR[3] &= ~0x00F0;       /* clear port selection for EXTI13 */
	SYSCFG->EXTICR[3] |=  0x0020;        /* select port C for EXTI13 */
	EXTI->IMR |= 0x2000;                /* unmask EXTI13 */
	EXTI->RTSR |=0x2000;               /* select rising edge trigger */

}

void sysTick_init(void){
	  SysTick->LOAD = 16000000-1;     /* reload with number of clocks per second */
	  SysTick->VAL = 0;
	  SysTick->CTRL = 7;              /* enable SysTick interrupt, use system clock */

}

char USART2_read(void){
	while (!(USART2->SR & 0x0020)) {
	} //wait until char arrives
	return USART2->DR; //causes flag saying it has been read
}

void USART2_write(char c){
	while (!(USART2->SR & 0x0080)) {
	} //wait until Tx buffer is empty
	USART2->DR = c;
}

void GPIOB_init(void){

	GPIOB->MODER &= ~0xFFFFFF; //clearing bits
	GPIOB->MODER |=  0x155555; // Initialize GPIOB pins 0-10
}


void USART2_init(void){

	RCC->AHB1ENR |= 1; //enable GPIOA clock
	RCC->APB1ENR |= 0x20000; //enable USART2 clock
	RCC->AHB1ENR |= 2;//Enable GPIOB

	GPIOA->AFR[0] &= ~0xF000; //clear mode of PA3
	GPIOA->AFR[0] |= 0x7000; //alternate function 7 for USART2 on PA3
	GPIOA->MODER &= ~0x00C0; //clear pin mode of PA3
	GPIOA->MODER |= 0x0080; //set mode of PA3 to alternate function


	GPIOA->AFR[0] &= ~0x0F00; //clear mode of PA2
	GPIOA->AFR[0] |= 0x0700; //alternate function 7 for USART2 on PA2
	GPIOA->MODER &= ~0x0030; //clear pin mode of PA2
	GPIOA->MODER |= 0x0020; //set mode of PA2 to alternate function


	USART2->BRR = 0x008B; //115200 baud at 16 MHz, oversampling 16
	USART2->CR1 = 0x0004; //enable Rx, 8-bit data
	USART2->CR2 = 0x0000; //one stop bit
	USART2->CR3 = 0x0000; //no flow control

	//configure USART2 transmission
	USART2->CR1 |= 8; //enable Tx, still 8 bit data
	USART2->CR1 |=1<<5; //184 enable interupt
	USART2->CR1 |= 0x2000; //enable USART2
}
void LED_init(void){
	RCC->AHB1ENR |= 1; //enable GPIOA clock
	GPIOA->MODER &= ~0x00000C00; //clear pin
    GPIOA->MODER |=  0x00000400;//set to ouput
    GPIOA->BSRR = (1<<21);//turn LED off
}

void LED_Blink(void){
	GPIOA->ODR^=0x20; //LED Toggle
}

void SysTick_Handler(void){
	  LED_Blink(); //Toggle LED
}
void USART2_IRQHandler(void){ // Only turns on the corresponding LED and turns the rest of them off by clearing bits before setting on
	char c = USART2_read();

	if(c == '0'){
		//GPIOB -> ODR &=~(0x7FF);//clear all before setting
		GPIOB -> ODR |=1; //PB0

	}else if(c == '1'){
		//GPIOB -> ODR &=~(0x7FF);//clear all
		GPIOB -> ODR |=0x2;//PB1

	}else if(c == '2'){
		//GPIOB -> ODR &=~(0x7FF);//clear all
		GPIOB -> ODR |=0x4;//PB2

	}else if(c == '3'){
		//GPIOB -> ODR &=~(0x7FF);//clear all
		GPIOB -> ODR |=0x10;//PB4

	}else if(c == '4'){
		//GPIOB -> ODR &=~(0x7FF);//clear all
		GPIOB -> ODR |=0x20;//PB5

	}else if(c == '5'){
		//GPIOB -> ODR &=~(0x7FF);//clear all
		GPIOB -> ODR |=0x40;//PB6

	}else if(c == '6'){
		//GPIOB -> ODR &=~(0x7FF);//clear all
		GPIOB -> ODR |=0x80;//PB7

	}else if(c == '7'){
		//GPIOB -> ODR &=~(0x7FF);//clear all
		GPIOB -> ODR |=0x100;//PB8

	}else if(c == '8'){
		//GPIOB -> ODR &=~(0x7FF);//clear all
		GPIOB -> ODR |=0x200;//PB9

	}else if(c == '9'){
		//GPIOB -> ODR &=~(0x7FF);//clear all
		GPIOB -> ODR |=0x400;	//PB10
	}else{
		GPIOB -> ODR &=~(0x7FF); //Turn off all outputs

	}
}
void EXTI15_10_IRQHandler(void) {
	while (counter < 14){ //Iterate through name
		USART2_write(*(Name+counter++));
	}
	counter = counter % 14;

	EXTI->PR = 0x2000;
}

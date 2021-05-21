#include "stm32f4xx.h"

void USART2_init(void);
void USART2_write(char c);
char USART2_read(void);
void LED_READ(void);

int main(void){
	RCC->AHB1ENR |= 1 << 2; //enable GPIOC clock

	GPIOC->MODER &= ~(3 << 12); //6
	GPIOC->MODER &= ~(3 << 16); //8
	GPIOC->MODER &= ~(3 << 10); //5
	GPIOC->MODER &= ~(3 << 26); //13--this also sets it to input mode since input mode = 00

	/*setting the pin modes to outputs */
	GPIOC->MODER |= 1 << 12; //6
	GPIOC->MODER |= 1 << 16; //8
	GPIOC->MODER |= 1 << 10; //5

	//initialize USART
	USART2_init();

	char Name[14] = "Cesar Delgado "; //char array
	char count = 0; // counter
	char pushed = 0;
	//char c;
	//poll CPU for conditions
	while (1) {

		if (pushed == 1) {
			USART2_write(*(Name + (count++)));
			pushed = 0;
			while (!(GPIOC->IDR & (1 << 13))) {
			} //waiting for the button to be released
			pushed = 0;
			count = count % 14;  //monitor where in name you are
		}
		LED_READ();

		if (!(GPIOC->IDR & (1 << 13))) {// latch for button press
			pushed = 1; //latch input
		}

	}

}

void USART2_init(void){

	RCC->AHB1ENR |= 1; //enable GPIOA clock
	RCC->APB1ENR |= 0x20000; //enable USART2 clock

	//configure PA3 for USART2 RX
	GPIOA->AFR[0] &= ~0xF000; //clear mode of PA3
	GPIOA->AFR[0] |= 0x7000; //alternate function 7 for USART2 on PA3
	GPIOA->MODER &= ~0x00C0; //clear pin mode of PA3
	GPIOA->MODER |= 0x0080; //set mode of PA3 to alternate function

	//configure PA2 for USART2 TX
	GPIOA->AFR[0] &= ~0x0F00; //clear mode of PA2
	GPIOA->AFR[0] |= 0x0700; //alternate function 7 for USART2 on PA2
	GPIOA->MODER &= ~0x0030; //clear pin mode of PA2
	GPIOA->MODER |= 0x0020; //set mode of PA2 to alternate function

	//configure USART2 receiving
	USART2->BRR = 0x008B; //115200 baud at 16 MHz, oversampling 16
	USART2->CR1 = 0x0004; //enable Rx, 8-bit data
	USART2->CR2 = 0x0000; //one stop bit
	USART2->CR3 = 0x0000; //no flow control

	//configure USART2 transmission
	USART2->CR1 |= 8; //enable Tx, still 8 bit data

	USART2->CR1 |= 0x2000; //enable USART2
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

void LED_READ(void){

	if (USART2->SR & 0x20) {
		char c = USART2_read();
		if (c == 0x72 || c == 0x52) { //red
			GPIOC->ODR &= ~(1 << 6); //clearing pins C6, c5, c8
			GPIOC->ODR &= ~(1 << 8);
			GPIOC->ODR &= ~(1 << 5);
			GPIOC->ODR |= (1 << 8); //setting C8 high
		} else if (c == 0x67 || c == 0x47) { //green
			GPIOC->ODR &= ~(1 << 6); //clearing pins C6, c5, c8
			GPIOC->ODR &= ~(1 << 8);
			GPIOC->ODR &= ~(1 << 5);
			GPIOC->ODR |= (1 << 6); //set C6 high
		} else if (c == 0x62 || c == 0x42) { //blue
			GPIOC->ODR &= ~(1 << 6); ////clearing pins C6, c5, c8
			GPIOC->ODR &= ~(1 << 8);
			GPIOC->ODR &= ~(1 << 5);
			GPIOC->ODR |= (1 << 5); //setting C5 high
		} else {
			GPIOC->ODR &= ~(1 << 6); //clearing pins C6, c5, c8 when r,g,b,R,G,B are not received
			GPIOC->ODR &= ~(1 << 8);
			GPIOC->ODR &= ~(1 << 5);
		}
	}
}

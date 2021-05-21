#include "stm32f4xx.h"

void delay_ms(uint32_t n);// takes in ms delay
int main(void){
// Enable the GPIOA clock
	RCC->AHB1ENR |= 0x1; //GPIOA is Enabled

// Configure pin A5
	//GPIO->MODER &= ~(3<<10);
	GPIOA->MODER &= 0xfffff3ff;
	GPIOA->MODER |= 1<<10;

// Toggle GPIO A5
	GPIOA->ODR ^= (0x1<<5); // turning LD2 on
	GPIOA->ODR ^= (0x1<<5); // turning LD2 off
	GPIOA->ODR ^= (0x1<<5); // turning LD2 on
	GPIOA->ODR ^= (0x1<<5); //turning LD2 off

while (1){

	GPIOA->ODR ^= (0x1<<5); // turning LD2 on
	delay_ms(500);
	GPIOA->ODR ^= (0x1<<5); // turning LD2 off
	delay_ms(500);

}

}

// delay function
void delay_ms (uint32_t n){ // delay function
 int i;
 for(; n > 0; n--)// takes the ms value specified in this case 500ms
 for(i = 0; i < 1597; i++); // time it takes to switch between 1ms



}



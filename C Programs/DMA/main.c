

#include "stm32f4xx.h"
#include <stdio.h>
#include <math.h>


void GPIOA_init(void);
void DMA2_init(void);
void DMA2_Stream0_setup(unsigned int src, unsigned int dst, int len);
void DMA1_init(void);
void DMA1_Stream6_setup(unsigned int src, unsigned int dst, int len);
void TIM2_init(int a);
void ADC1_init(void);
void USART2_init(void);
void delay(uint32_t n);


#define ADCBUFSIZE 128

int done = 0;

char buf[ADCBUFSIZE];
char buf2[ADCBUFSIZE];
char uartbuf[ADCBUFSIZE * 6];   /* buffer to hold ASCII numbers for display */
int i = 0;
int buff1=0;
int buff2=0;

int main(void){




	USART2_init();
	DMA2_init();
	DMA1_init();
	GPIOA_init();
	ADC1_init();
	TIM2_init(3200);


	while(1){

		done = 1;
		DMA2_Stream0_setup(buf, (uint32_t)&(ADC1->DR), ADCBUFSIZE);
		done=0;
		while (done == 0) {}     /* wait for ADC DMA transfer complete */
		if ( DMA2_Stream0->CR & (1 << 19) == 0) { //checks CT to change between the 2 buffers
			uint32_t p = uartbuf;
			for (i = 0; i < ADCBUFSIZE - 1; i++) {
				float result = (float) buf[i] * 3.30 / 255;
				sprintf(p, "$%.02f;", result);
				p += 6;
			}
		} else {
			uint32_t p = uartbuf;
			for (i = 0; i < ADCBUFSIZE - 1; i++) {  //second buffer when trigger ct is 1
				float result = (float) buf2[i] * 3.30 / 255;
				sprintf(p, "$%.02f;", result);
				p += 6;
			}
		}
			DMA1_Stream6_setup((uint32_t)uartbuf, (uint32_t)&(USART2->DR), sizeof(uartbuf));
		while(done==0){};
	}

}
void DMA1_Stream6_IRQHandler(void)
{

    DMA1->HIFCR = 0x003F0000;       /* clear all interrupt flags of Stream 6 */
    //DMA2_Stream0->CR &= 1<<19;  	/* ADC1_0 on DMA2 Stream0 Channel 0 */
    DMA1_Stream6->CR &= ~0x10;      /* disable DMA1 Stream 6 TCIE */
    done =1;

}

void DMA2_Stream0_IRQHandler(void)
{
	done = 1;
    DMA2_Stream0->CR = 0;           /* disable DMA2 Stream 0 */
    //DMA2_Stream0->CR ^= 1<<19;  	/* ADC1_0 on DMA2 Stream0 Channel 0 */
    DMA2->LIFCR = 0x003F;           /* clear DMA2 interrupt flags */
    ADC1->CR2 &= ~0x0100;           /* disable ADC conversion complete DMA */
    TIM2->CR1 &= ~1;                /* disable timer2 */

}

void GPIOA_init(void){
	RCC->AHB1ENR|=1;//enable communication w/GPIOA

	//set PA0 as analog input
	GPIOA->MODER &= ~3;//clear pinmode of pin 0
	GPIOA->MODER|=3;//set pinmode to analog
}


void DMA2_init(void) {
    RCC->AHB1ENR |= 0x00400000;     /* DMA2 controller clock enable */
    DMA2->HIFCR = 0x003F;           /* clear all interrupt flags of Stream 0 */
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);  /* DMA interrupt enable at NVIC */
}

void DMA2_Stream0_setup(unsigned int src, unsigned int dst, int len) {

    DMA2_Stream0->CR &= ~1;         /* disable DMA2 Stream 0 */
    while (DMA2_Stream0->CR & 1) {} /* wait until DMA2 Stream 0 is disabled */
    DMA2->HIFCR = 0x003F;           /* clear all interrupt flags of Stream 0 */
    DMA2_Stream0->PAR = dst;
    DMA2_Stream0->M0AR = src;
    DMA2_Stream0->M1AR = buf2;
    DMA2_Stream0->NDTR = len;
   // DMA2_Stream0->CR = 0x00000000;  /* ADC1_0 on DMA2 Stream0 Channel 0 */
    DMA2_Stream0->CR |= 1<<19;
    DMA2_Stream0->CR |= 0x00000400; /* data size byte, mem incr, peripheral-to-mem */
    DMA2_Stream0->CR |= 1<<18;      /* Double buffer enabled to take in both buffers */
    DMA2_Stream0->CR |= 0x16;       /* enable interrupts DMA_IT_TC | DMA_IT_TE | DMA_IT_DME */
    DMA2_Stream0->FCR = 0;          /* direct mode, no FIFO */
    DMA2_Stream0->CR |= 1;          /* enable DMA2 Stream 0 */

    ADC1->CR2 |= 0x0100;            /* enable ADC conversion complete DMA data transfer */
    TIM2->CR1 = 1;                  /* enable timer2 */
    GPIOA->MODER |= 1<<10;			//set pinmode of pin 5 to output
}
void DMA1_init(void) {
    RCC->AHB1ENR |= 0x00200000;     /* DMA controller clock enable */
    DMA1->HIFCR = 0x003F0000;       /* clear all interrupt flags of Stream 6 */
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);  /* DMA interrupt enable at NVIC */
}

void DMA1_Stream6_setup(unsigned int src, unsigned int dst, int len) {
    DMA1_Stream6->CR &= ~1;         /* disable DMA1 Stream 6 */
    while (DMA1_Stream6->CR & 1) {} /* wait until DMA1 Stream 6 is disabled */
    DMA1->HIFCR = 0x003F0000;       /* clear all interrupt flags of Stream 6 */
    DMA1_Stream6->PAR = dst;
    DMA1_Stream6->M0AR = src;
    DMA1_Stream6->NDTR = len;
    DMA1_Stream6->CR = 0x08000000;  /* USART2_TX on DMA1 Stream6 Channel 4 */
    DMA1_Stream6->CR |= 0x00000440; /* data size byte, mem incr, mem-to-peripheral */
    DMA1_Stream6->CR |= 0x16;       /* enable interrupts DMA_IT_TC | DMA_IT_TE | DMA_IT_DME */
    DMA1_Stream6->FCR  = 0;         /* direct mode, no FIFO */
    DMA1_Stream6->CR |= 1;          /* enable DMA1 Stream 6 */

    USART2->SR &= ~0x0040;          /* clear UART transmit complete interrupt flag */
    USART2->CR3 |= 0x80;            /* enable USART2 transmitter DMA */
}
void TIM2_init(int a){
	RCC->APB1ENR |= 1;      //enable communication with TIM2
	TIM2->CR1&= ~1;          //disable timer
	TIM2->PSC = 1;			//no clock division
	TIM2->ARR = a;			//set a initial value
	TIM2->CNT = 0;			//clear counter
	TIM2->CR2 |= 2<<4;		//set MMS bits to UPDATE MODE
}

void ADC1_init(){

	RCC->APB2ENR|= 1<<8;		//enable communication to ADC1

	ADC1->SQR1 &=~(15<<20);		//one channel on regular group
	ADC1->SQR3 &= ~(31);		//PA0 in regular group

	ADC1->CR2 &= ~2;			//single mode set w/CONT bit of CR2
	ADC1->CR2 &= ~(3<<28);
	ADC1->CR2 |= 1<<28;			// enable external trigger on rising edge for regular group
	ADC1->CR2 &= ~(15<<24);
	ADC1->CR2 |= 6<<24;			//set EXTSEL to TRGO mode from TIM2

	ADC1->CR1 |=2<<24;

	ADC1->CR2 |=1;				//enable ADC

}



void USART2_init (void) {
    RCC->AHB1ENR |= 1;          /* Enable GPIOA clock */
    RCC->APB1ENR |= 0x20000;    /* Enable USART2 clock */

    /* Configure PA2, PA3 for USART2 TX, RX */
    GPIOA->AFR[0] &= ~0xFF00;
    GPIOA->AFR[0] |=  0x7700;   /* alt7 for USART2 */
    GPIOA->MODER  &= ~0x00F0;
    GPIOA->MODER  |=  0x00A0;   /* enable alt. function for PA2, PA3 */

    USART2->BRR = 0x008B;       /* 115200 baud @ 16 MHz */
    USART2->CR1 = 0x000C;       /* enable Tx, Rx, 8-bit data */
    USART2->CR2 = 0x0000;       /* 1 stop bit */
    USART2->CR3 = 0x0000;       /* no flow control */
    USART2->CR1 |= 0x2000;      /* enable USART2 */

    USART2->SR = ~0x40; 		//clear TC flag
    USART2->CR1 |= 0x0040;		//enable transmit complete interrupt

    NVIC_EnableIRQ(USART2_IRQn);


}

void delay(uint32_t n) {				 // delay function
	int i;
	for (; n > 0; n--)					 // takes the ms value specified in this case 500ms
		for (i = 0; i < 399; i++); 		// time it takes to switch between 1ms
}

void USART2_IRQHandler(void)
{
    USART2->SR &= ~0x0040;          /* clear transmit complete interrupt flag */
    GPIOA->ODR^=1<<5;
}

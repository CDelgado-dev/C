/*
 * The connections
 * 8 LED    PC6
 * 7 SCLK sck   PA5
 * 6 DN	mosi	PA7
 * 5 D/C    PB6
 * 4 RST    PB10
 * 3 SCE	PA8
 * 2 GND	GND
 * 1 VCC	3.3V
 *
 * The connections
 * I2C1_SCL - PB08
 * I2C1_SDA - PB09
 *
 * Programmed and designed by Cesar Delgado and Zack Rudikoff
 * Past labs and the course textbook were used as references
 */

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>



/////////////////////////////////////
/*      Functions Declaration      */
/////////////////////////////////////
void USART2_init (void);
void I2C1_init(void);
void SPI_init(void);
void TCS34725init();
void GLCD_init(void);
void LED_Nokia_init(void);

///////////////////////
/* Helping Functions */
///////////////////////
void USART2_write (int ch);
void I2C1_byteWrite(uint8_t slave_addr, uint8_t reg_addr, uint8_t reg_data);
uint8_t I2C1_byteRead(uint8_t slave_addr, uint8_t reg_addr);
void SPI_write(unsigned char data);
int16_t red();
int16_t blue();
int16_t green();
void Custom_Nokia_UI(int a);
void GLCD_putchar(int c);
void GLCD_setCursor(unsigned char x, unsigned char y);
void GLCD_clear(void);
void GLCD_data_write(unsigned char data);
void GLCD_command_write(unsigned char data);
void delay(uint32_t n);
void delayMs(int n);
void myprint(char msg[]);

///////////////////////////
/* Variable Declarations */
///////////////////////////
/* define the pixel size of display */
#define GLCD_WIDTH  84
#define GLCD_HEIGHT 48

#define TCS34725_ADDRESS (0x29)     /**< I2C address **/
#define TCS34725_DeviceID (0x12)
#define TCS34725_COMMAND_BIT (0x80) /**< Command bit **/
#define TCS34725_ENABLE (0x00)      /**< Interrupt Enable register */
#define TCS34725_TIMING (0x01)		// RGBC timing register controls
#define TCS34725_WAITTIME (0x03)   //Wait time is set 2.4 ms increments unless the WLONG bit is asserted
#define TCS34725_CONTROLREG (0x0F)  //

#define TCS34725_CDATAL (0x14) /**< Clear channel data low byte */
#define TCS34725_CDATAH (0x15) /**< Clear channel data high byte */
#define TCS34725_RDATAL (0x16) /**< Red channel data low byte */
#define TCS34725_RDATAH (0x17) /**< Red channel data high byte */
#define TCS34725_GDATAL (0x18) /**< Green channel data low byte */
#define TCS34725_GDATAH (0x19) /**< Green channel data high byte */
#define TCS34725_BDATAL (0x1A) /**< Blue channel data low byte */
#define TCS34725_BDATAH (0x1B) /**< Blue channel data high byte */

/* sample font table */
const char font_table[][6] = {
	{0x7e, 0x11, 0x11, 0x11, 0x7e, 0},	/* A  0*/
    {0x7f, 0x49, 0x49, 0x49, 0x36, 0},  /* B  1*/
	{0x3e, 0x41, 0x41, 0x41, 0x22, 0}, 	/* C  2*/
	{0x7f, 0x41, 0x41, 0x43, 0x3C, 0},	/* D  3*/
	{0x7f, 0x49, 0x49, 0x49, 0x49, 0},	/* E  4*/
	{0x7f, 0x09, 0x09, 0x09, 0x09, 0},	/* F  5*/
	{0x7f, 0x41, 0x49, 0x49, 0x79, 0},	/* G  6*/
	{0x7f, 0x08, 0x08, 0x08, 0x7f, 0},	/* H  7*/
	{0x41, 0x41, 0x7f, 0x41, 0x41, 0},	/* I  8*/
	{0x71, 0x41, 0x7f, 0x01, 0x01, 0},	/* J  9*/
	{0x7f, 0x08, 0x14, 0x22, 0x41, 0},	/* K  10*/
	{0x7f, 0x40, 0x40, 0x40, 0x40, 0},	/* L  11*/
	{0x7f, 0x06, 0x7C, 0x06, 0x7f, 0},	/* M  12*/
	{0x7f, 0x06, 0x08, 0x30, 0x7f, 0},	/* N  13*/
	{0x7f, 0x41, 0x41, 0x41, 0x7f, 0},	/* O  14*/
	{0x7f, 0x09, 0x09, 0x09, 0x0F, 0},	/* P  15*/
	{0x3f, 0x41, 0x51, 0x21, 0x5F, 0},	/* Q  16*/
	{0x7f, 0x09, 0x19, 0x29, 0x6F, 0},	/* R  17*/
	{0x4F, 0x49, 0x49, 0x49, 0x79, 0},	/* S  18*/
	{0x01, 0x01, 0x7f, 0x01, 0x01, 0},	/* T  19*/
	{0x7f, 0x40, 0x40, 0x40, 0x7f, 0},	/* U  20*/
	{0x0f, 0x30, 0x40, 0x30, 0x0f, 0},	/* V  21*/
	{0x7f, 0x40, 0x18, 0x40, 0x7f, 0},	/* W  22*/
	{0x63, 0x14, 0x08, 0x14, 0x63, 0},	/* X  23*/
	{0x01, 0x02, 0x7C, 0x02, 0x01, 0},	/* Y  24*/
	{0x61, 0x51, 0x49, 0x45, 0x43, 0}, 	/* Z  25*/
	{0x14, 0x14, 0x14, 0x14, 0x14, 0},	/* = */
	{0, 0, 0, 0, 0, 0},                /*space*/

	{0x7f, 0x49, 0x49, 0x49, 0x36, 0},	/* 0 */
	{0x7f, 0x49, 0x49, 0x49, 0x36, 0},	/* 1 */
	{0x7f, 0x49, 0x49, 0x49, 0x36, 0},	/* 2 */
	{0x7f, 0x49, 0x49, 0x49, 0x36, 0},	/* 3 */
	{0x7f, 0x49, 0x49, 0x49, 0x36, 0},	/* 4 */
	{0x7f, 0x49, 0x49, 0x49, 0x36, 0},	/* 5 */
	{0x7f, 0x49, 0x49, 0x49, 0x36, 0},	/* 6 */
	{0x7f, 0x49, 0x49, 0x49, 0x36, 0},	/* 7 */
	{0x7f, 0x49, 0x49, 0x49, 0x36, 0},	/* 8 */
	{0x7f, 0x49, 0x49, 0x49, 0x36, 0}	/* 9 */

};

//////////////////////////////////
/*         Main Function        */
//////////////////////////////////
int main(void) {
	char txt[256];
	LED_Nokia_init();   /*init Nokia Init*/
    GLCD_init();        /* initialize the GLCD controller */
    GLCD_clear();       /* clear display and home the cursor */

	USART2_init();

	I2C1_init();
	TCS34725init();

	uint8_t test_value = I2C1_byteRead(TCS34725_ADDRESS, (TCS34725_COMMAND_BIT | TCS34725_DeviceID)); // must return 0x44- the ID register
	uint8_t Status = I2C1_byteRead(TCS34725_ADDRESS, (TCS34725_COMMAND_BIT | 0x0F)); //status

    while(1) {

		int16_t R = red();
		int16_t G = green();
		int16_t B = blue();

		//sprintf(txt, "$%d %d %d %d;", R, G, B, C);
		sprintf(txt, "$%d %d %d;", R, G, B);
		myprint(txt);

		if ( R < 31 &&  G < 31  &&  B < 31){ //BLACK
			Custom_Nokia_UI(5);
		}else if(R>200 && G > 200 && B >200){ //WHITE
			Custom_Nokia_UI(4);
		}else if(R >= 170 && G >= 45 && G <= 95 && B >= 30 && B <= 60){ //ORANGE
			Custom_Nokia_UI(7);
		}else if(R >=170 && R<=255 && G >=170 && G<=255){	//YELLOW
			Custom_Nokia_UI(6);
		}else if(R > G && B > G){	//PURPLE
			Custom_Nokia_UI(8);
		}else if(B > R && B > G){ //BLUE
			Custom_Nokia_UI(3);
		}else if(G > B && G >R){ //GREEN
			Custom_Nokia_UI(2);
		}else if (R > B && R > G){	//RED
			Custom_Nokia_UI(1);
		}
    }
}

/////////////////////////
/* Configure Functions */
/////////////////////////
/* initialize USART2 to transmit at 9600 Baud */
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
}

void I2C1_init(void) {
	RCC-> AHB1ENR |= 2; 					/* Enable GPIOB clock */
	RCC->APB1ENR |=  0x00200000;	        /* Enable I2C1 clock */


	GPIOB->MODER    &= ~0x000F0000;         /* PB8, PB9 use alternate function */
	GPIOB->MODER    |=  0x000A0000;
	GPIOB->AFR[1]   &= ~0x000000FF;         /* PB8, PB9 I2C1 SCL, SDA */
	GPIOB->AFR[1]   |=  0x00000044;
	GPIOB->OTYPER   |=  0x00000300;			/* output open-drain */

	I2C1->CR1       =   (1<<15);             /* software reset I2C1 */
	I2C1->CR1       &= ~(1<<15);             /* out of reset */
	I2C1->CR2       =   0x0010;             /* peripheral clock is 16 MHz */
	I2C1->CCR       =   80;                 /* standard mode, 100kHz clock */
	I2C1->TRISE     =   17;                 /* maximum rise time */
	I2C1->CR1       |=  0x0001;             /* enable I2C1 module */
}

void SPI_init(void) {
    RCC->AHB1ENR |= 1;              /* enable GPIOA clock */
    RCC->APB2ENR |= 0x1000;         /* enable SPI1 clock */

    /* PORTA 5, 7 for SPI1 MOSI and SCLK */
    GPIOA->MODER &=   ~0x0000CC00;    /* clear pin mode */
    GPIOA->MODER |=    0x00008800;    /* set pin alternate mode */
    GPIOA->AFR[0] &=  ~0xF0F00000;   /* clear alt mode */
    GPIOA->AFR[0] |=   0x50500000;   /* set alt mode SPI1 */

    /* PORTA 8 as GPIO output for SPI slave select */
    GPIOA->MODER &= ~0x00030000;    /* clear pin mode */
    GPIOA->MODER |=  0x00010000;    /* set pin output mode */

    SPI1->CR1 = 0x31C;				/*Baud rate flck/16*/
    //SPI1->CR1 = 0x364;
    SPI1->CR2 = 0;
    SPI1->CR1 |= 0x40;              /* enable SPI1 module */
}

void TCS34725init(){
	I2C1_byteWrite(TCS34725_ADDRESS, (TCS34725_COMMAND_BIT | TCS34725_ENABLE), 0x01);// Enabled POWER ON,
	delayMs(5);
	I2C1_byteWrite(TCS34725_ADDRESS, (TCS34725_COMMAND_BIT | TCS34725_ENABLE), (0x01 | 0x0A));//Enable RGBC ON, Wait enabled
	delayMs(5);
	I2C1_byteWrite(TCS34725_ADDRESS, (TCS34725_COMMAND_BIT |TCS34725_TIMING), 0xF6); //2.4ms timing
	I2C1_byteWrite(TCS34725_ADDRESS, (TCS34725_COMMAND_BIT |TCS34725_WAITTIME), 0xAB);   //2.4ms timing
	I2C1_byteWrite(TCS34725_ADDRESS, (TCS34725_COMMAND_BIT | TCS34725_CONTROLREG), 0x00);   //1x gain

}

/* send the initialization commands to controller */
void GLCD_init(void) {
    SPI_init();

    /* PORTB 6, 10 as GPIO output for GLCD DC and RESET */
    RCC->AHB1ENR |= 2;              /* enable GPIOB clock */
    GPIOB->MODER &= ~0x00303000;    /* clear pin mode */
    GPIOB->MODER |=  0x00101000;    /* set pin output mode */

    /* hardware reset of GLCD controller */
    GPIOB->BSRR = 0x04000000;       /* assert RESET */
    GPIOB->BSRR = 0x00000400;       /* deassert RESET */

    GLCD_command_write(0x21);       /* set extended command mode */
    GLCD_command_write(0xC0);       /* set LCD Vop for contrast */
    GLCD_command_write(0x07);       /* set temp coefficient */
    GLCD_command_write(0x14);       /* set LCD bias mode 1:48 */
    GLCD_command_write(0x20);       /* set normal command mode */
    GLCD_command_write(0x0C);       /* set display normal mode */
}

void LED_Nokia_init(void){
		RCC->AHB1ENR|=1<<2;
		GPIOC->MODER &= ~(3<<12);//C6
		GPIOC->MODER |= 1<<12;//C6
		GPIOC->ODR |= (1<<6);//set C6 high
}

///////////////////////
/* Helping Functions */
///////////////////////
/* Write a character to USART2 */
void USART2_write (int ch) {
    while (!(USART2->SR & 0x0080)) {}   // wait until Tx buffer empty
    USART2->DR = (ch & 0xFF);
}

// I2C Write Operation
// Input parameters:
//   slave_addr -> slave device id (address is already shifted one place)
//   reg_addr   -> register address inside the slave device that needs to be written
//   reg_data   -> data to be written to the register
//
// This function will
//  1- make sure that the peripheral is not busy
//  2- issue the START condition
//  3- check that you see start flag (meaning that the master can control the SDA)
//  4- Send the slave ID (7bits) along with the intended operation (1bit for read or write)
//  5- check that you see address flag (meaning there is a device with a matching address)
//  6- clear SR1 and SR2  (by reading them)
//  7- check that the data reg is empty
//  8- send reg_addr
//  9- check that the data reg is empty
//  10- send reg_data
//  11- issue the STOP condition
void I2C1_byteWrite(uint8_t slave_addr, uint8_t reg_addr, uint8_t reg_data) {
	  volatile int tmp;

	    while (I2C1->SR2 & 2);                  /* wait until bus not busy */

	    I2C1->CR1 |=  0x100;                    /* generate start */
	    while (!(I2C1->SR1 & 1));               /* wait until start flag is set */

	    I2C1->DR = slave_addr << 1;                  /* transmit slave address */
	    while (!(I2C1->SR1 & 2));               /* wait until addr flag is set */
	    tmp = I2C1->SR2;                        /* clear addr flag */

	    while (!(I2C1->SR1 & 0x80));            /* wait until data register empty */
	    I2C1->DR = reg_addr;                     /* send memory address */

	    while (!(I2C1->SR1 & 0x80));            /* wait until data register empty */
	    I2C1->DR = reg_data;                    /* transmit data */

	    while (!(I2C1->SR1 & 4));               /* wait until transfer finished */
	    I2C1->CR1 |= 0x200;                     /* generate stop */
}

// I2C Read Operation
// Input parameters:
//   slave_addr -> slave device id (address is already shifted one place)
//   reg_addr   -> register address inside the slave device that needs to be read
//  Returned
//   reg_data   -> data from register
//
// This function will
//  1- make sure that the peripheral is not busy
//  2- issue the START condition
//  3- check that you see start flag (meaning that the master can control the SDA)
//  4- Send the slave ID (7bits) along with the intended operation (1bit for read or write)
//  5- check that you see address flag (meaning there is a device with a matching address)
//  6- clear SR1 and SR2  (by reading them)
//  7- check that the data reg is empty
//  8- send reg_addr
//  9- check that the data reg is empty
//  10- issue the RESTART condition
//  11- check that you see start flag (meaning that the master can control the SDA)
//  12- Send the slave ID (7bits) along with the intended operation (1bit for read or write)
//  13- check that you see address flag (meaning there is a device with a matching address)
//  14- Disable Acknowledge
//  15- clear SR1 and SR2  (by reading them)
//  16- check that the receive data reg is not empty
//  17- read the received data and store it into reg_data
//  18- issue the STOP condition
//  19- return reg_data
uint8_t I2C1_byteRead(uint8_t slave_addr, uint8_t reg_addr) {
    uint8_t reg_data;

    volatile int tmp;

        while (I2C1->SR2 & 2);                  /* wait until bus not busy */

        I2C1->CR1 |=  0x100;                    /* generate start */
        while (!(I2C1->SR1 & 1));               /* wait until start flag is set */

        I2C1->DR = slave_addr << 1;              /* transmit slave address + Write */
        while (!(I2C1->SR1 & 2));               /* wait until addr flag is set */
        tmp = I2C1->SR2;                        /* clear addr flag */

        while (!(I2C1->SR1 & 0x80));            /* wait until data register empty */
        I2C1->DR = reg_addr;                       /* send memory address */

        while (!(I2C1->SR1 & 0x80));            /* wait until data register empty */

        I2C1->CR1 |= 0x100;                     /* generate restart */
        while (!(I2C1->SR1 & 1));               /* wait until start flag is set */
        I2C1->DR = slave_addr << 1 | 1;          	/* transmit slave address + Read */

        while (!(I2C1->SR1 & 2));               /* wait until addr flag is set */
        I2C1->CR1 &= ~0x400;                    /* Disable Acknowledge */
        tmp = I2C1->SR2;                        /* clear addr flag */

        I2C1->CR1 |= 0x200;                     /* generate stop after data received */

        while (!(I2C1->SR1 & 0x40));            /* Wait until RXNE flag is set */
        reg_data = I2C1->DR;                     /* Read data from DR */

    return reg_data;
}

void SPI_write(unsigned char data) {
    GPIOA->BSRR = 0x01000000;       /* assert slave select */
    while (!(SPI1->SR & 2)) {}      /* wait until transfer buffer Empty */
    SPI1->DR = data;                /* write command and upper 4 bits of data */
    while (SPI1->SR & 0x80) {}      /* wait for transmission done */
    delay(1);
    GPIOA->BSRR = 0x00000100;       /* de-assert slave select */
}

int16_t red(){
	int16_t red;
	red = I2C1_byteRead(TCS34725_ADDRESS,(TCS34725_COMMAND_BIT |TCS34725_RDATAL));
	return red;
}

int16_t blue(){
	int16_t blue;
	blue = I2C1_byteRead(TCS34725_ADDRESS,(TCS34725_COMMAND_BIT | TCS34725_BDATAL));
	return blue;
}

int16_t green(){
	int16_t green;
	green = I2C1_byteRead(TCS34725_ADDRESS,(TCS34725_COMMAND_BIT | TCS34725_GDATAL));
	return green;
}

void Custom_Nokia_UI(int a){
	int i2;
	for (i2 = 0; i2 < 30; i2++) //empty space
			GLCD_putchar(27);
	GLCD_putchar(27); //empty space
	GLCD_putchar(27); //empty space
	GLCD_putchar(27); //empty space

	if(a == 1){ //red
	GLCD_putchar(17);
	GLCD_putchar(4);
	GLCD_putchar(3);
	GLCD_putchar(27); //empty space
	} else if (a == 2) { //green
		GLCD_putchar(6);
		GLCD_putchar(17);
		GLCD_putchar(4);
		GLCD_putchar(4);
		GLCD_putchar(13);
	} else if (a == 3) {//blue
		GLCD_putchar(1);
		GLCD_putchar(11);
		GLCD_putchar(20);
		GLCD_putchar(4);
	}else if (a == 4){//white
		GLCD_putchar(22);
		GLCD_putchar(7);
		GLCD_putchar(8);
		GLCD_putchar(19);
		GLCD_putchar(4);
	}else if (a == 5){//black
		GLCD_putchar(1);
		GLCD_putchar(11);
		GLCD_putchar(0);
		GLCD_putchar(2);
		GLCD_putchar(10);
	}else if (a == 6){//YELLOW
		GLCD_putchar(24);
		GLCD_putchar(4);
		GLCD_putchar(11);
		GLCD_putchar(11);
		GLCD_putchar(14);
		GLCD_putchar(22);
	}else if (a == 7){//ORANGE
		GLCD_putchar(14);
		GLCD_putchar(17);
		GLCD_putchar(0);
		GLCD_putchar(13);
		GLCD_putchar(6);
		GLCD_putchar(4);
	}else if (a == 8){//PURPLE
		GLCD_putchar(15);
		GLCD_putchar(20);
		GLCD_putchar(17);
		GLCD_putchar(15);
		GLCD_putchar(11);
		GLCD_putchar(4);
	}

	GLCD_clear();
}

void GLCD_putchar(int c) {
    int i;
    for (i = 0; i < 6; i++)
        GLCD_data_write(font_table[c][i]);
}

void GLCD_setCursor(unsigned char x, unsigned char y) {
    GLCD_command_write(0x80 | x);  /* column */
    GLCD_command_write(0x40 | y);  /* bank (8 pixel rows per bank) */
}

/* clears the GLCD by writing zeros to the entire screen */
void GLCD_clear(void) {
    int32_t index;
    for (index = 0 ; index < (GLCD_WIDTH * GLCD_HEIGHT / 8) ; index++)
          GLCD_data_write(0x00);
    	  //GLCD_data_write(0xFF); /*USE WHEN INVERTED*/


      GLCD_setCursor(0, 0); /* return to the home position */
}

/* write to GLCD controller data register */
void GLCD_data_write(unsigned char data) {
    GPIOB->BSRR = 0x00000040;       /* select data register */
    SPI_write(data);                /* send data via SPI */
}

/* write to GLCD controller command register */
void GLCD_command_write(unsigned char data) {
    GPIOB->BSRR = 0x00400000;       /* select command register */
    SPI_write(data);                /* send data via SPI */
}


void delay(uint32_t n) { // delay function
	int i;
	for (; n > 0; n--) // takes the ms value specified in this case 500ms
		for (i = 0; i < 399; i++); // time it takes to switch between 1ms
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

void myprint (char msg[]){
    uint8_t idx=0;
    while(msg[idx]!='\0' ){ USART2_write(msg[idx++]);}
}

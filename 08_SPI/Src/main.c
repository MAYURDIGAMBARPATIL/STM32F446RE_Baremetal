#include "stm32f446xx.h"
/* Define the size of the data */
#define DATA_SIZE      16
#define SPI1EN 		(1<<12)
#define GPIOBEN		(1<<1)

/* Function prototypes */
void GPIO_Init(void);
void SPI1_Init(void);
void SPI1_Transmit(uint8_t *data, uint16_t size);
void SPI1_Receive(uint8_t *data, uint16_t size);
void CS_EN(uint8_t max);
void CS_DS(uint8_t max);
/*uint8_t txData[DATA_SIZE] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
uint8_t rxData[DATA_SIZE] = {0};*/



int main()
{
	GPIO_Init();
	SPI1_Init();
	CS_EN(3);
	SPI1_Transmit(12,1);
	CS_DS(3);


}





void GPIO_Init(void)
{
	//Enable clock access to GPIOB
	RCC->AHB1ENR |= GPIOBEN;

	//PB3--->SCK
	GPIOB->MODER &=~(1<<6);
	GPIOB->MODER |=(1<<7);

	//PB4--->MISO
	GPIOB->MODER &=~(1<<8);
	GPIOB->MODER |=(1<<9);

	//PB5--->MOSI
	GPIOB->MODER &=~(1<<10);
	GPIOB->MODER |=(1<<11);

	//SET AS OUPTPU PB0--->CS1
	GPIOB->MODER |= (1<<0);
	GPIOB->MODER &=~(1U<<1);

	//SET AS OUPTPU PB0--->CS2
	GPIOB->MODER |= (1<<2);
	GPIOB->MODER &=~(1U<<3);

	//SET AS OUPTPU PB0--->CS3
	GPIOB->MODER |= (1<<4);
	GPIOB->MODER &=~(1U<<5);

	//SET PB3, PB4, PB5 ALTERNATE FUNC TYPE TO SPI1

	//PB3
	GPIOB->AFR[0] |=(1<<12);
	GPIOB->AFR[0] &=~(1<<13);
	GPIOB->AFR[0] |=(1<<14);
	GPIOB->AFR[0] &=~(1<<15);

	//PB4
	GPIOB->AFR[0] |=(1<<16);
	GPIOB->AFR[0] &=~(1<<17);
	GPIOB->AFR[0] |=(1<<18);
	GPIOB->AFR[0] &=~(1<<19);

	//PB5
	GPIOB->AFR[0] |=(1<<20);
	GPIOB->AFR[0] &=~(1<<21);
	GPIOB->AFR[0] |=(1<<22);
	GPIOB->AFR[0] &=~(1<<23);
}

void CS_EN(uint8_t max){GPIOB->ODR &= ~(1U<<max);}
void CS_DS(uint8_t max){GPIOB->ODR |= (1U<<max);}



void SPI1_Init(void)
{
	//ENABLE CLOCK ACESS TO SPI1 MODULE
		RCC->APB2ENR |= SPI1EN;
		SPI1->CR1 = 0x31D;											//disable SPI, 8 bit, Master, Mode0 SPI
		SPI1->CR2 |= 0;												//Motorola Frame format
		SPI1->CR1 |= 0x40;
		SPI1->CR1 |= (1<<6);
		GPIOB->ODR |=(1<<4);
		GPIOB->ODR |=(1<<9);

}

void SPI1_Transmit(uint8_t *data, uint16_t size)
{
    uint16_t i;
    for (i = 0; i < size; i++)
    {
        /* Wait until TXE (Transmit buffer empty) flag is set */
        while (!(SPI1->SR & SPI_SR_TXE));

        /* Send data */
        SPI1->DR = data[i];

        /* Wait until RXNE (Receive buffer not empty) flag is set */
        while (!(SPI1->SR & SPI_SR_RXNE));

        /* Read received data to clear RXNE flag */
        (void)SPI1->DR;
    }

    /* Wait until BSY (Busy) flag is reset */
    while (SPI1->SR & SPI_SR_BSY);
}

void SPI1_Receive(uint8_t *data, uint16_t size)
{
    uint16_t i;
    for (i = 0; i < size; i++)
    {
        /* Wait until TXE (Transmit buffer empty) flag is set */
        while (!(SPI1->SR & SPI_SR_TXE));

        /* Send dummy data */
        SPI1->DR = 0xFF;

        /* Wait until RXNE (Receive buffer not empty) flag is set */
        while (!(SPI1->SR & SPI_SR_RXNE));

        /* Read received data */
        data[i] = SPI1->DR;
    }

    /* Wait until BSY (Busy) flag is reset */
    while (SPI1->SR & SPI_SR_BSY);
}

void Error_Handler(void)
{
    while (1)
    {
        /* Stay here */
    }
}

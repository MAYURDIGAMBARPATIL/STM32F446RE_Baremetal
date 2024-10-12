#include "stm32f446xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//some defination

#define GPIOAEN		(1U<<0)
#define UART2EN		(1U<<17)
#define CR1_TE		(1U<<3)
#define CR1_RE		(1U<<2)
#define CR1_UE		(1U<<13)
#define SR_TXE		(1U<<7)
#define SR_RXNE		(1U<<5)
#define SYS_FREQ	16000000
#define	APB1_CLK	SYS_FREQ

#define UART_BAUDRATE 115200


static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t comute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);
void USART_Text_Write_UART2( char *text);
void uart2_rxtx_init(void);
void uart2_write(int ch);
void USART_Text_Read_UART2 (void);
char uart2_read(void);

int i;
char rx_data[5];
char buffer[5];
char *text;

int main()
{
	uart2_rxtx_init();
	USART_Text_Write_UART2("Mayur_patil\r\n");
	while(1)
	{
		while(i<6)
	 {
		  rx_data[i]=uart2_read();
		  i++;
		  if(i==5)
		  {
			strcpy(buffer, rx_data);
			memset(rx_data, 0, 5);
			i=0;
		  }
	}








	}

}
void uart2_rxtx_init(void)
{
	/****************Configure uart gpio pin***************/
	/*Enable Clock acess to gpioa */
	RCC->AHB1ENR |= GPIOAEN;

	/*Set PA2 mode to alternate function mode*/
	GPIOA->MODER &=~(1U<<4);
	GPIOA->MODER |= (1U<<5);

	/*Set PA2 mode to alternate function mode*/
	GPIOA->MODER |=(1U<<7);
	GPIOA->MODER &=~ (1U<<6);


	/*Set PA2 alternate function type to UART_TX (AF7)*/
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &= ~(1U<<11);

	/*Set PA3 alternate function type to UART_RX (AF7)*/
	GPIOA->AFR[0] |= (1U<<12);
	GPIOA->AFR[0] |= (1U<<13);
	GPIOA->AFR[0] |= (1U<<14);
	GPIOA->AFR[0] &= ~(1U<<15);



	/****************Configure uart module ***************/
	/*Enable Clock acess to uart2*/
	RCC->APB1ENR |= UART2EN;

	/*Configure baudrate*/
	uart_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE);

	/*Configure the transfer direction*/
	//USART2->CR1 = (CR1_TE | CR1_RE);

	USART2->CR1 = (CR1_TE | CR1_RE);

	/*Enable uart module*/
	USART2->CR1	|= CR1_UE;
}

char uart2_read(void)
{
	/*Make sure the Receiver data register is empty*/
	while(!(USART2->SR & SR_RXNE)){}

	/* write to transmit data register*/
	return USART2->DR;

}
void USART_Text_Write_UART2( char *text)
{
while(*text) uart2_write(*text++);
}



void uart2_write(int ch)
{
	/*Make sure the transmit data register is empty*/
	while(!(USART2->SR & SR_TXE)){}

	/* write to transmit data register*/
	USART2->DR = (ch & 0xFF);
}



static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
	USARTx->BRR = comute_uart_bd(PeriphClk,BaudRate);
}
static uint16_t comute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
	return ((PeriphClk + (BaudRate/2U))/BaudRate);
}

// PB9	-----SCL
// PB9	-----SDA

#include "stm32f446xx.h"

uint8_t Address;
uint8_t data, read;

void I2C1_init(void);
void I2C_CON_REG(void);
void I2C_Master_tx(void);
void I2C_Master_send_Adress(void);
void I2C_Master_send_Data(uint8_t data);
uint8_t I2C_Master_read_data(void);

void delyMS(int delay);


int main()
{
	Address=0x12;
	Address=Address << 1;
	I2C1_init();
	I2C_CON_REG();
	I2C_Master_tx();
	I2C_Master_send_Adress();
    I2C_Master_send_Data(25);
    read =I2C_Master_read_data();

	//I2C_Master_send_Data(85);
	while(1)
	{


		//I2C_Master_send_Data();
	}

}

void I2C1_init(void)
{
	//ENABLE CLOCK TO I2C & GPIOB
	RCC->AHB1ENR |= (1<<1);
	RCC->APB1ENR |= (1<<21);
	//GPIOB AS ALTERNATE FUNCTION MODE
	GPIOB->MODER |= (1<<17);
	GPIOB->MODER &= ~(1<<16);
	GPIOB->MODER |= (1<<19);
	GPIOB->MODER &= ~(1<<18);
	//GPIOB AS OPEN DRAIN
	GPIOB->OTYPER |= (1<<8);
	GPIOB->OTYPER |= (1<<9);
	//GPIOB AS HIGH SPEED
	GPIOB->OSPEEDR |= (1<<16);
	GPIOB->OSPEEDR |= (1<<17);
	GPIOB->OSPEEDR |= (1<<18);
	GPIOB->OSPEEDR |= (1<<19);
	//GPIOB AS PUSH PULL
	GPIOB->PUPDR &= ~(1<<19);
	GPIOB->PUPDR |=  (1<<18);
	GPIOB->PUPDR &= ~(1<<17);
	GPIOB->PUPDR |=  (1<<16);
	//GPIOB P8 P9 AS I2C AFRH
	GPIOB->AFR[1]|= (1<<2);
	GPIOB->AFR[1]|= (1<<6);
	GPIOB->AFR[1]&= ~(1<<0);
	GPIOB->AFR[1]&= ~(1<<1);
	GPIOB->AFR[1]&= ~(1<<3);
	GPIOB->AFR[1]&= ~(1<<4);
	GPIOB->AFR[1]&= ~(1<<5);
	GPIOB->AFR[1]&= ~(1<<7);
}

void I2C_CON_REG(void)
{
	I2C1->CR1 &= ~(1U<<0);
	I2C1->CR1 |= (1<<15);
	I2C1->CR1 &= ~(1<<15);
	I2C1->OAR1 = 36;
	I2C1->CR2 |= (45<<0);
	I2C1->CCR = 225<<0;
	I2C1->TRISE = 46;
	I2C1->CR1 |= (1U<<0);
}

void I2C_Master_tx(void)
{
	uint16_t reg;
	I2C1->CR1 &=~(1<<11);
	I2C1->CR1 |= (1<<8);
	while(!(I2C1->SR1 & (1<<0))){}
	reg = I2C1->SR1;
}

void I2C_Master_send_Adress(void)
{
	uint16_t reg1;
	I2C1->DR = 0x12<<1;
	while(!(I2C1 -> SR1 & (1<<1))){}
	reg1 = 0x00;
	reg1 = I2C1->SR2;
	while(!(I2C1 -> SR1 & (1<<7))){}
}

void I2C_Master_send_Data(uint8_t data)
{
	while (!(I2C1->SR1 & I2C_SR1_TXE));
	I2C1->DR = data;
	while (!(I2C1->SR1 & I2C_SR1_BTF));
}

uint8_t I2C_Master_read_data(void)
{
	I2C_Master_tx();
	uint16_t reg1;
	I2C1->DR = (0x12<<1) | 1;
	while(!(I2C1 -> SR1 & (1<<1))){}
	reg1 = 0x00;
	reg1 = I2C1->SR2;
	while(!(I2C1 -> SR1 & (1<<7))){}

	while(!(I2C1->SR1 & (I2C_SR1_RXNE))){}
	        data = I2C1->DR;
	while (!(I2C1->SR1 & I2C_SR1_BTF));
	I2C1->CR1 |= I2C_CR1_ACK;     // Enable acknowledge

	        return data;
}

void delyMS(int delay)
{
	int i;
	for( ;delay>0; delay--)
	   for(i= 0; i<=3195; i++);

}

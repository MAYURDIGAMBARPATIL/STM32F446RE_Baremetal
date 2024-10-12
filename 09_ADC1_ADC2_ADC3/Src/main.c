#include "stm32f446xx.h"

void ADC1_init(void);
void ADC1_conversion(void);
int ADC1_read(void);

void ADC2_init(void);
void ADC2_conversion(void);
int ADC2_read(void);

void ADC3_init(void);
void ADC3_conversion(void);
int ADC3_read(void);


void delayMS(int delay);



int ADC1_data, ADC2_data, ADC3_data;

double Temp_LM35_1, Temp_LM35_2, Temp_LM35_3;

int main()
{
	ADC1_init();
	ADC1_conversion();
	ADC2_init();
	ADC2_conversion();
	ADC3_init();
	ADC3_conversion();

	while(1)
	{

		ADC1_data=ADC1_read();
		ADC2_data=ADC2_read();
		ADC3_data=ADC3_read();
		Temp_LM35_1 = ((ADC1_data*(3.3/4095)*1000)/10);    //lm35
		Temp_LM35_2 = ((ADC2_data*(3.3/4095)*1000)/10);    //lm35
		Temp_LM35_3 = ((ADC3_data*(3.3/4095)*1000)/10);    //lm35

	}
}

void ADC1_init(void)
{
	RCC->AHB1ENR |= (1<<0); //GPIOA ENBABLE
	GPIOA->MODER |= (1<<9); // PA0
	GPIOA->MODER |= (1<<8); // PA0

	RCC->APB2ENR |= (1<<8); //ADC1 EN
	//RESOLUTION 12 BIT
	ADC1->CR1 &= ~(1<<24);
	ADC1->CR1 &= ~(1<<25);
	ADC1->CR2 &= ~(1<<0); // ADC DISABLE
	ADC1->SQR3 |= 4;	  //ADC 1th CAHNNEL
	ADC1->CR2 |= (1<<0);  //ADC ON
}

void ADC1_conversion(void)
{
	ADC1->CR2 |= (1<<30); //START CONVERSION
}

int ADC1_read(void)
{
	ADC1->CR2 |= (1<<0);  //ADC ON
	while(!(ADC1->SR & (1<<1))) {} // wait for conversion to be complete
	return ADC1->DR;
	ADC1->CR2 &= ~(1<<0); // ADC DISABLE

}

void ADC2_init(void)
{
	RCC->AHB1ENR |= (1<<0); //GPIOA ENBABLE
	GPIOA->MODER |= (1<<2); // PA0
	GPIOA->MODER |= (1<<3); // PA0

	RCC->APB2ENR |= (1<<9); //ADC2 EN
	//RESOLUTION 12 BIT
	ADC2->CR1 &= ~(1<<24);
	ADC2->CR1 &= ~(1<<25);
	ADC2->CR2 &= ~(1<<0); // ADC DISABLE
	ADC2->SQR3 |= 1;	  //ADC 0th CAHNNEL
	ADC2->CR2 |= (1<<0);  //ADC ON
}

void ADC2_conversion(void)
{
	ADC2->CR2 |= (1<<30); //START CONVERSION
}

int ADC2_read(void)
{
	ADC2->CR2 |= (1<<0);  //ADC ON
	while(!(ADC2->SR & (1<<1))) {} // wait for conversion to be complete
	return ADC2->DR;
	ADC2->CR2 &= ~(1<<0); // ADC DISABLE

}

void ADC3_init(void)
{
	RCC->AHB1ENR |= (1<<0); //GPIOA ENBABLE
	GPIOA->MODER |= (3<<0); // PA0



	RCC->APB2ENR |= (1<<10); //ADC3 EN
	//RESOLUTION 12 BIT
	ADC3->CR1 &= ~(1<<24);
	ADC3->CR1 &= ~(1<<25);
	ADC3->CR2 &= ~(1<<0); // ADC DISABLE
	ADC3->SQR3 |= 0;	  //ADC 0th CAHNNEL
	ADC3->CR2 |= (1<<0);  //ADC ON
}

void ADC3_conversion(void)
{
	ADC3->CR2 |= (1<<30); //START CONVERSION
}

int ADC3_read(void)
{
	ADC3->CR2 |= (1<<0);  //ADC ON
	while(!(ADC3->SR & (1<<1))) {} // wait for conversion to be complete
	return ADC3->DR;
	ADC3->CR2 &= ~(1<<0); // ADC DISABLE

}


void delayMS(int delay)
{
	int i;
	for( ;delay>0; delay--)
	   for(i= 0; i<=3195; i++);

}

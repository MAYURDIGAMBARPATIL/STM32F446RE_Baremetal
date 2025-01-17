#include "stm32f446xx.h"


#define GPIOAEN			(1U<<0)
#define ADC1EN			(1U<<8)
#define SR_EOC				(1<<1)

#define ADC_CH1			(1U<<0)
#define ADC_SEQ_LEN_1		0x00
#define CR2_ADON		(1U<<0)
#define CR2_SWSTART		(1U<<30)
#define CR2_CONT		(1U<<1)
#define CR1_EOCIE		(1U<<5)

int sensor_value;
uint32_t adc_read(void);
void pa1_adc_interrupt_init(void);
void start_converstion(void);
uint32_t adc_read(void);


static void adc_callback(void)
{
	sensor_value =adc_read();

}

void ADC_IRQHandler (void)
{
	 if((ADC1->SR & SR_EOC) !=0)
	 {
		 /*Clear EOC*/
		 ADC1->SR &= ~ SR_EOC;

		 adc_callback();
	 }
}


int main()
{
	pa1_adc_interrupt_init();
	start_converstion();
	//sensor_value=adc_read();

}



void pa1_adc_interrupt_init(void)
{
	/***********Configure the ADC GPIOA Pins*******************/

	/*Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*SET mode of PA1 as analog pin */
	GPIOA->MODER |= (1U<<2);
	GPIOA->MODER |= (1U<<3);

	/***********Configure the ADC Module*******************/
	/*Enable clock access to ADC */
	RCC->APB2ENR |= ADC1EN;

	/*Enable ADC end of conversion interrupt */
	ADC1->CR1 |= CR1_EOCIE;

	/*Enable adc interrupt in nvic*/
	NVIC_EnableIRQ(ADC_IRQn);

	/*Conversion sequence start */
	ADC1->SQR3 = ADC_CH1;

	/*Conversion sequence length */
	ADC1->SQR1 = ADC_SEQ_LEN_1;


	/*Enable ADC module*/
	ADC1->CR2 |= CR2_ADON;
}



void start_converstion(void)
{
	/*Enable contineous conversion*/
	ADC1->CR2 |= CR2_CONT;
	/*Start adc conversion */
	ADC1->CR2 |= CR2_SWSTART;
}

uint32_t adc_read(void)
{
	/*Wait for conversion to be complete*/
	while(!(ADC1->SR & SR_EOC)){}

	/*Read Converted result*/
	return (ADC1->DR);

	ADC1->CR2 &=~(1U<<0);

}

















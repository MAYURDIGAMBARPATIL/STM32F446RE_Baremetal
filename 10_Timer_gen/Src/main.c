#include "stm32f446xx.h"

#define	TIM2EN		(1U<<0)
#define	CR1_CEN		(1U<<0)
#define SR_UIF 		(1U<<0)
#define GPIOAEN		(1<<0)
#define LED_PIN		(1<<5)

void tim2_1hz_init(void)
{
	/*Enable clock access to tim2*/
	RCC->APB1ENR |= TIM2EN;

	/*Set prescaler value*/
	TIM2->PSC = 1600 -1;

	/*Set auto-reload value*/
	TIM2->ARR = 10000-1;

	/*Clear Counter*/
	TIM2->CNT = 0;

	/*Enable timer*/
	TIM2->CR1 = CR1_CEN;
}

int main(void)
{
	RCC->AHB1ENR |= GPIOAEN;
	GPIOA->MODER |=(1U<<10);
	GPIOA->MODER &=~(1U<<11);

	tim2_1hz_init();

	while(1)
	{

			while(!(TIM2->SR & SR_UIF)){}
			TIM2->SR &= ~SR_UIF;
			GPIOA->ODR ^=LED_PIN;
	}
}


#include "stm32f446xx.h"

#define	TIM2EN		(1U<<0)
#define	CR1_CEN		(1U<<0)
#define SR_UIF 		(1U<<0)
#define GPIOAEN		(1<<0)
#define LED_PIN		(1<<5)
#define DIER_UIE	(1U<<0)
int var;
void tim2_1hz_interrupt_init(void)
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
	/*Enable TIM interrupt*/
	TIM2->DIER |= DIER_UIE;

	/*Enalbe TIM interrupt in NVIC*/
	NVIC_EnableIRQ(TIM2_IRQn);
}
void TIM2_IRQHandler(void)
{

				while(!(TIM2->SR & SR_UIF)){}
                       var++;
                       GPIOA->ODR ^=LED_PIN;
						TIM2->SR &= ~SR_UIF;

}
int main(void)
{
	RCC->AHB1ENR |= GPIOAEN;
	GPIOA->MODER |=(1U<<10);
	GPIOA->MODER &=~(1U<<11);

	tim2_1hz_interrupt_init();

	while(1)
	{



	}
}


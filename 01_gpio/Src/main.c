#include "stm32f446xx.h" //standard file for nucleo stm32f446re board

#define GPIOAEN		(1<<0)

int main()
{
	RCC->AHB1ENR |= GPIOAEN;
	GPIOA->MODER |= (1<<10);
	while(1)
	{
		GPIOA->ODR ^= (1<<5);
		for(int i=0; i<1000000; i++){}
	}

}

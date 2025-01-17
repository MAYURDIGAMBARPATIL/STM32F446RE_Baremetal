
//TIMER 2 CHANNEL 1////////////////
#include "stm32f446xx.h"
void delayMS(int delay);//DEFINE DELAY FUNCTION
void PWM_Configure(void);

int main (void){

	PWM_Configure();
	while(1){
		//pwm at pa5
	}
}

void PWM_Configure(void) {
    // PWM 1 Initialization pin PA5 H1
    RCC->AHB1ENR |= (1 << 0);  // Enable GPIOA clock
    RCC->APB1ENR |= (1 << 0);  // Enable TIM2 clock
    GPIOA->MODER |= (2 << 10);
    GPIOA->AFR[0] |= (1 << 20);
    TIM2->PSC = 16 - 1;
    TIM2->ARR = 1000 - 1;
    TIM2->CCR1 = 500;  //PWM DUTY
    TIM2->CCMR1 |= (6 << 4);
    TIM2->CCER |= (1 << 0);
    TIM2->CR1 |= (1 << 0);

    // PWM 2 Initialization pin PA6 H2
    RCC->AHB1ENR |= (1 << 0);
    RCC->APB1ENR |= (1 << 1);
    GPIOA->MODER |= (2 << 12);
    GPIOA->AFR[0] |= (1 << 24);
    TIM3->PSC = 16 - 1;
    TIM3->ARR = 1000 - 1;
    TIM3->CCMR1 |= (6 << 4);
    TIM3->CCER |= (1 << 0);
    TIM3->CR1 |= (1 << 0);

    // PWM 3 Initialization pin PB6 H3
    RCC->AHB1ENR |= (1 << 1);
    RCC->APB1ENR |= (1 << 2);
    GPIOB->MODER |= (2 << 12);
    GPIOB->AFR[0] |= (1 << 24);
    TIM4->PSC = 16 - 1;
    TIM4->ARR = 1000 - 1;
    TIM4->CCMR1 |= (6 << 4);
    TIM4->CCER |= (1 << 0);
    TIM4->CR1 |= (1 << 0);

    // PWM 4 Initialization pin PB14 FAN
    RCC->AHB1ENR |= (1 << 1);
    RCC->APB1ENR |= (1 << 6);
    GPIOB->MODER |= (1 << 29);
    GPIOB->MODER &= ~(1 << 28);
    GPIOB->AFR[1] |= (1 << 24);
    GPIOB->AFR[1] |= (1 << 27);
    GPIOB->AFR[1] &= ~(1 << 26);
    GPIOB->AFR[1] &= ~(1 << 25);
    TIM12->PSC = 16 - 1;
    TIM12->ARR = 1000 - 1;
    TIM12->CCMR1 |= (6 << 4);
    TIM12->CCER |= (1 << 0);
    TIM12->CR1 |= (1 << 0);
}
void delayMS(int delay)
{
	int i;
	for( ;delay>0; delay--)
	   for(i= 0; i<=3195; i++);

}

#include "stm32f446xx.h"

void I2C1_Init(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);

uint8_t rxBuffer[1];
uint8_t txBuffer[1]={0xAB};
volatile uint8_t rxIndex = 0;

int main(void) {
    // Initialize I2C1 in slave mode
    I2C1_Init();

    while (1) {
        // Main loop - can add other functionality here
    }
}



void I2C1_Init(void) {
    // Enable GPIOB and I2C1 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure PB8 and PB9 as alternate function (I2C1 SCL and SDA)
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOB->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
    GPIOB->AFR[1] |= (4 << (4 * (8 - 8))) | (4 << (4 * (9 - 8))); // AF4 for I2C1

    // Configure I2C1
    I2C1->CR1 &= ~I2C_CR1_PE; // Disable I2C1 to configure it
    I2C1->OAR1 = (0x12 << 1); // 7-bit address 0x52
    I2C1->OAR1 |= I2C_OAR1_ADDMODE; // 7-bit address mode
    I2C1->CR1 |= I2C_CR1_ACK;  // Enable ACK
    I2C1->CR2 = 0x10; // PCLK1 frequency in MHz (assuming 16MHz)
    I2C1->CCR = 80; // Configure clock control register for 100kHz
    I2C1->TRISE = 17; // Configure maximum rise time

    // Enable I2C1 and its interrupts
    // Enable I2C1 and interrupts
      I2C1->CR1 |= I2C_CR1_PE;      // Enable peripheral
      I2C1->CR1 |= I2C_CR1_ACK;     // Enable acknowledge
      I2C1->CR2 |= (1<<8);   // Enable error interrupt
      I2C1->CR2 |= (1<<9);   // Enable event interrupt

    // Enable I2C1 event and error interrupts in NVIC
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void I2C1_EV_IRQHandler(void) {
    if (I2C1->SR1 & I2C_SR1_ADDR) {
        // Address matched
        (void)I2C1->SR1; // Clear the address flag by reading SR1
        (void)I2C1->SR2; // Clear the ADDR flag by reading SR2
    }

    if (I2C1->SR1 & I2C_SR1_RXNE) {
        // Received data
        rxBuffer[0] = I2C1->DR; // Read data from the data register
    }

    if (I2C1->SR1 & I2C_SR1_TXE) {
            // Received data
           I2C1->DR=0x32; // Read data from the data register
          }

    if (I2C1->SR1 & I2C_SR1_STOPF) {
        // Stop condition detected
        (void)I2C1->SR1; // Clear the STOP flag by reading SR1
        I2C1->CR1 |= I2C_CR1_PE; // Re-enable the peripheral
        rxIndex = 0; // Reset the buffer index for the next reception
    }
}

void I2C1_ER_IRQHandler(void) {
    // Error handling
    if (I2C1->SR1 & I2C_SR1_AF) {
        I2C1->SR1 &= ~I2C_SR1_AF; // Clear NACK flag
    }

    if (I2C1->SR1 & I2C_SR1_BERR) {
        I2C1->SR1 &= ~I2C_SR1_BERR; // Clear bus error flag
    }

    if (I2C1->SR1 & I2C_SR1_ARLO) {
        I2C1->SR1 &= ~I2C_SR1_ARLO; // Clear arbitration lost flag
    }

    if (I2C1->SR1 & I2C_SR1_OVR) {
        I2C1->SR1 &= ~I2C_SR1_OVR; // Clear overrun/underrun flag
    }

    if (I2C1->SR1 & I2C_SR1_TIMEOUT) {
        I2C1->SR1 &= ~I2C_SR1_TIMEOUT; // Clear timeout flag
    }
}

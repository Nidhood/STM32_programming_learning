#include "stm32f401xe.h"
#include "delay.h"

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~(3UL << (5U * 2U));
    GPIOA->MODER |=  (1UL << (5U * 2U));

    first_project::initializeDelay();

    while (1) {
        GPIOA->ODR ^= (1UL << 5U);
        first_project::delay_s(1U);
    }
}
#pragma once

#include <stdint.h>

#include "stm32f401xe.h"

namespace first_project {

bool initializeDelay(void);

void delay_cycles(uint32_t cycles);
void delay_us(uint32_t time_us);
void delay_ms(uint32_t time_ms);
void delay_s(uint32_t time_s);

} // namespace first_project
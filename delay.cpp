#include "delay.h"

namespace first_project {

namespace {

constexpr uint32_t MAX_SAFE_DELAY_CYCLES = 0x7FFFFFFFUL;

uint32_t cycles_per_us(void) {
    return SystemCoreClock / 1000000UL;
}

uint32_t cycles_per_ms(void) {
    return SystemCoreClock / 1000UL;
}

bool is_dwt_counter_enabled(void) {
    return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
}

} // namespace

bool initializeDelay(void) {
    SystemCoreClockUpdate();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    __DSB();
    __ISB();

    return is_dwt_counter_enabled();
}

void delay_cycles(uint32_t cycles) {
    while (cycles > 0U) {
        const uint32_t current_chunk =
            (cycles > MAX_SAFE_DELAY_CYCLES) ? MAX_SAFE_DELAY_CYCLES : cycles;

        const uint32_t start = DWT->CYCCNT;

        while ((DWT->CYCCNT - start) < current_chunk) {
            __NOP();
        }

        cycles -= current_chunk;
    }
}

void delay_us(uint32_t time_us) {
    const uint32_t cycles = cycles_per_us();

    while (time_us > 0U) {
        delay_cycles(cycles);
        --time_us;
    }
}

void delay_ms(uint32_t time_ms) {
    const uint32_t cycles = cycles_per_ms();

    while (time_ms > 0U) {
        delay_cycles(cycles);
        --time_ms;
    }
}

void delay_s(uint32_t time_s) {
    while (time_s > 0U) {
        delay_ms(1000U);
        --time_s;
    }
}

} // namespace first_project
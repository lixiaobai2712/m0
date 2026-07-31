#include "delay.h"
#include "ti_msp_dl_config.h"

void Delay_Us(uint32_t microseconds)
{
    delay_cycles((uint64_t)CPUCLK_FREQ / 1000000U * microseconds);
}

void Delay_Ms(uint32_t milliseconds)
{
    while (milliseconds-- > 0U) {
        Delay_Us(1000U);
    }
}

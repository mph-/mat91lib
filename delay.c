#include "delay.h"

__attribute__((optimize(2)))
void delay_us (unsigned int us)
{
    DELAY_US (us);
}


void delay_ms (unsigned int ms)
{
    while (ms)
    {
        DELAY_US (1000);
        ms--;
    }
}

/** @file   delay.h
    @author M. P. Hayes, UCECE
    @date   02 June 2007
    @brief 
*/
#ifndef DELAY_H
#define DELAY_H

#include "config.h"
#include "cpu.h"
#include "mcu.h"


#ifdef RAM
#define DELAY_LOOP_CYCLES 4
#else
#define DELAY_LOOP_CYCLES (1 * MCU_FLASH_READ_CYCLES)
#endif


static inline unsigned int 
_delay_us_loops (double delay)
{
    double __tmp1 = ((double)F_CPU * (delay)) / (DELAY_LOOP_CYCLES * 1e6);
    unsigned int __ticks1;
    
    if (__tmp1 < 1.0)
        __ticks1 = 1;
    else
        __ticks1 = (unsigned int)__tmp1;
    
    return __ticks1;
}


#define DELAY_US(us)                            \
do                                              \
{                                               \
    double __tmp1 = ((double)F_CPU * (us)) / (DELAY_LOOP_CYCLES * 1e6); \
    unsigned int __ticks1;                      \
                                                \
    if (__tmp1 < 1.0)                           \
        __ticks1 = 1;                           \
    else                                        \
        __ticks1 = (unsigned int)__tmp1;        \
    mcu_delay_loop (__ticks1);                  \
}                                               \
while (0)


static inline
void delay_ms (int x)
{
    while (x)
    {
        DELAY_US (1000);
        x--;
    }
}


#endif /* DELAY_H  */

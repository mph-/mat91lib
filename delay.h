/** @file   delay.h
    @author M. P. Hayes, UCECE
    @date   02 June 2007
    @brief 
*/
#ifndef DELAY_H
#define DELAY_H

#include "config.h"
#include "cpu.h"

// #ifdef __THUMBEL__

#ifdef RAM_RUN
#define DELAY_LOOP_CYCLES 4
#else
#define DELAY_LOOP_CYCLES (4 * CPU_FLASH_READ_CYCLES)
#endif


static inline void
_delay_loop (unsigned int loops)
{
#ifdef __THUMBEL__
    __asm__ volatile ("\t sub %0, %0, #1;\n\t bcs . - 2" : "=r" (loops) : "0" (loops));
#else
    __asm__ volatile ("\t subs %0, %0, #1;\n\t bcs . - 4" : "=r" (loops) : "0" (loops));
#endif
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
    _delay_loop (__ticks1);                     \
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

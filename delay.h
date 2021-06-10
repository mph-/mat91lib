/** @file   delay.h
    @author M. P. Hayes, UCECE
    @date   02 June 2007
    @brief 
*/
#ifndef DELAY_H
#define DELAY_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"
#include "cpu.h"
#include "mcu.h"


#ifdef RAM
#define DELAY_LOOP_CYCLES 4
#else
#define DELAY_LOOP_CYCLES (1 * MCU_FLASH_READ_CYCLES)
#endif


// Ensure constant folding optimization
__attribute__((optimize (2)))
__always_inline__
static inline unsigned int _delay_us_loops (double delay_us)
{
    double tmp = ((double)F_CPU * (delay_us)) / (DELAY_LOOP_CYCLES * 1e6);
    unsigned int ticks;
    
    if (tmp < 1.0)
        ticks = 1;
    else
        ticks = (unsigned int)tmp;
    
    return ticks;
}

    
__attribute__((optimize (2)))
__always_inline__
static inline void DELAY_US (double delay_us)
{
    unsigned int ticks;                      

    ticks = _delay_us_loops (delay_us);
    mcu_delay_loop (ticks);
}


static inline
void delay_ms (int x)
{
    while (x)
    {
        DELAY_US (1000);
        x--;
    }
}



#ifdef __cplusplus
}
#endif    
#endif /* DELAY_H  */


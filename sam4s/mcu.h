/** @file   mcu.h
    @author M. P. Hayes, UCECE
    @date   13 July 2013
    @brief  System routines for SAM4S processors
*/
#ifndef MCU_H
#define MCU_H

#include "config.h"


#ifndef MCU_FLASH_READ_CYCLES 
#define MCU_FLASH_READ_CYCLES 6
#endif


static inline void
mcu_delay_loop (unsigned int loops)
{
    __asm__ volatile ("\t subs %0, %0, #1;\n\t bcs . - 2" : "=r" (loops) : "0" (loops));
}


void
mcu_init (void);


void
mcu_power_mode_low (void);


void
mcu_power_mode_normal (void);


void
mcu_sleep (void);


void
mcu_udp_disable (void);


void
mcu_udp_enable (void);


void
mcu_watchdog_reset (void);


void
mcu_watchdog_enable (void);


static inline void
mcu_pmc_enable (uint8_t id)
{
    if (id < 32)
        PMC->PMC_PCER0 = BIT (id);
    else
        PMC->PMC_PCER1 = BIT (id - 32);
}


static inline void
mcu_pmc_disable (uint8_t id)
{
    if (id < 32)
        PMC->PMC_PCDR0 = BIT (id);
    else
        PMC->PMC_PCDR1 = BIT (id - 32);
}


void
mcu_cpu_idle (void);


#endif /* MCU_H  */

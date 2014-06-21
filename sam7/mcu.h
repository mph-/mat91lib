/** @file   sys.h
    @author M. P. Hayes, UCECE
    @date   13 July 2013
    @brief  System routines for SAM4S processors
*/
#ifndef MCU_H
#define MCU_H

#include "config.h"


#ifndef MCU_FLASH_READ_CYCLES 
#define MCU_FLASH_READ_CYCLES 2
#endif


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



#endif /* MCU_H  */

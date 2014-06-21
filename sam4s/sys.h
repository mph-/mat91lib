/** @file   sys.h
    @author M. P. Hayes, UCECE
    @date   13 July 2013
    @brief  System routines for SAM4S processors
*/
#ifndef SYS_H
#define SYS_H

#include "config.h"


#ifndef SYS_FLASH_READ_CYCLES 
#define SYS_FLASH_READ_CYCLES 2
#endif


void
sys_init (void);


void
sys_power_mode_low (void);


void
sys_power_mode_normal (void);


void
sys_sleep (void);


void
sys_udp_disable (void);


void
sys_udp_enable (void);


void
sys_watchdog_reset (void);


void
sys_watchdog_enable (void);



#endif /* SYS_H  */

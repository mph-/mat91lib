/** @file   mcu_sleep.h
    @author M. P. Hayes, UCECE
    @date   4 May 2015
    @brief  Sleep routines for SAM4S processors
*/
#ifndef MCU_SLEEP_H
#define MCU_SLEEP_H

#include "config.h"
#include "pio.h"

typedef enum 
{
    MCU_SLEEP_MODE_BACKUP,
    MCU_SLEEP_MODE_WAIT,
    MCU_SLEEP_MODE_SLEEP
} mcu_sleep_mode_t;


typedef struct mcu_sleep_cfg_struct
{
    mcu_sleep_mode_t mode;
    pio_t pio;
    bool active;
} mcu_sleep_cfg_t;


void
mcu_sleep (const mcu_sleep_cfg_t *cfg);


#endif /* MCU_SLEEP_H  */

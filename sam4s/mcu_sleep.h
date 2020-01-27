/** @file   mcu_sleep.h
    @author M. P. Hayes, UCECE
    @date   4 May 2015
    @brief  Sleep routines for SAM4S processors
*/
#ifndef MCU_SLEEP_H
#define MCU_SLEEP_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"
#include "pio.h"

typedef enum 
{
    MCU_SLEEP_MODE_BACKUP,
    MCU_SLEEP_MODE_WAIT,
    MCU_SLEEP_MODE_SLEEP
} mcu_sleep_mode_t;


typedef struct mcu_sleep_wakeup_cfg_struct
{
    pio_t pio;
    bool active_high;
} mcu_sleep_wakeup_cfg_t;


typedef struct mcu_sleep_cfg_struct
{
    mcu_sleep_mode_t mode;
} mcu_sleep_cfg_t;    
    

bool
mcu_sleep_wakeup_set (const mcu_sleep_wakeup_cfg_t *cfg);    

    
void
mcu_sleep (const mcu_sleep_cfg_t *cfg);


#ifdef __cplusplus
}
#endif    
#endif /* MCU_SLEEP_H  */


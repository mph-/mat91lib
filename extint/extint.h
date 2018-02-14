/** @file   extint.h
    @author M. P. Hayes
    @date   15 April 2013
    @brief  External interrupt handling.
*/

#ifndef EXTINT_H
#define EXTINT_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"
#include "pio.h"

typedef struct extint_cfg_struct
{
    pio_t pio;
    void (*handler)(void);
} extint_cfg_t;


typedef struct extint_dev_struct extint_dev_t;

typedef extint_dev_t *extint_t;


extint_t extint_init (const extint_cfg_t *cfg);

void extint_enable (extint_t extint);

void extint_disable (extint_t extint);

void extint_sleep (extint_t extint);


#ifdef __cplusplus
}
#endif    
#endif



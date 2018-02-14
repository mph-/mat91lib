/** @file   ac.h
    @author M. P. Hayes, UCECE
    @date   18 Feb 2015

    @brief Routines to use AT91 onboard analogue comparator.
*/

#ifndef AC_H
#define AC_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"

/** AC channels.  */
typedef enum
{
    AC_CHANNEL_0, AC_CHANNEL_1, AC_CHANNEL_2, AC_CHANNEL_3, 
    AC_CHANNEL_4, AC_CHANNEL_5, AC_CHANNEL_6, AC_CHANNEL_7,
    AC_CHANNEL_NUM
} ac_channel_t;


/** AC channels.  */
typedef enum
{
    AC_REFERENCE_TEMP_SENSOR,
    AC_REFERENCE_EXTERNAL,
    AC_REFERENCE_DAC0,
    AC_REFERENCE_DAC1,
    AC_REFERENCE_0, AC_REFERENCE_1,  AC_REFERENCE_2, AC_REFERENCE_3
} ac_reference_t;


typedef enum
{
    AC_EDGE_RISING,
    AC_EDGE_FALLING,
    AC_EDGE_ANY,
} ac_edge_t;


typedef enum
{
    AC_HYSTERESIS_NONE,
    AC_HYSTERESIS_SMALL,
    AC_HYSTERESIS_LARGE = 3,
} ac_hysteresis_t;


typedef enum
{
    AC_CURRENT_LOW_POWER,
    AC_CURRENT_HIGH_SPEED
} ac_current_t;


typedef struct ac_dev_struct
{
    uint32_t MR;
    uint32_t ACR;
} ac_dev_t;


typedef ac_dev_t *ac_t;


typedef struct ac_cfg_struct
{
    ac_channel_t channel;
    ac_reference_t reference;
    ac_edge_t edge;
    ac_hysteresis_t hysteresis;
    ac_current_t current;
    bool invert;
} ac_cfg_t;


bool
ac_channel_set (ac_t ac, ac_channel_t channel);


bool
ac_reference_set (ac_t ac, ac_reference_t reference);


bool
ac_edge_set (ac_t ac, ac_edge_t edge);


bool
ac_hysteresis_set (ac_t ac, ac_hysteresis_t hysteresis);


bool
ac_current_set (ac_t ac, ac_current_t current);


void
ac_enable (ac_t ac);


void
ac_disable (ac_t ac);


void
ac_irq_enable (ac_t ac);


void
ac_irq_disable (ac_t ac);


bool
ac_poll (ac_t ac);


ac_t 
ac_init (const ac_cfg_t *cfg);


void
ac_shutdown (ac_t ac);


#ifdef __cplusplus
}
#endif    
#endif


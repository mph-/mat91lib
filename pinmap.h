/** @file   pinmap.h
    @author M. P. Hayes, UCECE
    @date   2 Jun 2007
*/
#ifndef PINMAP_H
#define PINMAP_H

#ifdef __cplusplus
extern "C" {
#endif
    


#include "pio.h"

typedef struct pinmap_struct
{
    uint8_t channel;
    pio_t pio;
    pio_config_t periph;
    uint8_t flags;
} pinmap_t;



#ifdef __cplusplus
}
#endif    
#endif


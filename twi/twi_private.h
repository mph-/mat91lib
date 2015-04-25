/** @file   twi.h
    @author M. P. Hayes
    @date   25 April 2015
    @brief  TWI routines for AT91 processors
*/

#ifndef TWI_PRIVATE_H
#define TWI_PRIVATE_H

#include "config.h"

typedef uint8_t twi_id_t;

typedef struct twi_dev_struct
{
    Twi *base;
    twi_id_t slave_addr;
} twi_dev_t;

#endif

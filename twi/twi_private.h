/** @file   twi.h
    @author M. P. Hayes
    @date   25 April 2015
    @brief  TWI routines for AT91 processors
*/

#ifndef TWI_PRIVATE_H
#define TWI_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"

typedef uint8_t twi_slave_addr_t;

typedef enum
{
    TWI_MODE_NODE,
    TWI_MODE_MASTER,
    TWI_MODE_SLAVE
} twi_mode_t;


typedef struct twi_dev_struct
{
    Twi *base;
    twi_slave_addr_t slave_addr;
    twi_mode_t mode;
    uint32_t clock_config;
    uint8_t channel;
} twi_dev_t;


#ifdef __cplusplus
}
#endif    
#endif


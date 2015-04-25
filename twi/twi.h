/** @file   twi.h
    @author M. P. Hayes
    @date   25 April 2015
    @brief  TWI routines for AT91 processors
*/

#ifndef TWI_H
#define TWI_H

#include "config.h"
#include "pio.h"

#define TWI_PERIOD_DIVISOR(FREQ) ((twi_period_t)((F_CPU / 2) / (FREQ)))

typedef enum
{
    TWI_CHANNEL_0,
    TWI_CHANNEL_1,
} twi_channel_t;


typedef enum
{
    TWI_MODE_MASTER,
    TWI_MODE_SLAVE
} twi_mode_t;


typedef uint16_t twi_period_t;


/** TWI configuration structure.  */
typedef struct
{
    uint8_t channel;
    twi_mode_t mode;
    twi_period_t period;         /* Clocks */
} twi_cfg_t;


/** Include definition of data structures required for the driver
    implementation.  Do not use.  */
#include "twi_private.h"


typedef enum twi_ret
{
    TWI_OK = 0,
    TWI_ERROR_TIMEOUT = -1,
    TWI_ERROR_NO_ACK = -2,
} twi_ret_t;


/** Define datatype for handle to TWI functions.  */
typedef twi_dev_t *twi_t;


twi_t 
twi_init (const twi_cfg_t *cfg);


twi_ret_t
twi_master_write (twi_t twi, twi_id_t slave, twi_id_t addr,
                  uint8_t addr_size, void *buffer, uint8_t size);


twi_ret_t
twi_master_read (twi_t twi, twi_id_t slave, twi_id_t addr,
                 uint8_t addr_size, void *buffer, uint8_t size);


void
twi_shutdown (twi_t twi);

#endif







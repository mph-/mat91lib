/** @file   twi.h
    @author M. P. Hayes
    @date   25 April 2015
    @brief  TWI routines for AT91 processors
*/

#ifndef TWI_H
#define TWI_H

#include "config.h"
#include "pio.h"
#include "twi_private.h"

#define TWI_PERIOD_DIVISOR(FREQ) ((twi_period_t)((F_CPU / 2) / (FREQ)))

#ifndef TWI_TIMEOUT_US_DEFAULT
#define TWI_TIMEOUT_US_DEFAULT 1000
#endif


typedef enum
{
    TWI_CHANNEL_0,
    TWI_CHANNEL_1,
} twi_channel_t;


typedef uint16_t twi_period_t;

typedef uint32_t twi_timeout_t;


/** TWI configuration structure.  */
typedef struct
{
    uint8_t channel;
    twi_mode_t mode;
    twi_period_t period;         /* Clocks */
    twi_id_t slave_addr;
} twi_cfg_t;


/** Include definition of data structures required for the driver
    implementation.  Do not use.  */
#include "twi_private.h"


typedef enum twi_ret
{
    TWI_DONE = 3,
    /* Master performing a write, slave a read.  */
    TWI_WRITE = 2,
    /* Master performing a read, slave a write.  */
    TWI_READ = 1,
    TWI_OK = 0,
    TWI_ERROR_TIMEOUT = -1,
    TWI_ERROR_NO_ACK = -2,
    /* Another master got in first.  */
    TWI_ERROR_CONFLICT = -3,
    TWI_ERROR_READ_EXPECTED = -4,
    TWI_ERROR_WRITE_EXPECTED = -5,
    TWI_ERROR_SVACC = -6
} twi_ret_t;


/** Define datatype for handle to TWI functions.  */
typedef twi_dev_t *twi_t;


twi_t 
twi_init (const twi_cfg_t *cfg);


twi_ret_t
twi_master_addr_write_timeout (twi_t twi, twi_id_t slave_addr,
                               twi_id_t addr, uint8_t addr_size,
                               void *buffer, uint8_t size, twi_timeout_t timeout_us);

twi_ret_t
twi_master_addr_write (twi_t twi, twi_id_t slave, twi_id_t addr,
                       uint8_t addr_size, void *buffer, uint8_t size);


/** Write data from slave.  No internal address is sent.  */
twi_ret_t
twi_master_write (twi_t twi, uint8_t addr_size, void *buffer, uint8_t size);


twi_ret_t
twi_master_addr_read_timeout (twi_t twi, twi_id_t slave_addr,
                              twi_id_t addr, uint8_t addr_size,
                              void *buffer, uint8_t size, twi_timeout_t timeout_us);

twi_ret_t
twi_master_addr_read (twi_t twi, twi_id_t slave, twi_id_t addr,
                      uint8_t addr_size, void *buffer, uint8_t size);


/** Read data from slave.  No internal address is sent.  */
twi_ret_t
twi_master_read (twi_t twi, twi_id_t slave, void *buffer, uint8_t size);


twi_ret_t
twi_slave_poll (twi_t twi);


twi_ret_t
twi_slave_read (twi_t twi, void *buffer, uint8_t size);


twi_ret_t
twi_slave_write (twi_t twi, void *buffer, uint8_t size);

void
twi_shutdown (twi_t twi);

#endif







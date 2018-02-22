/** @file   twi.h
    @author M. P. Hayes
    @date   25 April 2015
    @brief  TWI routines for AT91 processors
    This assumes that the slave address is 7-bits.  10-bit addresses can
    be handled using the internal address.
*/

#ifndef TWI_H
#define TWI_H

#ifdef __cplusplus
extern "C" {
#endif
    

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


/* Internal address.  */
typedef uint32_t twi_iaddr_t;

typedef uint16_t twi_period_t;

typedef uint32_t twi_timeout_t;

/* Don't want messages too long that will hog bus.  */
typedef uint16_t twi_size_t;


/** TWI configuration structure.  */
typedef struct
{
    uint8_t channel;             /* The controller channel, 0 or 1.  */
    twi_period_t period;         /* Clocks.  */
    twi_slave_addr_t slave_addr; /* Only required for slave mode.  */
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
    TWI_IDLE = 0,
    TWI_ERROR_TIMEOUT = -1,
    TWI_ERROR_NO_ACK = -2,
    /* Another master got in first.  */
    TWI_ERROR_CONFLICT = -3,
    TWI_ERROR_READ_EXPECTED = -4,
    TWI_ERROR_WRITE_EXPECTED = -5,
    TWI_ERROR_SVACC = -6,
    TWI_ERROR_PROTOCOL = -7,
    TWI_ERROR_NO_STOP = -8
} twi_ret_t;


/** Define datatype for handle to TWI functions.  */
typedef twi_dev_t *twi_t;


twi_t 
twi_init (const twi_cfg_t *cfg);


/** Perform a master write to the specified slave address with internal address
    and timeout
    @param twi TWI controller to use
    @param slave_addr 7 bit slave address
    @param addr optional internal address
    @param addr_size number of bytes for internal address (0--3)
    @param buffer buffer to write from
    @param size number of bytes to transfer
    @param timeout_us timeout in microseconds
    @return number of bytes read or negative value for an error
*/    
twi_ret_t
twi_master_addr_write_timeout (twi_t twi, twi_slave_addr_t slave_addr,
                               twi_iaddr_t addr, uint8_t addr_size,
                               const void *buffer, twi_size_t size,
                               twi_timeout_t timeout_us);


/** Perform a master write to the specified slave address with internal address
    and default timeout
    @param twi TWI controller to use
    @param slave_addr 7 bit slave address
    @param addr optional internal address
    @param addr_size number of bytes for internal address (0--3)
    @param buffer buffer to write from
    @param size number of bytes to transfer
    @return number of bytes read or negative value for an error
*/    
twi_ret_t
twi_master_addr_write (twi_t twi, twi_slave_addr_t slave, twi_iaddr_t addr,
                       uint8_t addr_size, const void *buffer, twi_size_t size);


/** Perform a master write to the specified slave address without internal address
    using default timeout
    @param twi TWI controller to use
    @param slave_addr 7 bit slave address
    @param buffer buffer to write from
    @param size number of bytes to transfer
    @return number of bytes read or negative value for an error
*/
twi_ret_t
twi_master_write (twi_t twi, uint8_t addr_size, const void *buffer, twi_size_t size);


/** Perform a master read to the specified slave address with internal address
    and timeout
    @param twi TWI controller to use
    @param slave_addr 7 bit slave address
    @param addr optional internal address
    @param addr_size number of bytes for internal address (0--3)
    @param buffer buffer to read into
    @param size number of bytes to transfer
    @param timeout_us timeout in microseconds
    @return number of bytes read or negative value for an error
*/    
twi_ret_t
twi_master_addr_read_timeout (twi_t twi, twi_slave_addr_t slave_addr,
                              twi_iaddr_t addr, uint8_t addr_size,
                              void *buffer, twi_size_t size, 
                              twi_timeout_t timeout_us);


/** Perform a master read to the specified slave address with internal address
    and default timeout
    @param twi TWI controller to use
    @param slave_addr 7 bit slave address
    @param addr optional internal address
    @param addr_size number of bytes for internal address (0--3)
    @param buffer buffer to read into
    @param size number of bytes to transfer
    @return number of bytes read or negative value for an error
*/    
twi_ret_t
twi_master_addr_read (twi_t twi, twi_slave_addr_t slave, twi_iaddr_t addr,
                      uint8_t addr_size, void *buffer, twi_size_t size);


/** Perform a master read to the specified slave address without internal address
    and default timeout
    @param twi TWI controller to use
    @param slave_addr 7 bit slave address
    @param buffer buffer to read into
    @param size number of bytes to transfer
    @param timeout_us timeout in microseconds
    @return number of bytes read or negative value for an error
*/
twi_ret_t
twi_master_read (twi_t twi, twi_slave_addr_t slave, void *buffer, twi_size_t size);


/** Poll TWI controller to detect a packet from the master
    @return TWI_WRITE if master write detected,
            TWI_READ if master read detected,
            otherwise TWI_OK
*/    
twi_ret_t
twi_slave_poll (twi_t twi);


/** Perform a slave write with specified timeout
    @param twi TWI controller to use
    @param size number of bytes to transfer
    @param timeout_us timeout in microseconds
    @return number of bytes read or negative value for an error
    @note If there are more bytes to be written than required, 
          they will be ignored
*/
twi_ret_t
twi_slave_write_timeout (twi_t twi, void *buffer, twi_size_t size, 
                         twi_timeout_t timeout_us);


/** Perform a slave read  with specified timeout
    @param twi TWI controller to use
    @param size number of bytes to transfer
    @timeout_us timeout in microseconds
    @return number of bytes read or negative value for an error
    @note If there are more bytes to be read than specified, they will be gobbled
*/
twi_ret_t
twi_slave_read_timeout (twi_t twi, void *buffer, twi_size_t size,
                        twi_timeout_t timeout_us);

/** Perform a slave read with default timeout
    @param twi TWI controller to use
    @param size number of bytes to transfer
    @return number of bytes read or negative value for an error
    @note If there are more bytes to be read than specified, they will be gobbled
*/
twi_ret_t
twi_slave_read (twi_t twi, void *buffer, twi_size_t size);


/** Perform a slave write with default timeout
    @param twi TWI controller to use
    @param size number of bytes to transfer
    @param timeout_us timeout in microseconds
    @return number of bytes read or negative value for an error
    @note If there are more bytes to be written than required, 
          they will be ignored
*/
twi_ret_t
twi_slave_write (twi_t twi, void *buffer, twi_size_t size);


void
twi_reset (twi_t twi);


void
twi_shutdown (twi_t twi);


#ifdef __cplusplus
}
#endif    
#endif



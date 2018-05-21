/** @file   twi.c
    @author M. P. Hayes
    @date   25 April 2015
    @brief  TWI routines for AT91 processors
*/

/* This driver only supports 7 bit slave addressing.  The user can
   check the internal address if 10 bit addressing is required.  

   General call is not supported.

   The master controls the clock although the slave can stretch it
   when it can't keep up.

   A master write consists of the slave address, followed by the
   internal address, and the data payload.  Each byte needs to be
   acknowledged by the slave.  

   A master read consists of a write followed by a read.  The write
   consists of the slave address followed by the internal address.  A
   repeated START is then sent followed by the slave address.  The
   master the reads the data payload from the slave.  The master
   acknowledges each byte and controls the number of bytes read by
   sending a STOP.

   Note, the TWI1 controller on a SAM4S shares TWCK1/TWD1 pins with
   JTAG.  By default, these are configured for JTAG.  They can be
   reconfigured by calling mcu_jtag_disable.

   If the MCU is reset in the middle of a transfer, some slave devices
   hold the SDA/TWD line low.  This condition can be resolved by
   sending out a number of SCL/TWCK pulses to advance the slave's
   state machine.
 
*/


#include "twi.h"
#include "pio.h"
#include "mcu.h"
#include "bits.h"
#include "delay.h"

#define TWI_DEVICES_NUM 2

static twi_dev_t twi_devices[TWI_DEVICES_NUM];


static void twi_unstick (twi_t twi)
{
    unsigned int i;
    
    switch (twi->channel)
    {
    case TWI_CHANNEL_0:
        pio_config_set (TWD0_PIO, PIO_PULLUP);
        pio_config_set (TWCK0_PIO, PIO_PULLUP);
        /* If the data line is stuck low, send some dummy clocks until
           it goes high.  */
        if (! pio_input_get (TWD0_PIO))
        {
            for (i = 0; i < 16; i++)
            {
                pio_config_set (TWCK0_PIO, PIO_OUTPUT_LOW);
                DELAY_US (5);
                pio_config_set (TWCK0_PIO, PIO_PULLUP);
                DELAY_US (5);
                if (pio_input_get (TWD0_PIO))
                    break;
            }
        }
        pio_config_set (TWD0_PIO, TWD0_PERIPH);
        pio_config_set (TWCK0_PIO, TWCK0_PERIPH);            
        break;

    case TWI_CHANNEL_1:
        pio_config_set (TWD1_PIO, PIO_PULLUP);
        pio_config_set (TWCK1_PIO, PIO_PULLUP);
        /* If the data line is stuck low, send some dummy clocks until
           it goes high.  */
        if (! pio_input_get (TWD1_PIO))
        {
            for (i = 0; i < 16; i++)
            {
                pio_config_set (TWCK1_PIO, PIO_OUTPUT_LOW);
                DELAY_US (5);
                pio_config_set (TWCK1_PIO, PIO_PULLUP);
                DELAY_US (5);
                if (pio_input_get (TWD1_PIO))
                    break;
            }
        }
        pio_config_set (TWD1_PIO, TWD1_PERIPH);
        pio_config_set (TWCK1_PIO, TWCK1_PERIPH);            
        break;
    }
}


void
twi_reset (twi_t twi)
{
    /* Dummy read of status register.  */
    twi->base->TWI_SR;

    /* Perform software reset of peripheral.  */
    twi->base->TWI_CR = TWI_CR_SWRST;

    /* The controller becomes a master on reset.  */
    twi->mode = TWI_MODE_MASTER;

    /* Clock ony required for master mode.  Set a 50% duty cycle.  */
    twi->base->TWI_CWGR = twi->clock_config;

    twi_unstick (twi);
}


static void twi_config (twi_t twi)
{
    switch (twi->channel)
    {
    case TWI_CHANNEL_0:
        pio_config_set (TWD0_PIO, TWD0_PERIPH);
        pio_config_set (TWCK0_PIO, TWCK0_PERIPH);
        break;

    case TWI_CHANNEL_1:
        pio_config_set (TWD1_PIO, TWD1_PERIPH);
        pio_config_set (TWCK1_PIO, TWCK1_PERIPH);
        break;
    }
}


twi_t 
twi_init (const twi_cfg_t *cfg)
{
    twi_t twi;
    
    twi = &twi_devices[cfg->channel];
    
    switch (cfg->channel)
    {
    case TWI_CHANNEL_0:
        twi->base = TWI0;
        break;

    case TWI_CHANNEL_1:
        twi->base = TWI1;
        break;

    default:
        return 0;
    }

    twi->channel = cfg->channel;
    twi_config (twi);
    
    /* Enable TWIx peripheral clock.  */
    mcu_pmc_enable (ID_TWI0 + cfg->channel);
    
    twi->clock_config = TWI_CWGR_CLDIV (cfg->period - 4) 
        | TWI_CWGR_CHDIV (cfg->period - 4)
        | TWI_CWGR_CKDIV ((cfg->period - 4) >> 8);

    /* Reset TWI peripheral.  */
    twi_reset (twi);

    /* Slave address ony required for slave mode.  */
    twi->slave_addr = cfg->slave_addr;

    return twi;
}


static twi_ret_t
twi_master_init (twi_t twi, twi_slave_addr_t slave_addr,
                 twi_iaddr_t addr, uint8_t addr_size, uint32_t read)
{
    /* Switch to master mode.  */
    twi->base->TWI_CR = TWI_CR_MSDIS;
    twi->base->TWI_CR = TWI_CR_SVDIS;
    twi->base->TWI_CR = TWI_CR_MSEN;

    twi->mode = TWI_MODE_MASTER;

    /* The flowchart Fig 33-17 suggests this order for MMR and IADR.  */
    twi->base->TWI_MMR = TWI_MMR_DADR (slave_addr)
        | BITS (addr_size, 8, 9) | read;
    
    twi->base->TWI_IADR = addr;
    
    return TWI_OK;
}


static twi_ret_t
twi_master_write_wait_ack (twi_t twi, twi_timeout_t timeout_us)
{
    uint32_t status;
    uint32_t retries = timeout_us;

    while (retries--)
    {
        status = twi->base->TWI_SR;

        /* TXCOMP set at same time as NACK.  */
        if (status & TWI_SR_NACK)
            return TWI_ERROR_NO_ACK;

        if (status & TWI_SR_ARBLST)
            return TWI_ERROR_CONFLICT;

        if (status & TWI_SR_TXRDY)
            return TWI_OK;

        DELAY_US (1);
    }
    return TWI_ERROR_TIMEOUT;
}


static twi_ret_t
twi_master_wait_txcomp (twi_t twi, twi_timeout_t timeout_us)
{
    uint32_t status;
    uint32_t retries = timeout_us;

    while (retries--)
    {
        status = twi->base->TWI_SR;

        if (status & TWI_SR_TXCOMP)
            return TWI_OK;

        DELAY_US (1);
    }
    return TWI_ERROR_TIMEOUT;
}


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
                               twi_timeout_t timeout_us)
{
    twi_size_t i;
    const uint8_t *data = buffer;
    twi_ret_t ret;

    /* If addr size is zero, need to set start and stop bits at same
       time.  */

    twi_master_init (twi, slave_addr, addr, addr_size, 0);

    /* Perhaps check that both the clock and data lines are high?  A
       common mistake is not to have pullup resistors for these
       lines.  */

    /* A START command is sent followed by the 7 bit slave address
       (MSB first) the read/write bit (0 for write, 1 for read), the
       acknowledge bit, then the optional internal address.  This is
       initiated by writing to THR.  Each of the sent bytes needs to
       be acknowledged by the slave.  There are two error scenarios 1)
       another master transmits at the same time with a higher
       priority 2) no slave responds to the desired address.
    */

    for (i = 0; i < size; i++)
    {
        twi->base->TWI_THR = *data++;

        ret = twi_master_write_wait_ack (twi, timeout_us);
        if (ret < 0)
        {
            /* FIXME.  The datasheet does not say what to do here!
               The slave has not responded in time.  Perhaps we need
               to reset the controller to prevent bus being left in a
               weird state.  */
            twi_reset (twi);
            return ret;
        }
    }

    /* Figure 33-16 says if there is a single data byte then write to
       THR, set TW_CR_STOP, check TXRDY, then check TXCOMP.  */

    twi->base->TWI_CR = TWI_CR_STOP;
    ret = twi_master_wait_txcomp (twi, timeout_us);
    if (ret < 0)
        return ret;

    return i;
}


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
twi_master_addr_write (twi_t twi, twi_slave_addr_t slave_addr,
                       twi_iaddr_t addr, uint8_t addr_size,
                       const void *buffer, twi_size_t size)
{
    return twi_master_addr_write_timeout (twi, slave_addr, addr, addr_size,
                                          buffer, size, TWI_TIMEOUT_US_DEFAULT);
}


/** Perform a master write to the specified slave address without internal address
    using default timeout
    @param twi TWI controller to use
    @param addr_size number of bytes for internal address (0--3)
    @param buffer buffer to write from
    @param size number of bytes to transfer
    @return number of bytes read or negative value for an error
*/    
twi_ret_t
twi_master_write (twi_t twi, twi_slave_addr_t slave_addr,
                  const void *buffer, twi_size_t size)
{
    return twi_master_addr_write (twi, slave_addr, 0, 0, buffer, size);
}


twi_ret_t
twi_master_read_wait_ack (twi_t twi, twi_timeout_t timeout_us)
{
    uint32_t status;
    uint32_t retries = timeout_us;

    while (retries--)
    {
        status = twi->base->TWI_SR;

        /* TXCOMP set at same time as NACK.  */
        if (status & TWI_SR_NACK)
            return TWI_ERROR_NO_ACK;

        if (status & TWI_SR_ARBLST)
            return TWI_ERROR_CONFLICT;

        if (status & TWI_SR_RXRDY)
            return TWI_OK;

        DELAY_US (1);
    }
    return TWI_ERROR_TIMEOUT;
}


/** Perform a master read to the specified slave address   
    @param twi TWI controller to use
    @param slave_addr 7 bit slave address
    @param addr optional internal address
    @param addr_size number of bytes for internal address (0--3)
    @param buffer buffer to read into
    @param size number of bytes to transfer
    @param timeout_us timeout in microseconds
    @return number of bytes read or negative value for an error
    @note If there are more bytes to be read than specified, they will be gobbled
*/    
twi_ret_t
twi_master_addr_read_timeout (twi_t twi, twi_slave_addr_t slave_addr,
                              twi_iaddr_t addr, uint8_t addr_size,
                              void *buffer, twi_size_t size,
                              twi_timeout_t timeout_us)
{
    twi_size_t i;
    uint8_t *data = buffer;
    twi_ret_t ret;

    if (size == 0)
        return 0;

    twi_master_init (twi, slave_addr, addr, addr_size, TWI_MMR_MREAD);

    if (size == 1)
        twi->base->TWI_CR = TWI_CR_START | TWI_CR_STOP;
    else
        twi->base->TWI_CR = TWI_CR_START;

    /* The slave address and optional internal address is sent. 
       Each sent byte should be acknowledged.  See Figure 33-20
       for a flowchart.  */

    for (i = 0; i < size; i++)
    {
        /* The master does not acknowledge receipt of the last byte.  */
        if ((i == size - 1) && (size > 1))
            twi->base->TWI_CR = TWI_CR_STOP;

        ret = twi_master_read_wait_ack (twi, timeout_us);
        if (ret < 0)
        {
            /* A STOP is automatically performed if we get a NACK.
               But what about a timeout, say while the clock is being
               stretched?  */
            twi_reset (twi);
            return ret;
        }

        *data++ = twi->base->TWI_RHR;
    }

    ret = twi_master_wait_txcomp (twi, timeout_us);
    if (ret < 0)
        return ret;

    /* Clear flags.  I'm not sure why!  */
    twi->base->TWI_SR;

    return i;
}


/** Perform a master read to the specified slave address   
    @param twi TWI controller to use
    @param slave_addr 7 bit slave address
    @param addr optional internal address
    @param addr_size number of bytes for internal address (0--3)
    @param buffer buffer to read into
    @param size number of bytes to transfer
    @return number of bytes read or negative value for an error
    @note If there are more bytes to be read than specified, they will be gobbled
*/    
twi_ret_t
twi_master_addr_read (twi_t twi, twi_slave_addr_t slave_addr,
                      twi_iaddr_t addr, uint8_t addr_size,
                      void *buffer, twi_size_t size)
{
    return twi_master_addr_read_timeout (twi, slave_addr, addr, addr_size,
                                         buffer, size, TWI_TIMEOUT_US_DEFAULT);
}


/** Perform a master read to the specified slave address   
    @param twi TWI controller to use
    @param slave_addr 7 bit slave address
    @param buffer buffer to read into
    @param size number of bytes to transfer
    @param timeout_us timeout in microseconds
    @return number of bytes read or negative value for an error
    @note If there are more bytes to be read than specified, 
          they will be gobbled
*/
twi_ret_t
twi_master_read (twi_t twi, twi_slave_addr_t slave_addr,
                 void *buffer, twi_size_t size)
{
    return twi_master_addr_read (twi, slave_addr, 0, 0, buffer, size);
}


static twi_ret_t
twi_slave_init (twi_t twi)
{
    /* Must be set before enabling slave mode.  */
    twi->base->TWI_SMR = TWI_MMR_DADR (twi->slave_addr);

    /* Switch to slave mode.  */
    twi->base->TWI_CR = TWI_CR_MSDIS;
    twi->base->TWI_CR = TWI_CR_SVDIS;
    twi->base->TWI_CR = TWI_CR_SVEN;

    twi->mode = TWI_MODE_SLAVE;

    return TWI_OK;
}


/** Poll TWI controller to detect a packet from the master
    @return TWI_WRITE if master write detected,
            TWI_READ if master read detected,
            otherwise TWI_OK
*/    
twi_ret_t
twi_slave_poll (twi_t twi)
{
    uint32_t status;

    /* Ensure in slave mode.  */
    if (twi->mode != TWI_MODE_SLAVE)
        twi_slave_init (twi);

    status = twi->base->TWI_SR;

    /* SVACC is set when our address matches until a STOP or repeated START.  */
    if (! (status & TWI_SR_SVACC))
        return TWI_IDLE;    

    /*  SVREAD is curiously named.  It is high when the master wants to read!  */
    if (status & TWI_SR_SVREAD)
        return TWI_READ;

    return TWI_WRITE;
}


static twi_ret_t
twi_slave_write_wait (twi_t twi, twi_timeout_t timeout_us)
{
    uint32_t status;
    uint32_t retries = timeout_us;

    while (retries--)
    {
        status = twi->base->TWI_SR;
        
        if (status & TWI_SR_SVACC)
        {
            /* Look for GACC 0, SVREAD 1, TXRDY 1.  */
            if (!(status & TWI_SR_GACC)
                && ((status & (TWI_SR_SVREAD | TWI_SR_TXRDY))
                    == (TWI_SR_SVREAD | TWI_SR_TXRDY)))
            {
                /* The master does not acknowledge the last byte of a
                   read and so NACK=1.  Even though TXRDY=1 we do not
                   want to write to THR so indicate that the transfer
                   is finished.  */
                if (status & TWI_SR_NACK)
                    return TWI_DONE;
                return TWI_OK;            
            }
        }
        else
        {
            /* Look for EOSACC 1, TXCOMP 1.  */
            if ((status & (TWI_SR_EOSACC | TWI_SR_TXCOMP))
                == (TWI_SR_EOSACC | TWI_SR_TXCOMP))
                return TWI_DONE;    
        }
        
        DELAY_US (1);
    }
    twi_reset (twi);
    return TWI_ERROR_TIMEOUT;
}


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
                         twi_timeout_t timeout_us)
{
    twi_size_t i;
    uint8_t *data = buffer;
    twi_ret_t ret;

    ret = twi_slave_poll (twi);
    if (ret <= 0)
        return ret;
    if (ret != TWI_READ)
        return TWI_ERROR_READ_EXPECTED;

    /* See Figure 33-31 for repeated START and reversal from write to
       read mode.  It appears that TXRDY goes high unexpectedly before
       TXCOMP goes low.  Thus we will load the THR prematurely.  This
       is not sent until the next master read.  To avoid this, we need
       to check NACK since the master does not acknowledge the last
       byte and this will set NACK.  */

    for (i = 0; i < size; i++)
    {
        ret = twi_slave_write_wait (twi, timeout_us);
        if (ret < 0)
            return ret;
        if (ret == TWI_DONE)
            return i;
            
        twi->base->TWI_THR = *data++;
    }

    /* Until the master sends a STOP, send dummy data.  */
    for (i = 0; i < 32767; i++)
    {
        ret = twi_slave_write_wait (twi, timeout_us);
        if (ret < 0)
            return ret;
        if (ret == TWI_DONE)
            break;
            
        /* Send recognisable sequence for debugging.  */
        twi->base->TWI_THR = 'A' + (i & 0xff);
    }
    if (i == 32768)
        return TWI_ERROR_NO_STOP;

    /* Return number of bytes written.  */
    return size + i;
}


/** Perform a slave write with default timeout
    @param twi TWI controller to use
    @param size number of bytes to transfer
    @return number of bytes read or negative value for an error
    @note If there are more bytes to be written than required, 
          they will be ignored
*/
twi_ret_t
twi_slave_write (twi_t twi, void *buffer, twi_size_t size)
{
    return twi_slave_write_timeout (twi, buffer, size, TWI_TIMEOUT_US_DEFAULT);
}


static twi_ret_t
twi_slave_read_wait (twi_t twi, twi_timeout_t timeout_us)
{
    uint32_t status;
    uint32_t retries = timeout_us;

    while (retries--)
    {
        status = twi->base->TWI_SR;
        
        if (status & TWI_SR_SVACC)
        {
            /* Look for GACC 0, SVREAD 0, RXRDY 1.  */
            if (!(status & (TWI_SR_GACC | TWI_SR_SVREAD))
                && (status & TWI_SR_RXRDY))
                return TWI_OK;            
        }
        else
        {
            /* Look for EOSACC 1 or TXCOMP 1. 

               When have a reversal from write to read mode
               EOSACC=1, TXCOMP=0.

               When receive STOP EOSACC=1, TXCOMP=1.  */
            if (status & (TWI_SR_EOSACC | TWI_SR_TXCOMP))
                return TWI_DONE;    
        }
        DELAY_US (1);
    }
    twi_reset (twi);
    return TWI_ERROR_TIMEOUT;
} 


/** Perform a slave read with specified timeout
    @param twi TWI controller to use
    @param size number of bytes to transfer
    @timeout_us timeout in microseconds
    @return number of bytes read or negative value for an error
    @note If there are more bytes to be read than specified, they will be gobbled
*/
twi_ret_t
twi_slave_read_timeout (twi_t twi, void *buffer, twi_size_t size,
                        twi_timeout_t timeout_us)
{
    twi_size_t i;
    uint8_t *data = buffer;
    twi_ret_t ret;

    ret = twi_slave_poll (twi);
    if (ret <= 0)
        return ret;
    if (ret != TWI_WRITE)
        return TWI_ERROR_WRITE_EXPECTED;

    /* SVACC=1, SVREAD=0.  */

    for (i = 0; i < size; i++)
    {
        ret = twi_slave_read_wait (twi, timeout_us);
        if (ret < 0)
            return ret;
        if (ret == TWI_DONE)
            return i;
            
        *data++ = twi->base->TWI_RHR;
    }

    /* Read any other pending data.  */
    while (1)
    {
        ret = twi_slave_read_wait (twi, timeout_us);
        if (ret < 0)
            return ret;
        if (ret == TWI_DONE)
            break;

        /* Discard data.  */
        twi->base->TWI_RHR;
    }
    return size;
}


/** Perform a slave read with default timeout
    @param twi TWI controller to use
    @param size number of bytes to transfer
    @return number of bytes read or negative value for an error
    @note If there are more bytes to be read than specified, they will be gobbled
*/
twi_ret_t
twi_slave_read (twi_t twi, void *buffer, twi_size_t size)
{
    return twi_slave_read_timeout (twi, buffer, size, TWI_TIMEOUT_US_DEFAULT);
}


void
twi_shutdown (twi_t twi)
{
    if (twi->base == TWI1)
    {
        pio_config_set (TWD1_PIO, PIO_PULLUP);
        pio_config_set (TWCK1_PIO, PIO_PULLUP);
        mcu_pmc_disable (ID_TWI1);
    }
    else
    {
        pio_config_set (TWD0_PIO, PIO_PULLUP);
        pio_config_set (TWCK0_PIO, PIO_PULLUP);
        mcu_pmc_disable (ID_TWI0);
    }
}

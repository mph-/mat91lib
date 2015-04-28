/** @file   twi.c
    @author M. P. Hayes
    @date   25 April 2015
    @brief  TWI routines for AT91 processors
*/

/* This driver only supports 7 bit slave addressing.  The user can
   check the internal address if 10 bit addressing is required.  

   General call is not supported.
*/


#include "twi.h"
#include "pio.h"
#include "mcu.h"
#include "bits.h"
#include "delay.h"


#define TWI_DEVICES_NUM 2

static twi_dev_t twi_devices[TWI_DEVICES_NUM];


twi_t 
twi_init (const twi_cfg_t *cfg)
{
    twi_t twi;
    
    twi = &twi_devices[cfg->channel];
    
    switch (cfg->channel)
    {
    case TWI_CHANNEL_0:
        twi->base = TWI0;
        pio_config_set (TWD0_PIO, TWD0_PERIPH);
        pio_config_set (TWCK0_PIO, TWCK0_PERIPH);
        break;

    case TWI_CHANNEL_1:
        twi->base = TWI1;
        pio_config_set (TWD1_PIO, TWD0_PERIPH);
        pio_config_set (TWCK1_PIO, TWCK0_PERIPH);
        break;

    default:
        return 0;
    }

    /* Enable TWIx peripheral clock.  */
    mcu_pmc_enable (ID_TWI0 + cfg->channel);
    
    twi = &twi_devices[cfg->channel];

    /* Clock ony required for master mode.  */
    twi->base->TWI_CWGR = cfg->period;

    /* Slave addres ony required for slave mode.  */
    twi->slave_addr = cfg->slave_addr;

    return twi;
}


static twi_ret_t
twi_master_init (twi_t twi, twi_id_t slave_addr,
                 twi_id_t addr, uint8_t addr_size, uint32_t read)
{
    twi->base->TWI_MMR = TWI_MMR_DADR (slave_addr)
        | BITS (addr_size, 8, 9) | read;
    
    twi->base->TWI_IADR = addr;
    
    /* Switch to master mode.  */
    twi->base->TWI_CR = TWI_CR_SVDIS | TWI_CR_MSEN;
    twi->mode = TWI_MODE_MASTER;

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


twi_ret_t
twi_master_addr_write_timeout (twi_t twi, twi_id_t slave_addr,
                               twi_id_t addr, uint8_t addr_size,
                               void *buffer, uint8_t size, twi_timeout_t timeout_us)
{
    uint8_t i;
    uint8_t *data = buffer;
    twi_ret_t ret;

    twi_master_init (twi, slave_addr, addr, addr_size, 0);

    twi->base->TWI_CR = TWI_CR_START;

    /* A START command is sent followed by the slave address and the
       optional internal address.  This initiated by writing to THR.
       Each of the sent bytes needs to be acknowledged by the slave.
       There are two error scenarios 1) another master transmits at
       the same time with a higher priority 2) no slave responds to
       the desired address
    */

    for (i = 0; i < size; i++)
    {
    
        twi->base->TWI_THR = *data++;
        if (i == size - 1)
            twi->base->TWI_CR = TWI_CR_STOP;
            
        ret = twi_master_write_wait_ack (twi, timeout_us);
        if (ret < 0)
        {
            twi->base->TWI_CR = TWI_CR_STOP;
            return ret;
        }
    }

    return i;
}


twi_ret_t
twi_master_addr_write (twi_t twi, twi_id_t slave_addr,
                       twi_id_t addr, uint8_t addr_size,
                       void *buffer, uint8_t size)
{
    return twi_master_addr_read_timeout (twi, slave_addr, addr, addr_size,
                                         buffer, size, TWI_TIMEOUT_US_DEFAULT);
}


twi_ret_t
twi_master_write (twi_t twi, twi_id_t slave_addr,
                  void *buffer, uint8_t size)
{
    return twi_master_addr_write (twi, slave_addr, 0, 0, buffer, size);
}


static twi_ret_t
twi_master_read_wait_ack (twi_t twi, twi_timeout_t timeout_us)
{
    uint32_t status;
    uint32_t retries = timeout_us;

    while (retries--)
    {
        status = twi->base->TWI_SR;

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


twi_ret_t
twi_master_addr_read_timeout (twi_t twi, twi_id_t slave_addr,
                              twi_id_t addr, uint8_t addr_size,
                              void *buffer, uint8_t size, twi_timeout_t timeout_us)
{
    uint8_t i;
    uint8_t *data = buffer;
    twi_ret_t ret;

    twi_master_init (twi, slave_addr, addr, addr_size, TWI_MMR_MREAD);

    if (size == 1)
        twi->base->TWI_CR = TWI_CR_START;
    else
        twi->base->TWI_CR = TWI_CR_START | TWI_CR_STOP;

    /* The slave address and optional internal address is sent. 
       Each sent byte should be acknowledged.  */

    for (i = 0; i < size; i++)
    {
        ret = twi_master_read_wait_ack (twi, timeout_us);
        if (ret < 0)
        {
            twi->base->TWI_CR = TWI_CR_STOP;
            return ret;
        }

        *data++ = twi->base->TWI_RHR;

        /* Need to set STOP prior to reading last byte.  */
        if ((i != 0) && (i == size - 1))
            twi->base->TWI_CR = TWI_CR_STOP;
    }

    return i;
}



twi_ret_t
twi_master_addr_read (twi_t twi, twi_id_t slave_addr,
                      twi_id_t addr, uint8_t addr_size,
                      void *buffer, uint8_t size)
{
    return twi_master_addr_read_timeout (twi, slave_addr, addr, addr_size,
                                         buffer, size, TWI_TIMEOUT_US_DEFAULT);
}



twi_ret_t
twi_master_read (twi_t twi, twi_id_t slave_addr,
                 void *buffer, uint8_t size)
{
    return twi_master_addr_read (twi, slave_addr, 0, 0, buffer, size);
}


static twi_ret_t
twi_slave_init (twi_t twi)
{
    twi->base->TWI_SMR = TWI_MMR_DADR (twi->slave_addr);

    /* Switch to slave mode.  */
    twi->base->TWI_CR = TWI_CR_MSDIS | TWI_CR_SVEN;
    twi->mode = TWI_MODE_SLAVE;

    return TWI_OK;
}


twi_ret_t
twi_slave_poll (twi_t twi)
{
    uint32_t status;

    if (twi->mode != TWI_MODE_SLAVE)
        twi_slave_init (twi);

    status = twi->base->TWI_SR;

    if (! (status & TWI_SR_SVACC))
        return TWI_OK;    

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
        
        if (! (status & TWI_SR_SVACC))
            return TWI_DONE;    
        
        if (status & TWI_SR_TXRDY)
            return TWI_OK;
        
        DELAY_US (1);
    }
    return TWI_ERROR_TIMEOUT;
}


twi_ret_t
twi_slave_write_timeout (twi_t twi, void *buffer, uint8_t size, 
                         twi_timeout_t timeout_us)
{
    uint8_t i;
    uint8_t *data = buffer;
    twi_ret_t ret;

    ret = twi_slave_poll (twi);
    if (ret <= 0)
        return ret;
    if (ret != TWI_WRITE)
        return TWI_ERROR_WRITE_EXPECTED;

    for (i = 0; i < size; i++)
    {
        ret = twi_slave_write_wait (twi, timeout_us);
        if (ret < 0)
            return ret;
        if (ret == TWI_DONE)
            return i;
            
        twi->base->TWI_THR = *data++;
    }

    /* What if the master wants more data?  We could keep sending
       zero bytes.  Let's let the user handle this.  */

    return i;
}


twi_ret_t
twi_slave_write (twi_t twi, void *buffer, uint8_t size)
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
        
        if (! (status & TWI_SR_SVACC))
            return TWI_DONE;    
        
        if (status & TWI_SR_RXRDY)
            return TWI_OK;
        
        DELAY_US (1);
    }
    return TWI_ERROR_TIMEOUT;
} 


twi_ret_t
twi_slave_read_timeout (twi_t twi, void *buffer, uint8_t size,
                        twi_timeout_t timeout_us)
{
    uint8_t i;
    uint8_t *data = buffer;
    twi_ret_t ret;

    ret = twi_slave_poll (twi);
    if (ret <= 0)
        return ret;
    if (ret != TWI_READ)
        return TWI_ERROR_READ_EXPECTED;

    for (i = 0; i < size; i++)
    {
        ret = twi_slave_read_wait (twi, timeout_us);
        if (ret < 0)
            return ret;
        if (ret == TWI_DONE)
            return i;
            
        *data++ = twi->base->TWI_THR;
    }

    /* What if there is unread data still pending?  */
    return i;
}


twi_ret_t
twi_slave_read (twi_t twi, void *buffer, uint8_t size)
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

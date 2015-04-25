/** @file   twi.c
    @author M. P. Hayes
    @date   25 April 2015
    @brief  TWI routines for AT91 processors
*/


#include "twi.h"
#include "pio.h"
#include "mcu.h"
#include "bits.h"
#include "pinmap.h"


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
        break;

    case TWI_CHANNEL_1:
        twi->base = TWI1;
        break;

    default:
        return 0;
    }

    /* Enable TWIx peripheral clock.  */
    mcu_pmc_enable (ID_TWI0 + cfg->channel);
    
    return twi;
}



twi_ret_t
twi_master_write (twi_t dev, twi_id_t slave_addr,
                  twi_id_t addr, uint8_t addr_size,
                  void *buffer, uint8_t size)
{
    return 0;
}


twi_ret_t
twi_master_read (twi_t dev, twi_id_t slave_addr,
                 twi_id_t addr, uint8_t addr_size,
                 void *buffer, uint8_t size)
{
    return 0;
}

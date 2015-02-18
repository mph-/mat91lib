/** @file   ac.c
    @author 
    @date   12 February 2008
    @brief  Analogue to digital converter routines for AT91SAM7 processors
*/

#include "ac.h"
#include "bits.h"
#include "mcu.h"


/* There is no limit.  */
#ifndef AC_DEVICES_NUM
#define AC_DEVICES_NUM 8
#endif


static uint8_t ac_devices_num = 0;
static ac_dev_t ac_devices[AC_DEVICES_NUM];
static ac_dev_t *ac_config_last = 0;


/* Resets AC.  */
static void
ac_reset (void)
{
    ACC->ACC_CR = ACC_CR_SWRST;
}



bool
ac_channel_set (ac_t ac, ac_channel_t channel)
{
    BITS_INSERT (ac->MR, channel, 4, 6);
    return 1;
}


bool
ac_reference_set (ac_t ac, ac_reference_t reference)
{
    BITS_INSERT (ac->MR, reference, 0, 2);
    return 1;
}


bool
ac_edge_set (ac_t ac, ac_edge_t edge)
{
    BITS_INSERT (ac->MR, edge, 9, 10);
    return 1;
}


bool
ac_hysteresis_set (ac_t ac, ac_hysteresis_t hysteresis)
{
    BITS_INSERT (ac->ACR, hysteresis, 1, 2);
    return 1;
}


bool
ac_current_set (ac_t ac, ac_current_t current)
{
    BITS_INSERT (ac->ACR, current, 0, 0);
    return 1;
}


bool
ac_config (ac_t ac)
{
    if (ac == ac_config_last)
        return 1;
    ac_config_last = ac;

    ACC->ACC_MR = ac->MR;
    ACC->ACC_ACR = ac->ACR;

    return 1;
}


static void
ac_config_set (ac_t ac, const ac_cfg_t *cfg)
{
    ac_channel_set (ac, cfg->channel);
    ac_reference_set (ac, cfg->reference);
    ac_edge_set (ac, cfg->edge);
    ac_hysteresis_set (ac, cfg->hysteresis);
    ac_current_set (ac, cfg->current);
}


void
ac_enable (ac_t ac)
{
    BITS_INSERT (ac->MR, 1, 8, 8);    
    ac_config (ac);
}


void
ac_disable (ac_t ac)
{
    BITS_INSERT (ac->MR, 0, 8, 8);
    ac_config (ac);
}


void
ac_irq_enable (ac_t ac)
{
    ACC->ACC_IER = 1;
}


void
ac_irq_disable (ac_t ac)
{
    ACC->ACC_IDR = 1;
}


bool
ac_poll (ac_t ac)
{
    return (ACC->ACC_ISR & 1) != 0;
}



/** Initalises the AC registers for polling operation.  */
ac_t
ac_init (const ac_cfg_t *cfg)
{
    ac_dev_t *ac;
    
    if (ac_devices_num >= AC_DEVICES_NUM)
        return 0;

    if (ac_devices_num == 0)
    {
        /* The clock only needs to be enabled when sampling.  The clock is
           automatically started for the SAM7.  */
        mcu_pmc_enable (ID_ACC);
        
        ac_reset ();
    }

    ac = ac_devices + ac_devices_num;
    ac_devices_num++;

    ac->MR = 0;
    ac->ACR = 0;
    ac_config_set (ac, cfg);
    ac_config (ac);
    
    return ac;
}


void
ac_shutdown (ac_t ac)
{
    /* TODO.  */
    mcu_pmc_disable (ID_ACC);
}

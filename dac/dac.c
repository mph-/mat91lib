/** @file   dac.c
    @author M. P. Hayes
    @date   28 August 2016
    @brief  Analogue to digital converter routines for AT91SAM processors
*/

#include "dac.h"
#include "bits.h"
#include "mcu.h"

/* On reset the the PIO pins are configured as inputs with pullups.
   To make a PIO pin an DAC pin requires programming DAC_CHER.

   The SAM4S requires 25 clocks for a conversion.

   Note, the analog output voltage droops after 20 microseconds.
   There is an automatic refresh mode.
*/


/* There is no limit.  This is arbitrary.  Note each device can
   support multiple channels.  */
#ifndef DAC_DEVICES_NUM
#define DAC_DEVICES_NUM 2
#endif


#define DAC_STARTUP_TIME_MIN 12e-6


/* Startup time from standby mode to normal mode.  */
#ifndef DAC_STARTUP_TIME
#define DAC_STARTUP_TIME DAC_STARTUP_TIME_MIN
#endif

#define DAC DACC


static uint8_t dac_devices_num = 0;
static dac_dev_t dac_devices[DAC_DEVICES_NUM];
static dac_dev_t *dac_config_last = 0;


/** Reset DAC.  */
static void
dac_reset (void)
{
    DAC->DACC_CR = DACC_CR_SWRST;
}


/** Put DAC into sleep mode.
    It should wake on next trigger.  */
void
dac_sleep (dac_t dac)
{
    DAC->DACC_MR |= DACC_MR_SLEEP;
}


/** Set the DAC triggering.  */
void
dac_trigger_set (dac_t dac, dac_trigger_t trigger) 
{
    dac->trigger = trigger;

    /* Could also handle FREERUN here where no triggering is
       required.  */

    if (trigger == DAC_TRIGGER_SW)
    {
        /* Disable trigger.  */
        dac->MR &= ~DACC_MR_TRGEN_EN;
    }
    else
    {
        /* Select trigger.  */
        BITS_INSERT (dac->MR, trigger - DAC_TRIGGER_EXT, 1, 3);

        /* Enable trigger.  */
        dac->MR |= DACC_MR_TRGEN_EN;
    }
}


/** Set the clock divider (prescaler).  */
static void
dac_clock_divisor_set (dac_t dac, dac_clock_divisor_t clock_divisor) 
{
    /* The SAM4S requires 20 clocks per sample. 

       DAC_CLOCK = (F_CPU / 2) / clock_divisor.  
    */

    if (clock_divisor >= 256)
        clock_divisor = 256;

    BITS_INSERT (dac->MR, clock_divisor - 1, 8, 15);
    dac->clock_divisor = clock_divisor;
}


/** Set clock speed.  */
dac_clock_speed_t
dac_clock_speed_kHz_set (dac_t dac, dac_clock_speed_t clock_speed_kHz)
{
    uint32_t clock_speed;

    clock_speed = clock_speed_kHz * 1000;
    dac_clock_divisor_set (dac, ((F_CPU / 2) + clock_speed - 1) / clock_speed);
    clock_speed = (F_CPU / 2) / dac->clock_divisor;

    /* STARTUP: With 24 MHz clock need 288 clocks to start up on
       SAM4S.  Let's allocate 512.  TODO, scan through table to find
       appropriate value.  */
    BITS_INSERT (dac->MR, 8, 16, 19);

    return clock_speed / 1000;
}


/** Set number of clocks between refreshes or 0 to disable.  */
static uint16_t
dac_refresh_clocks_set (dac_t dac, uint16_t refresh_clocks)
{
    uint16_t refresh;

    /* Need to use multiple of 1024 clocks so round to
       nearest multiple.   */
    refresh = (refresh_clocks + (1 << 9)) >> 10;
    BITS_INSERT (dac->MR, refresh, 8, 15);

    return refresh << 10;
}


/** Set the channels to convert.
    This is not actioned until dac_channels_select called.  */
bool
dac_channels_set (dac_t dac, dac_channels_t channels)
{
    dac->channels = channels;
    return 1;
}


/** Select the channels to convert.  */
static void
dac_channels_select (dac_t dac)
{
    DAC->DACC_CHDR = ~0;
    DAC->DACC_CHER = dac->channels;
}


/** Set number of bits to convert.  */
uint8_t
dac_bits_set (dac_t dac, uint8_t bits)
{
    if (bits != 12)
        return 0;

    dac->bits = bits;
    return bits;
}


bool
dac_config (dac_t dac)
{
    dac_channels_select (dac);

    if (dac == dac_config_last)
        return 1;
    dac_config_last = dac;

    /* Set mode register.  */
    DAC->DACC_MR = dac->MR;
    return 1;
}


static void
dac_config_set (dac_t dac, const dac_cfg_t *cfg)
{
    dac_bits_set (dac, cfg->bits);
    dac_clock_speed_kHz_set (dac, cfg->clock_speed_kHz);
    dac_trigger_set (dac, cfg->trigger);
    if (cfg->channels == 0)
    {
        dac_channels_set (dac, BIT (cfg->channel));
        /* Select channel.  */
        BITS_INSERT(dac->MR, BIT (cfg->channel), 16, 17);
        /* Clear tag bit.  */
        BITS_INSERT(dac->MR, 0, 20, 20);
    }
    else
    {
        dac_channels_set (dac, cfg->channels);
        /* Set tag bit.  In this mode, bits 12 and 13 of the data
           specify the channel.  */
        BITS_INSERT(dac->MR, 1, 20, 20);
    }

    dac_refresh_clocks_set (dac, cfg->refresh_clocks);
}


Pdc *
dac_pdc_get (dac_t dac)
{
    return PDC_DACC;
}


void
dac_enable (dac_t dac)
{
    /* Dummy function for symmetry with ssc driver.  */
}


void
dac_disable (dac_t dac)
{
    /* Dummy function for symmetry with ssc driver.  */
}


/** Initalises the DAC registers for polling operation.  */
dac_t
dac_init (const dac_cfg_t *cfg)
{
    dac_sample_t dummy;
    dac_dev_t *dac;
    const dac_cfg_t dac_default_cfg =
        {
            .bits = 10,
            .channel = 0,
            .clock_speed_kHz = 1000
        };
    
    if (dac_devices_num >= DAC_DEVICES_NUM)
        return 0;

    if (dac_devices_num == 0)
    {
        /* The clock only needs to be enabled when sampling.  The clock is
           automatically started for the SAM7.  */
        mcu_pmc_enable (ID_DACC);
        
        dac_reset ();
    }

    dac = dac_devices + dac_devices_num;
    dac_devices_num++;
    
    dac->MR = 0;

    if (!cfg)
        cfg = &dac_default_cfg;

    dac_config_set (dac, cfg);

    /* Note, the DAC is not configured until dac_config is called.  */
    dac_config (dac);

    return dac;
}


/** Returns true if a conversion has finished.  */
bool
dac_ready_p (dac_t dac)
{
    return (DAC->DACC_ISR & DACC_ISR_TXRDY) != 0;
}


/** Blocking write.  This will hang if a trigger is not supplied
    (except for software triggering mode).  */
int8_t
dac_write (dac_t dac, void *buffer, uint16_t size)
{
    uint16_t i;
    uint16_t samples;
    dac_sample_t *data;

    dac_config (dac);

    samples = size / sizeof (dac_sample_t);
    data = buffer;

    for (i = 0; i < samples; i++)
    {
        /* Should have timeout, especially for external trigger.  */
        while (!dac_ready_p (dac))
            continue;

        DAC->DACC_CDR = data[i];
    }

    return samples * sizeof (dac_sample_t);
}


void
dac_shutdown (dac_t dac)
{
    /* TODO.  */
    mcu_pmc_disable (ID_DACC);
}

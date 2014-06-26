/** @file    ssc.c
    @author  Stuart Duncan, Michael Hayes
    @date    10 January 2012
    @brief   Routines for control and configuration of the AT91SAM7's SSC
   
   */

#include "ssc.h"
#include "mcu.h"
#include "bits.h"



/* Init and configure, still need to be enabled after calling this.  */
void
ssc_init (const ssc_cfg_t *cfg)
{
    ssc_config (cfg);
    ssc_enable ();
}


/* set the clock divider.  */
void
ssc_set_clock_div (ssc_clock_div_t clockdiv) 
{
   SSC->SSC_CMR = clockdiv;
}


/* configure a module.  */
uint8_t
ssc_module_config (ssc_module_cfg_t *cfg) 
{
    /* Check if the config exists (by way of null pointer check. FIXME).  */
    if (cfg)
    {
        /* Select options to apply to clock mode register.  */
        uint32_t cmr;
        uint32_t fmr;


        cmr = (cfg->period << 24) | (cfg->delay << 16) | cfg->start_mode
            | cfg->clock_select | cfg->clock_out_mode
            | cfg->clock_gate_mode | cfg->clock_sampling_edge;
        
        /* Select options to apply to frame mode register FIXME magic numbers*/
        fmr = cfg->fsedge_mode | cfg->fsos_mode | (cfg->fslen << 16)
            | ((cfg->words_per_frame-1)<<8) | (cfg->word_size-1) | cfg->msb; 
        
        /* Apply the configuration to the appropriate module.  */
        if (cfg->tx_or_rx == SSC_TX)
        {
            SSC->SSC_TFMR = fmr | cfg->td_default | cfg->sync_data_enable;
            SSC->SSC_TCMR = cmr;
            return 0;
        }
        else if ((cfg->tx_or_rx) == SSC_RX)
        {
            SSC->SSC_RFMR = fmr | cfg->loop;
            SSC->SSC_RCMR = cmr | cfg->stop_mode;
            return 0;
        }
        else
            return 1;
    }
    else
        return 2;
}


/* Configure the ssc peripheral, null pointer = don't use a module.  */
void ssc_config (const ssc_cfg_t *cfg) 
{
    /* Enable the peripheral clock.  */
    mcu_pmc_enable (ID_SSC);
    
    /* Set the clock divider.  */
    ssc_set_clock_div (cfg->clock_div);
    
    /* Configure the reciever module if the configuration exists.  */
    ssc_module_config (cfg->rx_cfg);
    
    /* Configure the transmit module if the configuration exists.  */
    ssc_module_config (cfg->tx_cfg);
    
}


/* read data from the rx buffer*/
ssc_data_t
ssc_read_data (bool wait) 
{
    if (wait)
    {
        while (!ssc_buffer_ready (SSC_RX))
            continue;
    }
    
    return SSC->SSC_RHR;
}



/* check if a buffer (tx or rx) is ready (empty or full respectively).  */
bool
ssc_buffer_ready (ssc_module_t tx_rx)
{
    unsigned int mask = 0;

    switch (tx_rx)
    {
    case SSC_TX:
        mask = SSC_SR_TXRDY;
        break;

    case SSC_RX:
        mask = SSC_SR_RXRDY;
        break;
    }
    
    if ((mask & SSC->SSC_SR) != 0)
        return true;
    else
        return false;
}


/* enable a SSC module*/
void
ssc_module_enable (ssc_module_t tx_rx) 
{
    switch (tx_rx) 
    {
    case SSC_TX: 
        SSC->SSC_CR |= SSC_CR_TXEN;

        pio_config_set (RD_PIO, PIO_PERIPH_A);
        pio_config_set (RK_PIO, PIO_PERIPH_A);
        pio_config_set (RF_PIO, PIO_PERIPH_A);
        break;

    case SSC_RX:
        SSC->SSC_CR |= SSC_CR_RXEN;

        pio_config_set (TD_PIO, PIO_PERIPH_A);
        pio_config_set (TK_PIO, PIO_PERIPH_A);
        pio_config_set (TF_PIO, PIO_PERIPH_A);
        break;
    }
}


/* Disable a SSC module.  */
void
ssc_module_disable (ssc_module_t tx_rx) 
{
    switch (tx_rx)
    {
    case SSC_TX: 
        SSC->SSC_CR |= SSC_CR_TXDIS;

        pio_config_set (TD_PIO, PIO_INPUT);
        pio_config_set (TK_PIO, PIO_INPUT);
        pio_config_set (TF_PIO, PIO_INPUT);
        break;

    case SSC_RX:
        SSC->SSC_CR |= SSC_CR_RXDIS;

        pio_config_set (RD_PIO, PIO_INPUT);
        pio_config_set (RK_PIO, PIO_INPUT);
        pio_config_set (RF_PIO, PIO_INPUT);
        break;
    }
}


/* Disable all of the modules.  */
void
ssc_disable (void)
{
    ssc_module_disable (SSC_TX);
    ssc_module_disable (SSC_RX);
}


/* Enable all of the modules.  */
void
ssc_enable (void)
{
    ssc_module_enable (SSC_TX);
    ssc_module_enable (SSC_RX);
}



/* Read from the receive holding register.  */
ssc_data_t
ssc_read (bool wait)
{
   ssc_data_t temp_data = 0;
   
   if (!wait)
       temp_data = SSC->SSC_RHR;
   
   return temp_data;
}


/* Write to the transmit buffer.  */
void
ssc_write (ssc_data_t data, bool wait)
{
    SSC->SSC_THR = data;
}


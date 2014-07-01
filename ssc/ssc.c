/** @file    ssc.c
    @author  Stuart Duncan, Michael Hayes
    @date    10 January 2012
    @brief   Routines for control and configuration of the AT91SAM7's SSC
   
   */

#include "ssc.h"
#include "mcu.h"
#include "bits.h"


ssc_t ssc_dev;


/* Set the clock divider.  */
static void
ssc_clock_divisor_set (ssc_t *ssc, ssc_clock_divisor_t clock_divisor) 
{
   SSC->SSC_CMR = clock_divisor;
   ssc->clock_divisor = clock_divisor;
}


ssc_clock_speed_t
ssc_clock_speed_kHz_set (ssc_t *ssc, ssc_clock_speed_t clock_speed_kHz)
{
    uint32_t clock_speed;

    clock_speed = clock_speed_kHz * 1000;
    ssc_clock_divisor_set (ssc, (F_CPU + clock_speed - 1) / clock_speed);
    clock_speed = F_CPU / ssc->clock_divisor;

    return clock_speed / 1000;
}


/* Configure a module.  */
static uint8_t
ssc_module_config (ssc_module_cfg_t *cfg, ssc_module_t module) 
{
    /* Select options to apply to clock mode register.  */
    uint32_t cmr;
    uint32_t fmr;
    
    if (!cfg)
        return 0;
        
    cmr = (cfg->period << 24) | (cfg->delay << 16) | cfg->start_mode
        | cfg->clock_sampling_edge;
    
    switch (cfg->clock_select)
    {
    case SSC_CLOCK_INTERNAL:
        // Use internally generated clock
        break;
        
    case SSC_CLOCK_OTHER:
        // Use clock from other module
        cmr |= SSC_RCMR_CKS_TK;
        break;
        
    case SSC_CLOCK_PIN:
        // Use clock from clock pin
        cmr |= SSC_RCMR_CKS_RK;
        break;
        }
    
    switch (cfg->clock_out_mode)
    {
    case SSC_CLOCK_INPUT:
        // External clock - no clock control
        cmr |= SSC_RCMR_CKO_NONE;
        break;
        
    case SSC_CLOCK_CONTINUOUS:
        // Continuous clock output
        cmr |= SSC_RCMR_CKO_CONTINUOUS;
        break;
        
    case SSC_CLOCK_TRANSFER:
        // Clock output only for data transfers
        cmr |= SSC_RCMR_CKO_TRANSFER;
        break;
    }
    
    switch (cfg->clock_gate_mode)
    {
    case SSC_CLOCK_GATE_NONE:
        break;
        
    case SSC_CLOCK_GATE_RF_LOW:
        cmr |= 1 << 7;
        break;
        
    case SSC_CLOCK_GATE_RF_HIGH:
        cmr |= 2 << 7;
        break;
    }
    
    fmr = cfg->fsos_mode | (cfg->fslen << 16)
        | ((cfg->words_per_frame-1) << 8) | (cfg->word_size - 1); 
    

    switch (cfg->fsedge)
    {
    case SSC_FSEDGE_POSITIVE:
        break;
        
    case SSC_FSEDGE_NEGATIVE:
        fmr |= SSC_RFMR_FSEDGE;
        break;
    }
    
    if (cfg->msb_first)
        fmr |= SSC_RFMR_MSBF;
    
    /* Apply the configuration to the appropriate module.  */
    if (module == SSC_TX)
    {
        if (cfg->td_default)
            fmr |= SSC_TFMR_DATDEF;
        
        SSC->SSC_TFMR = fmr | cfg->sync_data_enable;
        SSC->SSC_TCMR = cmr;
    }
    else 
    {
        if (cfg->loop_mode)
            fmr |= SSC_RFMR_LOOP;
        
        SSC->SSC_RFMR = fmr;
        SSC->SSC_RCMR = cmr | cfg->stop_mode;
    }
    return 1;
}


/* Configure the ssc peripheral, null pointer = don't use a module.  */
static void
ssc_config (ssc_t *ssc, const ssc_cfg_t *cfg) 
{
    /* Enable the peripheral clock.  */
    mcu_pmc_enable (ID_SSC);
    
    /* Set the clock divider.  */
    ssc_clock_speed_kHz_set (ssc, cfg->clock_speed_kHz);
    
    /* Configure the receiver module if the configuration exists.  */
    ssc_module_config (cfg->rx, SSC_RX);
    
    /* Configure the transmit module if the configuration exists.  */
    ssc_module_config (cfg->tx, SSC_TX);
}


/* Check if a buffer (tx or rx) is ready (empty or full respectively).  */
static bool
ssc_module_ready_p (ssc_t *ssc, ssc_module_t tx_rx)
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
    
    return ((mask & SSC->SSC_SR) != 0);
}


bool
ssc_read_ready_p (ssc_t *ssc)
{
    return ssc_module_ready_p (ssc, SSC_RX);
}


bool
ssc_write_ready_p (ssc_t *ssc)
{
    return ssc_module_ready_p (ssc, SSC_TX);
}


/* Enable an SSC module.  */
static void
ssc_module_enable (ssc_t *ssc, ssc_module_t tx_rx) 
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


/* Disable an SSC module.  */
static void
ssc_module_disable (ssc_t *ssc, ssc_module_t tx_rx) 
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
ssc_disable (ssc_t *ssc)
{
    if (ssc->tx)
        ssc_module_disable (ssc, SSC_TX);

    if (ssc->rx)
        ssc_module_disable (ssc, SSC_RX);
} 


/* Enable all of the modules.  */
void
ssc_enable (ssc_t *ssc)
{
    if (ssc->tx)
        ssc_module_enable (ssc, SSC_TX);

    if (ssc->rx)
        ssc_module_enable (ssc, SSC_RX);
}


/* Read data from the rx buffer.  */
uint16_t
ssc_read (ssc_t *ssc, void *buffer, uint16_t length)
{
    uint8_t *dst = buffer;
    int i;

    for (i = 0; i < length; i++)
    {
        while (!ssc_read_ready_p (ssc))
            continue;
        
        *dst++ = SSC->SSC_RHR;
    }
    
    return length;
}


/* Write to the transmit buffer.  */
uint16_t
ssc_write (ssc_t *ssc, void *buffer, uint16_t length)
{
    uint8_t *src = buffer;
    int i;

    for (i = 0; i < length; i++)
    {
        while (!ssc_write_ready_p (ssc))
            continue;

        SSC->SSC_THR = *src++;        
    }
    
    return length;
}


/* Init and configure, still need to be enabled after calling this.  */
ssc_t *
ssc_init (const ssc_cfg_t *cfg)
{
    ssc_t *ssc;

    ssc = &ssc_dev;

    ssc_config (ssc, cfg);

    ssc->rx = cfg->rx;
    ssc->tx = cfg->tx;

    ssc_enable (ssc);

    return ssc;
}


/** @file ssc_dma.c
    @author Stuart Duncan, Michael Hayes
    @date 30 January 2012
    @version 1.0
    @breif DMA routines for the SSC peripheral
    
    This DMA driver is essentially the same as the SPI one, but with
    different base controller addresses.  It could be neater to have
    one PDC driver with its own configuration structure rather than a
    separate DMA driver for each peripheral with a PDC interface.  */

#include "ssc.h"
#include "ssc_dma.h"
#include "config.h"
#include "bits.h"


/* Note, this has been superseded by pdc.  */

bool
ssc_dma_write_finished_p (void)
{
    return (SSC->SSC_SR & SSC_SR_ENDTX) != 0;
}


bool
ssc_dma_read_finished_p (void)
{
    return (SSC->SSC_SR & SSC_SR_ENDRX) != 0;
}


bool
ssc_dma_write_completed_p (void)
{
    return PDC_SSC->PERIPH_TNCR == 0;
}


bool
ssc_dma_read_completed_p (void)
{
    return PDC_SSC->PERIPH_TNCR == 0;
}


bool
ssc_dma_write_enable_p (void)
{
    return (PDC_SSC->PERIPH_PTCR & PERIPH_PTCR_TXTEN) != 0;
}


bool
ssc_dma_read_enable_p (void)
{
    return (PDC_SSC->PERIPH_PTCR & PERIPH_PTCR_RXTEN) != 0;
}


void
ssc_dma_write_enable (void)
{
    /* Enable receiver.  */
    PDC_SSC->PERIPH_PTCR = PERIPH_PTCR_TXTEN;
}


void 
ssc_dma_write_disable (void)
{
    /* Disable receiver.  */
    PDC_SSC->PERIPH_PTCR = PERIPH_PTCR_TXTDIS;
}


void
ssc_dma_read_enable (void)
{
    /* Enable receiver.  */
    PDC_SSC->PERIPH_PTCR = PERIPH_PTCR_RXTEN;
}


void 
ssc_dma_read_disable (void)
{
    /* Disable receiver.  */
    PDC_SSC->PERIPH_PTCR = PERIPH_PTCR_RXTDIS;
}


void
ssc_dma_write_init (void *buffer, uint16_t size)
{
    ssc_dma_write_disable ();

    PDC_SSC->PERIPH_TNPR = 0;
    PDC_SSC->PERIPH_TNCR = 0;

    PDC_SSC->PERIPH_TPR = (uint32_t) buffer;
    PDC_SSC->PERIPH_TCR = size;

    ssc_dma_write_enable ();
}

void
ssc_dma_read_init (void *buffer, uint16_t size)
{
    ssc_dma_read_disable ();

    PDC_SSC->PERIPH_RNPR = 0;
    PDC_SSC->PERIPH_RNCR = 0;

    PDC_SSC->PERIPH_RPR = (uint32_t) buffer;
    PDC_SSC->PERIPH_RCR = size;

    ssc_dma_read_enable ();
}


void
ssc_dma_write_next (void *buffer, uint16_t size)
{
    PDC_SSC->PERIPH_TNPR = (uint32_t) buffer;
    PDC_SSC->PERIPH_TNCR = size;
}


void
ssc_dma_read_next (void *buffer, uint16_t size)
{
    PDC_SSC->PERIPH_RNPR = (uint32_t) buffer;
    PDC_SSC->PERIPH_RNCR = size;
}


void
ssc_dma_init (void *txbuffer, void *rxbuffer, uint16_t size)
{
    ssc_dma_read_init (rxbuffer, size);
    ssc_dma_write_init (txbuffer, size);
}


void
ssc_dma_next (void *txbuffer, void *rxbuffer, uint16_t size)
{
    ssc_dma_read_next (rxbuffer, size);
    ssc_dma_write_next (txbuffer, size);
}


void 
ssc_dma_enable (void)
{
    ssc_dma_read_enable ();
    ssc_dma_write_enable ();
}


void 
ssc_dma_disable (void)
{
    ssc_dma_read_disable ();
    ssc_dma_write_disable ();
}

/** @file   pdc.c
    @author Michael Hayes
    @date   2 July 2014

   @brief Routines for interfacing to a PDC.  This provides DMA transfer
   for peripherals.
*/

#include "pdc.h"
#include "config.h"
#include "bits.h"

/*
  For peripherals with an associated PDC, the PDC address is at a
  fixed offset from the start of the peripheral's registers.

  The following flags are stored in the peripheral's status register
  (SR) sometimes called ISR.

  ENDRX flag is set when the RCR register reaches zero.
  RXBUFF flag is set when both RCR and RNCR reach zero.
  ENDTX flag is set when the TCR register reaches zero.
  TXBUFE flag is set when both TCR and TNCR reach zero.

  Unfortunately, the ISR, IER, IDR, and IMR, have different offsets
  for different peripherals.
*/

#ifndef PDC_NUM
#define PDC_NUM 4
#endif

static pdc_dev_t pdc_dev[PDC_NUM];
static uint8_t pdc_num = 0;



/** Return true if DMA has nothing more to write.  */
bool
pdc_write_completed_p (pdc_t pdc)
{
    return pdc->base->PERIPH_TNCR == 0;
}


/** Return true if DMA has nothing more to read.  */
bool
pdc_read_completed_p (pdc_t pdc)
{
    return pdc->base->PERIPH_TNCR == 0;
}


bool
pdc_write_enable_p (pdc_t pdc)
{
    return (pdc->base->PERIPH_PTCR & PERIPH_PTCR_TXTEN) != 0;
}


bool
pdc_read_enable_p (pdc_t pdc)
{
    return (pdc->base->PERIPH_PTCR & PERIPH_PTCR_RXTEN) != 0;
}


void
pdc_write_enable (pdc_t pdc)
{
    /* Enable transmitter.  */
    pdc->base->PERIPH_PTCR = PERIPH_PTCR_TXTEN;
}


void
pdc_write_disable (pdc_t pdc)
{
    /* Disable transmitter.  */
    pdc->base->PERIPH_PTCR = PERIPH_PTCR_TXTDIS;
}


void
pdc_read_enable (pdc_t pdc)
{
    /* Enable receiver.  */
    pdc->base->PERIPH_PTCR = PERIPH_PTCR_RXTEN;
}


void
pdc_read_disable (pdc_t pdc)
{
    /* Disable receiver.  */
    pdc->base->PERIPH_PTCR = PERIPH_PTCR_RXTDIS;
}


void
pdc_write_init (pdc_t pdc, void *buffer, uint16_t size)
{
    pdc_write_disable (pdc);

    pdc->base->PERIPH_TNPR = 0;
    pdc->base->PERIPH_TNCR = 0;

    pdc->base->PERIPH_TPR = (uint32_t) buffer;
    pdc->base->PERIPH_TCR = size;
}


void
pdc_read_init (pdc_t pdc, void *buffer, uint16_t size)
{
    pdc_read_disable (pdc);

    pdc->base->PERIPH_RNPR = 0;
    pdc->base->PERIPH_RNCR = 0;

    pdc->base->PERIPH_RPR = (uint32_t) buffer;
    pdc->base->PERIPH_RCR = size;
}


void
pdc_write_next (pdc_t pdc, void *buffer, uint16_t size)
{
    pdc->base->PERIPH_TNPR = (uint32_t) buffer;
    pdc->base->PERIPH_TNCR = size;
}


void
pdc_read_next (pdc_t pdc, void *buffer, uint16_t size)
{
    pdc->base->PERIPH_RNPR = (uint32_t) buffer;
    pdc->base->PERIPH_RNCR = size;
}


void
pdc_next (pdc_t pdc, void *txbuffer, void *rxbuffer, uint16_t size)
{
    pdc_read_next (pdc, rxbuffer, size);
    pdc_write_next (pdc, txbuffer, size);
}


pdc_t
pdc_init (void *base, pdc_descriptor_t *tx, pdc_descriptor_t *rx)
{
    pdc_t pdc;

    if (pdc_num >= PDC_NUM)
        return 0;

    pdc = &pdc_dev[pdc_num++];

    pdc->base = base;
    pdc->rx_count = 0;
    pdc->tx_count = 0;

    if (tx)
    {
        pdc_write_init (pdc, tx->buffer, tx->size);
        tx = tx->next;
        if (tx)
        {
            pdc_write_next (pdc, tx->buffer, tx->size);
            tx = tx->next;
        }
        pdc_write_enable (pdc);
    }
    pdc->tx = tx;

    if (rx)
    {
        pdc_read_init (pdc, rx->buffer, rx->size);
        rx = rx->next;
        if (rx)
        {
            pdc_read_next (pdc, rx->buffer, rx->size);
            rx = rx->next;
        }
        pdc_read_enable (pdc);
    }
    pdc->rx = rx;




    return pdc;
}

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

  The transfer count specifies the number of transfers.

  The memory pointer is incremented by 1, 2 or 4 bytes for byte,
  half-word or word transfers.
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


/** This polls the PDC and updates the next transfer pointer and count
    as appropriate from the list of descriptors.  It could be called
    from an ISR.  */
pdc_descriptor_t *
pdc_read_poll (pdc_t pdc)
{
    pdc_descriptor_t *current;

    current = pdc->rx.current;

    if (!current)
        return 0;

    /* When a transfer is complete, RNCR -> RCR and 0 -> RNCR.  */
    if (pdc->base->PERIPH_RNCR != 0)
        return 0;
        
    pdc->rx.count++;

    pdc->rx.current = current->next;

    if (!pdc->rx.current)
    {
        pdc_read_disable (pdc);
        return current;
    }
    
    if (pdc->rx.current->next)
        pdc_read_next (pdc, pdc->rx.current->next->buffer, 
                       pdc->rx.current->next->size);

    return current;
}


/** This polls the PDC and updates the next transfer pointer and count
    as appropriate from the list of descriptors.  It could be called
    from an ISR.  */
pdc_descriptor_t *
pdc_write_poll (pdc_t pdc)
{
    pdc_descriptor_t *current;

    current = pdc->tx.current;

    if (!current)
        return 0;

    /* When a transfer is complete, TNCR -> TCR and 0 -> TNCR.  */
    if (pdc->base->PERIPH_TNCR != 0)
        return 0;
        
    pdc->tx.count++;

    pdc->tx.current = current->next;

    if (!pdc->tx.current)
    {
        pdc_write_disable (pdc);
        return current;
    }
    
    if (pdc->tx.current->next)
        pdc_write_next (pdc, pdc->tx.current->next->buffer, 
                        pdc->tx.current->next->size);

    return current;
}


void
pdc_write_config (pdc_t pdc, pdc_descriptor_t *tx)
{
    pdc_write_disable (pdc);

    pdc->tx.current = tx;
    pdc->tx.count = 0;

    if (!tx)
        return;
        
    pdc->base->PERIPH_TPR = (uint32_t) tx->buffer;
    /* This starts the transfer if channel active.  */
    pdc->base->PERIPH_TCR = tx->size;
    
    if (tx->next)
    {
        pdc->base->PERIPH_TNPR = (uint32_t) tx->next->buffer;
        pdc->base->PERIPH_TNCR = tx->next->size;
    }
    else
    {
        pdc->base->PERIPH_TNPR = 0;
        pdc->base->PERIPH_TNCR = 0;
    }
}


void
pdc_read_config (pdc_t pdc, pdc_descriptor_t *rx)
{
    pdc_read_disable (pdc);

    pdc->rx.current = rx;
    pdc->rx.count = 0;
    if (!rx)
        return;

    /* Need to write to RNCR before RCR otherwise RNCR gets set to 0.  */
    
    pdc->base->PERIPH_RPR = (uint32_t) rx->buffer;
    /* This starts the transfer if channel active.  */
    pdc->base->PERIPH_RCR = rx->size;
    
    if (rx->next)
    {
        pdc->base->PERIPH_RNPR = (uint32_t) rx->next->buffer;
        pdc->base->PERIPH_RNCR = rx->next->size;
    }
    else
    {
        pdc->base->PERIPH_RNPR = 0;
        pdc->base->PERIPH_RNCR = 0;
    }
}


void
pdc_config (pdc_t pdc, pdc_descriptor_t *tx, pdc_descriptor_t *rx)
{
    pdc_write_config (pdc, tx);
    pdc_read_config (pdc, rx);
}


void
pdc_start (pdc_t pdc)
{
    if (pdc->rx.current)
        pdc_read_enable (pdc);

    if (pdc->tx.current)
        pdc_write_enable (pdc);
}


void
pdc_stop (pdc_t pdc)
{
    if (pdc->rx.current)
        pdc_read_disable (pdc);

    if (pdc->tx.current)
        pdc_write_disable (pdc);
}


pdc_t
pdc_init (void *base, pdc_descriptor_t *tx, pdc_descriptor_t *rx)
{
    pdc_t pdc;

    if (pdc_num >= PDC_NUM)
        return 0;

    pdc = &pdc_dev[pdc_num++];

    pdc->base = base;

    pdc_config (pdc, tx, rx);

    return pdc;
}

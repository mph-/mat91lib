#include "bssc.h"
#include "ssc_dma.h"
#include "irq.h"
#include "pio.h"


static bssc_t *bssc_dev;


static void bssc_advance (bssc_t *dev)
{
    char *next;
    
    /** Advance the pointers for the DMA to use (ring isn't written
        via ring_write) ignore overrun for now.  */
    ring_write_advance (dev->ring, dev->dma_size);

    next = ring_write_next (dev->ring, dev->dma_size);
    ssc_dma_next (0, next, dev->dma_size);
}


/** bssc_isr
   advance the ring buffer once the data has been written
   */
static void bssc_isr (void)
{
    bssc_advance (bssc_dev);
}


/** bssc_init
   Initialize the ssc with a buffer using DMA
   @param ssc_cfg_t *cfg, ssc configuration
   @param char *buffer, buffer to save to
   @param size, the size of the buffer in bytes
   @param dma_size, the size of each DMA transfer in bytes
   @return bssc_t *, the pointer to the buffered ssc structure
   */
void
bssc_init (bssc_t *dev, const ssc_cfg_t *cfg,
           char *buffer, uint16_t size, uint16_t dma_size)
{
    bssc_dev = dev;
    
    if (dma_size >= size)
        dev = 0;
    
    ring_init (dev->ring, buffer, size);
    ssc_init (cfg);
    
    ssc_dma_init (0, buffer, dma_size);
    ssc_dma_next (0, buffer + dma_size, dma_size);
    
    ssc_dma_enable ();
    
    dev->dma_size = dma_size;
    dev->size = size;
    
    // Enable peripheral interrupts
    SSC->SSC_IER = BIT(2);
    
    // Configure the ISR
    irq_config (ID_SSC, 1, bssc_isr);
    
    // Enable interrupts
    irq_enable (ID_SSC);
    
    dev->cfg = *cfg;
    dev->buffer = buffer;
}


int bssc_read_num (bssc_t *dev)
{
    return ring_read_num (dev->ring);
}


int bssc_read (bssc_t *dev, void *buffer, int size)
{
    return ring_read (dev->ring, buffer, size);
}

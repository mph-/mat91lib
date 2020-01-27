#include <stdlib.h>
#include <string.h>
#include "cadc.h"
#include "adc.h"
#include "pdc.h"
#include "irq.h"
#include "capture.h"
#include "delay.h"


#ifndef ADC_IRQ_PRIORITY
#define ADC_IRQ_PRIORITY 4
#endif    


struct cadc_dev
{
    tc_t tc;
    pdc_t pdc;
    adc_t adc;
    uint16_t dma_size;
    uint16_t *prev;
    uint32_t count;
    uint8_t num_channels;
    volatile uint32_t isr_count;
    capture_t *cap;
    uint8_t num_buffers;
    bool started;
    pdc_descriptor_t *descriptors;
    volatile adc_sample_t **buffers;    
};


static struct cadc_dev cadc_dev;



static void
cadc_isr (void)
{
    pdc_descriptor_t *descr;
    cadc_t dev = &cadc_dev;
    
    descr = pdc_read_poll (dev->pdc);

    // Something is wrong if the descriptor is zero; it means the
    // transfer has not finished.  Perhaps another ADC interrupt is
    // enabled?
    if (!descr)
        return;

    dev->isr_count++;

    capture_update (dev->cap, descr->buffer, dev->prev, dev->dma_size);
    
    dev->prev = descr->buffer;
}


cadc_t cadc_init (const cadc_cfg_t *cfg)
{
    int i;
    uint32_t channels;
    uint8_t num_channels;
    cadc_t dev = &cadc_dev;

    dev->cap = capture_init ();
    if (! dev->cap)
        return 0;

    dev->dma_size = cfg->dma_size;
    dev->num_buffers = cfg->num_buffers;
    if (dev->num_buffers < 3)
        dev->num_buffers = 3;

    /* Generate sampling clock.  */
    dev->tc = tc_init (&cfg->tc);
    if (! dev->tc)
        return 0;

    dev->adc = adc_init (&cfg->adc);
    if (! dev->adc)
        return 0;

    tc_start (dev->tc);

    channels = cfg->adc.channels;
    num_channels = 0;
    while (channels)
    {
        if (channels & 1)
            num_channels++;
        channels >>= 1;
    }
    dev->num_channels = num_channels;

    if (! num_channels)
        return 0;

    dev->descriptors = calloc (dev->num_buffers, sizeof (*dev->descriptors));
    
    for (i = 0; i < dev->num_buffers; i++)
    {
        dev->descriptors[i].buffer = calloc (dev->dma_size, sizeof (uint16_t));
        dev->descriptors[i].size = dev->dma_size;
        dev->descriptors[i].next = &dev->descriptors[(i + 1) % dev->num_buffers];
    }

    dev->prev = 0;
    dev->started = 0;
    dev->pdc = pdc_init (adc_pdc_get (dev->adc), 0, dev->descriptors);

    ADC->ADC_IER = ADC_ISR_ENDRX;
    irq_config (ID_ADC, ADC_IRQ_PRIORITY, cadc_isr);
    irq_enable (ID_ADC);
    
    return dev;
}


void
cadc_start (void)
{
    cadc_t dev = &cadc_dev;
    
    if (dev->started)
        return;
    
    pdc_config (cadc_dev.pdc, 0, dev->descriptors);

    // There is a problem with restarting.   It sees like the ADC
    // needs a hardware reset to reset the multiplexer sequencer
    // but something is not configured correctly after the reset.
    // It is best to leave the ADC running.
    
    adc_enable (dev->adc);
    pdc_start (dev->pdc);
    cadc_dev.count = 0;
    cadc_dev.prev = 0;
    dev->started = 1;    
}


void
cadc_stop (void)
{
    // adc_disable does nothing; the ADC runs on...
    adc_disable (cadc_dev.adc);
    // ... but the DMA is stopped
    pdc_stop (cadc_dev.pdc);
}


void
cadc_shutdown (void)
{
    cadc_stop ();
    adc_shutdown (cadc_dev.adc);
    tc_shutdown (cadc_dev.tc);
}


bool
cadc_captured_p (void)
{
    return capture_ready_p (cadc_dev.cap);
}


bool
cadc_capture_start (adc_sample_t *buffer, uint16_t pretrigger,
                    uint16_t samples,
                    adc_sample_t high_threshold,
                    adc_sample_t low_threshold,
                    capture_callback_t callback)
{
    cadc_t dev = &cadc_dev;    

    if (pretrigger > dev->dma_size)
        pretrigger = dev->dma_size;
    if (pretrigger > samples)
        pretrigger = samples;
    
    capture_start (dev->cap, buffer, pretrigger,
                   samples, dev->num_channels,
                   high_threshold,
                   low_threshold, callback);

    dev->prev = 0;
    cadc_start ();
    
    return 1;
}


void
cadc_capture_stop (void)
{
    cadc_t dev = &cadc_dev;
    
    capture_stop (dev->cap);
}

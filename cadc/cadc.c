/** @file   cadc.c
    @author M. P. Hayes
    @date   12 February 2020
    @brief  Continuous analogue to digital converter using DMA
*/


#include <stdlib.h>
#include <string.h>
#include "cadc.h"
#include "adc.h"
#include "pdc.h"
#include "irq.h"
#include "delay.h"
#include "bits.h"


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
    volatile uint32_t isr_count;
    uint8_t num_buffers;
    uint8_t num_channels;
    uint8_t last_channel;
    bool started;
    pdc_descriptor_t *descriptors;
    volatile adc_sample_t **buffers;
    void *callback_data;
    cadc_callback_t callback_func;
};

static struct cadc_dev cadc_dev;

void cadc_sync (cadc_t dev, uint8_t channel)
{
    uint8_t val;

    // FIXME, brutal hack to fix ADC synchronisation.  This blocks
    // and should have a timeout.
    while (1)
    {
        val = ADC->ADC_LCDR >> 12;
        if (val == channel)
            break;
    }
}


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

    if (dev->callback_func)
        dev->callback_func (dev->callback_data, descr->buffer,
                            dev->prev, dev->dma_size);

    dev->prev = descr->buffer;
}


cadc_t cadc_init (const cadc_cfg_t *cfg)
{
    int i;
    uint32_t channels;
    uint8_t num_channels;
    cadc_t dev = &cadc_dev;

    dev->callback_func = 0;

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

    adc_calibrate (dev->adc);

    // Enable tagging of the data.  The four MSBs of each 16-bit sample
    // specify the channel number.  This is useful for when the ADC MUX
    // gets out of whack.
    adc_tag_set (dev->adc, 1);
    adc_config (dev->adc);

    channels = cfg->adc.channels;
    dev->num_channels = 0;
    for (i = 0; channels; channels >>= 1, i++)
    {
        if (channels & 1)
        {
            dev->num_channels++;
            dev->last_channel = i;
        }
    }

    if (! dev->num_channels)
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

    irq_config (ID_ADC, ADC_IRQ_PRIORITY, cadc_isr);
    irq_enable (ID_ADC);

    tc_start (dev->tc);

    ADC->ADC_IER = ADC_ISR_ENDRX;

    return dev;
}


void
cadc_start (cadc_t dev)
{
    if (dev->started)
        return;

    pdc_config (dev->pdc, 0, dev->descriptors);

    // There is a problem with restarting.   It sees like the ADC
    // needs a hardware reset to reset the multiplexer sequencer
    // but something is not configured correctly after the reset.
    // It is best to leave the ADC running.

    adc_enable (dev->adc);

    // cadc_sync (dev, 0);

    pdc_start (dev->pdc);
    dev->count = 0;
    dev->prev = 0;
    dev->started = 1;
    dev->prev = 0;
}


void
cadc_stop (cadc_t dev)
{
    // adc_disable does nothing; the ADC runs on...
    adc_disable (dev->adc);
    // ... but the DMA is stopped
    pdc_stop (dev->pdc);
}


void
cadc_shutdown (cadc_t dev)
{
    cadc_stop (dev);
    adc_shutdown (dev->adc);
    tc_shutdown (dev->tc);
}


void cadc_callback_register (cadc_t dev,
                             cadc_callback_t callback_func,
                             void *callback_data)
{
    dev->callback_func = callback_func;
    dev->callback_data = callback_data;
}


uint8_t cadc_num_channels_get (cadc_t dev)
{
    return dev->num_channels;
}

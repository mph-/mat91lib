/** @file   adc.h
    @author M. P. Hayes, UCECE
    @date   3 Feb 2005

    @brief Routines to use AT91 onboard ADC in polling mode.
*/

#ifndef CADC_H
#define CADC_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"
#include "sys.h"
#include "adc.h"
#include "tc.h"
    

typedef struct
{
    adc_cfg_t adc;
    tc_cfg_t tc;
    uint16_t dma_size;
    // Minimum 3.
    uint8_t num_buffers;
} cadc_cfg_t;


typedef struct cadc_dev *cadc_t;    

typedef void (*cadc_callback_t) (void *handle, adc_sample_t *buffer,
                                 adc_sample_t *prev_buffer, uint16_t size);
    
cadc_t cadc_init (const cadc_cfg_t *cfg);

void cadc_start (cadc_t dev);

void cadc_stop (cadc_t dev);

uint8_t cadc_num_channels_get (cadc_t dev);
    
void cadc_callback_register (cadc_t dev, cadc_callback_t callback_func,
                             void *callback_data);

void cadc_shutdown (cadc_t dev);

    
#ifdef __cplusplus
}
#endif    
#endif

    

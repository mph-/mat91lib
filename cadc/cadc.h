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
#include "capture.h"    
    

typedef struct
{
    adc_cfg_t adc;
    tc_cfg_t tc;
    uint16_t dma_size;
    // Minimum 3.
    uint8_t num_buffers;
} cadc_cfg_t;


typedef struct cadc_dev *cadc_t;    


cadc_t cadc_init (const cadc_cfg_t *cfg);

void cadc_start (void);

void cadc_stop (void);

void cadc_shutdown (void);

bool cadc_captured_p (void);

bool cadc_capture_start (adc_sample_t *buffer, adc_sample_t pretrigger,
                         adc_sample_t samples,
                         adc_sample_t high_threshold,
                         adc_sample_t low_threshold,
                         capture_callback_t callback);

void cadc_capture_stop (void);

    
#ifdef __cplusplus
}
#endif    
#endif

    

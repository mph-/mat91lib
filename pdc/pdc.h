/** @file   pdc.h
    @author Michael Hayes
    @date   2 July 2014

    @brief Routines for interfacing to the PDC.
*/

#ifndef PDC_H
#define PDC_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"


typedef struct pdc_descriptor_struct 
{
    void *buffer;
    uint16_t size;
    struct pdc_descriptor_struct *next;
} pdc_descriptor_t;


typedef struct pdc_module_struct
{
    pdc_descriptor_t *current;
    uint32_t count;
} pdc_module_t;


typedef struct pdc_dev_struct
{
    Pdc *base;
    pdc_module_t tx;
    pdc_module_t rx;
} pdc_dev_t;


typedef pdc_dev_t *pdc_t;


/** Return true if DMA has finished writing a buffer.  */
bool
pdc_write_finished_p (pdc_t pdc);


/** Return true if DMA has finished reading a buffer.  */
bool
pdc_read_finished_p (pdc_t pdc);


/** Return true if DMA has nothing more to write.  */
bool
pdc_write_completed_p (pdc_t pdc);


/** Return true if DMA has nothing more to read.  */
bool
pdc_read_completed_p (pdc_t pdc);


bool
pdc_write_enable_p (pdc_t pdc);


bool
pdc_read_enable_p (pdc_t pdc);


void
pdc_write_enable (pdc_t pdc);


void 
pdc_write_disable (pdc_t pdc);


void
pdc_read_enable (pdc_t pdc);


void 
pdc_read_disable (pdc_t pdc);


void
pdc_start (pdc_t pdc);


void
pdc_stop (pdc_t pdc);


pdc_descriptor_t *
pdc_read_poll (pdc_t pdc);


pdc_descriptor_t *
pdc_write_poll (pdc_t pdc);


void
pdc_config (pdc_t pdc, pdc_descriptor_t *tx, pdc_descriptor_t *rx);


pdc_t
pdc_init (void *base, pdc_descriptor_t *tx, pdc_descriptor_t *rx);


void
pdc_enable (pdc_t pdc);


void
pdc_disable (pdc_t pdc);


#ifdef __cplusplus
}
#endif    
#endif 


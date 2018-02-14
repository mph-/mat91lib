/** @file   bspi.h
    @author Stuart Duncan, UC ECE
    @date   25 January 2012   
    @brief  Buffered SSC.
*/
#ifndef BSSC_H
#define BSSC_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"
#include "ssc.h"
#include "ring.h"


typedef struct bssc_struct bssc_t;
struct bssc_struct
{
    ssc_cfg_t  cfg;
    ssc_t      ssc;
    ring_t     *ring;
    void       *buffer;
    uint16_t   dma_size;
    uint16_t   size;
};


int bspi_read_num (bssc_t *dev);


int bspi_read (bssc_t *dev, void *buffer, int size);


void
bssc_init (bssc_t *, const ssc_cfg_t *, char *, uint16_t, uint16_t );


#ifdef __cplusplus
}
#endif    
#endif


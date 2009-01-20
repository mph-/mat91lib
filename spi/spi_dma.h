/** @file   spi_dma.c
    @author Michael Hayes
    @date   18 December 2007

    @brief Routines for interfacing to the SPI bus with DMA access.
*/

#ifndef SPI_DMA_H
#define SPI_DMA_H

#include "config.h"

extern bool
spi_dma_write_finished_p (void);


extern bool
spi_dma_read_finished_p (void);


extern bool
spi_dma_write_completed_p (void);


extern bool
spi_dma_read_completed_p (void);


extern bool
spi_dma_write_enable_p (void);


extern bool
spi_dma_read_enable_p (void);


extern void
spi_dma_write_enable (void);


extern void 
spi_dma_write_disable (void);


extern void
spi_dma_read_enable (void);


extern void 
spi_dma_read_disable (void);


extern void
spi_dma_write_init (void *buffer, uint16_t size);

extern void
spi_dma_read_init (void *buffer, uint16_t size);

extern void
spi_dma_write_next (void *buffer, uint16_t size);

extern void
spi_dma_read_next (void *buffer, uint16_t size);

#endif

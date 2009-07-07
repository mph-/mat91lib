/** @file   spi_dma.c
    @author Michael Hayes
    @date   18 December 2007

    @brief Routines for interfacing to the SPI bus with DMA access.
*/

#ifndef SPI_DMA_H
#define SPI_DMA_H

#include "config.h"

extern bool
spi_dma_write_finished_p (spi_t spi);


extern bool
spi_dma_read_finished_p (spi_t spi);


extern bool
spi_dma_write_completed_p (spi_t spi);


extern bool
spi_dma_read_completed_p (spi_t spi);


extern bool
spi_dma_write_enable_p (spi_t spi);


extern bool
spi_dma_read_enable_p (spi_t spi);


extern void
spi_dma_write_enable (spi_t spi);


extern void 
spi_dma_write_disable (spi_t spi);


extern void
spi_dma_read_enable (spi_t spi);


extern void 
spi_dma_read_disable (spi_t spi);


extern void
spi_dma_write_init (spi_t spi, void *buffer, uint16_t size);


extern void
spi_dma_read_init (spi_t spi, void *buffer, uint16_t size);


extern void
spi_dma_write_next (spi_t spi, void *buffer, uint16_t size);


extern void
spi_dma_read_next (spi_t spi, void *buffer, uint16_t size);


extern void
spi_dma_init (spi_t spi, void *txbuffer, void *rxbuffer, uint16_t size);


extern void
spi_dma_next (spi_t spi, void *txbuffer, void *rxbuffer, uint16_t size);


extern void
spi_dma_enable (spi_t spi);


extern void
spi_dma_disable (spi_t spi);

#endif

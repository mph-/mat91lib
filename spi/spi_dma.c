/** @file   spi_dma.c
    @author Michael Hayes
    @date   18 December 2007

   @brief Routines for interfacing to the SPI bus with DMA access.
*/

#include "spi.h"
#include "spi_dma.h"
#include "config.h"
#include "bits.h"

/* The SPI peripheral has been designed so that blocks of a data can
   be streamed using DMA to multiple devices on the bus (Variable
   Peripheral Selection).  It achieves this by using 32 bits to
   specify the 8 or 16 bit data plus which peripheral to write to.
*/

bool
spi_dma_write_finished_p (void)
{
    return (AT91C_BASE_SPI->SPI_SR & AT91C_SPI_ENDTX) != 0;
}


bool
spi_dma_read_finished_p (void)
{
    return (AT91C_BASE_SPI->SPI_SR & AT91C_SPI_ENDRX) != 0;
}


bool
spi_dma_write_enable_p (void)
{
    AT91S_PDC *pPDC = AT91C_BASE_PDC_SPI;

    return (pPDC->PDC_PTCR & AT91C_PDC_TXTEN) != 0;
}


bool
spi_dma_read_enable_p (void)
{
    AT91S_PDC *pPDC = AT91C_BASE_PDC_SPI;

    return (pPDC->PDC_PTCR & AT91C_PDC_RXTEN) != 0;
}


void
spi_dma_write_enable (void)
{
    AT91S_PDC *pPDC = AT91C_BASE_PDC_SPI;

    /* Enable receiver.  */
    pPDC->PDC_PTCR = AT91C_PDC_TXTEN;
}


void 
spi_dma_write_disable (void)
{
    AT91S_PDC *pPDC = AT91C_BASE_PDC_SPI;

    /* Disable receiver.  */
    pPDC->PDC_PTCR = AT91C_PDC_TXTDIS;
}


void
spi_dma_read_enable (void)
{
    AT91S_PDC *pPDC = AT91C_BASE_PDC_SPI;

    /* Enable receiver.  */
    pPDC->PDC_PTCR = AT91C_PDC_RXTEN;
}


void 
spi_dma_read_disable (void)
{
    AT91S_PDC *pPDC = AT91C_BASE_PDC_SPI;

    /* Disable receiver.  */
    pPDC->PDC_PTCR = AT91C_PDC_RXTDIS;
}


void spi_dma_write_init (void *address, uint16_t size)
{
    AT91S_PDC *pPDC = AT91C_BASE_PDC_SPI;

    spi_dma_write_disable ();

    /* No chaining for now.  */
    pPDC->PDC_TNPR = (uint32_t) 0;
    pPDC->PDC_TNCR = 0;

    pPDC->PDC_TPR = (uint32_t) address;
    pPDC->PDC_TCR = size;

    spi_dma_write_enable ();
}


void spi_dma_read_init (void *address, uint16_t size)
{
    AT91S_PDC *pPDC = AT91C_BASE_PDC_SPI;

    spi_dma_read_disable ();

    /* No chaining for now.  */
    pPDC->PDC_RNPR = (uint32_t) 0;
    pPDC->PDC_RNCR = 0;

    pPDC->PDC_RPR = (uint32_t) address;
    pPDC->PDC_RCR = size;

    spi_dma_read_enable ();
}

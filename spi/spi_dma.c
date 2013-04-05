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

   ENDRX flag is set when the RCR register reaches zero.
   RXBUFF flag is set when both RCR and RNCR reach zero.
   ENDTX flag is set when the TCR register reaches zero.
   TXBUFE flag is set when both TCR and TNCR reach zero.

*/


#ifndef AT91C_BASE_SPI0
#define AT91C_BASE_SPI0 AT91C_BASE_SPI
#define AT91C_BASE_PDC_SPI0 AT91C_BASE_PDC_SPI
#endif

#ifndef AT91C_BASE_SPI1
#define AT91C_BASE_SPI1 0
#define AT91C_BASE_PDC_SPI1 0
#endif


/* Number of channels per controller.  */
#define SPI_CHANNELS_NUM 4


#define SPI_BASE_GET(spi) (((spi)->channel < SPI_CHANNELS_NUM) ? AT91C_BASE_SPI0 : AT91C_BASE_SPI1)
#define PDC_BASE_GET(spi) (((spi)->channel < SPI_CHANNELS_NUM) ? AT91C_BASE_PDC_SPI0 : AT91C_BASE_PDC_SPI1)


/** Return true if DMA has finished writing a buffer.  */
bool
spi_dma_write_finished_p (spi_t spi)
{
    AT91S_SPI *pSPI = SPI_BASE_GET (spi);

    return (pSPI->SPI_SR & AT91C_SPI_ENDTX) != 0;
}


/** Return true if DMA has finished reading a buffer.  */
bool
spi_dma_read_finished_p (spi_t spi)
{
    AT91S_SPI *pSPI = SPI_BASE_GET (spi);

    return (pSPI->SPI_SR & AT91C_SPI_ENDRX) != 0;
}


/** Return true if DMA has nothing more to write.  */
bool
spi_dma_write_completed_p (spi_t spi)
{
    AT91S_PDC *pPDC = PDC_BASE_GET (spi);
    
    return pPDC->PDC_TNCR == 0;
}


/** Return true if DMA has nothing more to read.  */
bool
spi_dma_read_completed_p (spi_t spi)
{
    AT91S_PDC *pPDC = PDC_BASE_GET (spi);
    
    return pPDC->PDC_TNCR == 0;
}


bool
spi_dma_write_enable_p (spi_t spi)
{
    AT91S_PDC *pPDC = PDC_BASE_GET (spi);

    return (pPDC->PDC_PTCR & AT91C_PDC_TXTEN) != 0;
}


bool
spi_dma_read_enable_p (spi_t spi)
{
    AT91S_PDC *pPDC = PDC_BASE_GET (spi);

    return (pPDC->PDC_PTCR & AT91C_PDC_RXTEN) != 0;
}


void
spi_dma_write_enable (spi_t spi)
{
    AT91S_PDC *pPDC = PDC_BASE_GET (spi);

    /* Enable receiver.  */
    pPDC->PDC_PTCR = AT91C_PDC_TXTEN;
}


void 
spi_dma_write_disable (spi_t spi)
{
    AT91S_PDC *pPDC = PDC_BASE_GET (spi);

    /* Disable receiver.  */
    pPDC->PDC_PTCR = AT91C_PDC_TXTDIS;
}


void
spi_dma_read_enable (spi_t spi)
{
    AT91S_PDC *pPDC = PDC_BASE_GET (spi);

    /* Enable receiver.  */
    pPDC->PDC_PTCR = AT91C_PDC_RXTEN;
}


void 
spi_dma_read_disable (spi_t spi)
{
    AT91S_PDC *pPDC = PDC_BASE_GET (spi);

    /* Disable receiver.  */
    pPDC->PDC_PTCR = AT91C_PDC_RXTDIS;
}


/** Enable variable peripheral select.  */
static void
spi_dma_multichannel_select (spi_t spi)
{
    AT91S_SPI *pSPI = SPI_BASE_GET (spi);

    pSPI->SPI_MR |= AT91C_SPI_PS_VARIABLE;
}


#if 0
/** Enable fixed peripheral select.  */
static void
spi_dma_multichannel_deselect (spi_t spi)
{
    AT91S_SPI *pSPI = SPI_BASE_GET (spi);

    BITS_INSERT (pSPI->SPI_MR, 0x0f, 16, 19);
}
#endif


void
spi_dma_write_init (spi_t spi, void *buffer, uint16_t size)
{
    AT91S_PDC *pPDC = PDC_BASE_GET (spi);

    spi_dma_multichannel_select (spi);
    spi_dma_write_disable (spi);

    pPDC->PDC_TNPR = 0;
    pPDC->PDC_TNCR = 0;

    pPDC->PDC_TPR = (uint32_t) buffer;
    pPDC->PDC_TCR = size;

    spi_dma_write_enable (spi);
}


void
spi_dma_read_init (spi_t spi, void *buffer, uint16_t size)
{
    AT91S_PDC *pPDC = PDC_BASE_GET (spi);

    spi_dma_multichannel_select (spi);
    spi_dma_read_disable (spi);

    pPDC->PDC_RNPR = 0;
    pPDC->PDC_RNCR = 0;

    pPDC->PDC_RPR = (uint32_t) buffer;
    pPDC->PDC_RCR = size;

    spi_dma_read_enable (spi);
}


void
spi_dma_write_next (spi_t spi, void *buffer, uint16_t size)
{
    AT91S_PDC *pPDC = PDC_BASE_GET (spi);

    pPDC->PDC_TNPR = (uint32_t) buffer;
    pPDC->PDC_TNCR = size;
}


void
spi_dma_read_next (spi_t spi, void *buffer, uint16_t size)
{
    AT91S_PDC *pPDC = PDC_BASE_GET (spi);

    pPDC->PDC_RNPR = (uint32_t) buffer;
    pPDC->PDC_RNCR = size;
}


void
spi_dma_init (spi_t spi, void *txbuffer, void *rxbuffer, uint16_t size)
{
    spi_dma_read_init (spi, rxbuffer, size);
    spi_dma_write_init (spi, txbuffer, size);
}


void
spi_dma_next (spi_t spi, void *txbuffer, void *rxbuffer, uint16_t size)
{
    spi_dma_read_next (spi, rxbuffer, size);
    spi_dma_write_next (spi, txbuffer, size);
}


void 
spi_dma_enable (spi_t spi)
{
    spi_dma_read_enable (spi);
    spi_dma_write_enable (spi);
}


void 
spi_dma_disable (spi_t spi)
{
    spi_dma_read_disable (spi);
    spi_dma_write_disable (spi);
}

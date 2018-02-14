/** @file ssc_dma.h
    @author Stuart Duncan, Michael Hayes
    @date 30 January 2012
    @version 1.0
    @breif prototypes and definitions for the SSC peripheral DMA routines
    
    */

#ifndef SSC_DMA_H
#define SSC_DMA_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"
#include "ssc.h"

/** ssc_dma_write_finished_p
    Check if the dma write is finished
    @return true if finished, false if not
 */
bool
ssc_dma_write_finished_p ( void );

/** ssc_dma_write_finished_p
    Check if the dma read is finished
    @return true if finished, false if not
 */
bool
ssc_dma_read_finished_p ( void );


bool
ssc_dma_write_completed_p ( void );


bool
ssc_dma_read_completed_p ( void );

/** ssc_dma_write_finished_p
    Check if the write channels is enabled
    @return true if enabled, false if disabled
 */
bool
ssc_dma_write_enable_p ( void );

/** ssc_dma_read_isnebabled
    Check if the read channel is enabled
    @return true if enabled, false if disabled
   */
bool
ssc_dma_read_enable_p ( void );


/** ssc_dma_write_enable
    Enables the write channel
   */
void
ssc_dma_write_enable ( void );


/** ssc_dma_write_disable
    Disable write channel
   */
void 
ssc_dma_write_disable ( void );


/** ssc_dma_read_enable
    Enables the read channel
   */
void
ssc_dma_read_enable ( void );


/** ssc_dma_read_disable
    Disables the read channel
   */
void 
ssc_dma_read_disable ( void );


/** ssc_dma_write_init
    Initialize the dma write channel with the specified buffer and number of
    transfers
    @param pointer to the buffer
    @param uint16_t size of the buffer
 */
void
ssc_dma_write_init ( void *buffer, uint16_t size);


/** ssc_dma_read_init
    Initialize the dma read channel with the specified buffer and number of
    transfers
    @param pointer to the buffer
    @param uint16_t size of the buffer
 */
void
ssc_dma_read_init ( void *buffer, uint16_t size);

/** ssc_dma_write_init
    Initialize the dma write channel with the specified buffer and number of
    transfers
    @param pointer to the buffer
    @param uint16_t size of the buffer
 */
void
ssc_dma_write_next ( void *buffer, uint16_t size);


/** ssc_dma_read_init
    Initialize the dma write channel with the specified buffer and number of
    transfers
    @param *buffer pointer to the buffer
    @param size of the buffer
 */
void
ssc_dma_read_next ( void *buffer, uint16_t size);


/** ssc_dma_init
    Initialize both the read and write channels of the DMA
    @param pointer to the transmit buffer
    @param pointer to the receive buffer
    @param uint16_t size of the buffer
 */
void
ssc_dma_init ( void *txbuffer, void *rxbuffer, uint16_t size);


/** ssc_dma_next
    advances the dma to the next word in the buffer
    @param pointer to the transmit buffer
    @param pointer to the receive buffer
    @param uint16_t size, transfers to do
 */
void
ssc_dma_next ( void *txbuffer, void *rxbuffer, uint16_t size);


/** ssc_dma_enable
    Enable both DMA channels
 */
void
ssc_dma_enable ( void );


/** ssc_dma_disable
    Disable both DMA channels
 */
void
ssc_dma_disable ( void );


#ifdef __cplusplus
}
#endif    
#endif //SSC_DMA_H


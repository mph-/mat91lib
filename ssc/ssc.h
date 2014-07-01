/** @file ssc.h
    @author Stuart Duncan, Michael Hayes
    @date 16 December 2011
    @brief Simple hardware driver for the AT91SAM7 SSC peripheral
   

    The driver does not support interrupts or DMA, and only the RX
    module has been tested (the TX module was sort-of tested via
    loopback mode).

    The AT91SAM7 family all have one ssc controller, which is
    essentially a highly flexible synchronous serial communication
    peripheral.  The flexibility is it's downfall, making it tricky
    and complicated to configure.  Before attempting to use this you
    should thoroughly read the SSC section of the datasheet for a
    clear description of the options available.  
*/

#ifndef SSC_H
#define SSC_H

#include "config.h"
#include "pio.h"  //For selecting the SSC as the pio driver


/* Clock Divider Type */
typedef uint16_t ssc_clock_divisor_t;

/* Data type, SSC allows for up to 32 bit data words */
typedef uint32_t ssc_data_t;

typedef uint32_t ssc_clock_speed_t;


/* Include the big complicated configuration structure for the
 * ssc modules (transmit and receive)
 */
#include "ssc_module.h"


typedef struct 
{
    // Transmitter configuration
    ssc_module_cfg_t *tx;
    
    // Receiver configuration
    ssc_module_cfg_t *rx;

    uint16_t clock_divisor;
} ssc_t;



/* SSC configuration structure, allows tx and rx modules to be independently
 * configured.
 */
typedef struct 
{
    // Transmitter configuration
    ssc_module_cfg_t *tx;
    
    // Receiver configuration
    ssc_module_cfg_t *rx;
    
    /* Clock speed in kHz (maximum).  */
    ssc_clock_speed_t clock_speed_kHz;
} ssc_cfg_t;



/* Function Prototypes */


/** Enable all the modules.
 */
void
ssc_enable (ssc_t *ssc);


/** Disable all the modules
 */
void
ssc_disable (ssc_t *ssc);


bool
ssc_read_ready_p (ssc_t *ssc);


bool
ssc_write_ready_p (ssc_t *ssc);


/** Read the data in the rx buffer TODO (test)
   @return the data read from the buffer
   @param boolean wait for buffer ready, true = wait, false = don't wait
 */
uint16_t
ssc_read (ssc_t *ssc, void *buffer, uint16_t length);


/** Write to the tx buffer TODO (test)
   @param the data to write
   @param boolean wait for ready, true = wait, false = don't wait
 */
uint16_t
ssc_write (ssc_t *ssc, void *buffer, uint16_t length);


/** Initialize/configure the SSC. You still need to enable the
   appropriate modules.  If a module is to be disabled then the
   pointer to its configuration from within the ssc config should be
   null. 
   @param pointer to the ssc configuration structure to apply
 */
ssc_t *
ssc_init (const ssc_cfg_t *);


#endif //SSC_H


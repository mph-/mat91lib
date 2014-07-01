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
typedef uint16_t ssc_clock_div_t;

/* Data type, SSC allows for up to 32 bit data words */
typedef uint32_t ssc_data_t;

/* Include the big complicated configuration structure for the
 * ssc modules (transmit and receive)
 */
#include "ssc_module.h"

/* SSC configuration structure, allows tx and rx modules to be independently
 * configured.
 */
typedef struct 
{
    // Transmitter configuration, null pointer = won't be configured
    ssc_module_cfg_t  *tx_cfg; // From ssc_module.h
    
    // Receiver configuration, null pointer = disabled
    ssc_module_cfg_t  *rx_cfg; // From ssc_module.h
    
    // Clock Div, multiplied by 2 before dividing, 0 = won't be configured
    ssc_clock_div_t clock_div;
} ssc_cfg_t;



/* Function Prototypes */



/** Initialize/configure the SSC. You still need to enable the
   appropriate modules.  If a module is to be disabled then the
   pointer to its configuration from within the ssc config should be
   null. 
   @param pointer to the ssc configuration structure to apply
 */
void
ssc_init (const ssc_cfg_t *);


/** Enable all the modules.
 */
void
ssc_enable (void);


/** Disable all the modules
 */
void
ssc_disable (void);


void
ssc_config (const ssc_cfg_t *);


/** Configure clocks
   Applies only the clock configuration options (only changes the CMR register)
   @param configuration, either for tx or rx module
*/
void
ssc_clock_cfg (ssc_module_cfg_t *);



/** Configure frame
   Applies only the frame configuration options (only changes the FMR register)
   @param configuration for either the tx or rx module
*/
void
ssc_frame_cfg (ssc_module_cfg_t *);


/** Set the clock divider
   @param 12-bit clock divider. 0 = internal ssc clock disabled, 1-4095 the
   internal ssc clock is enabled with a frequency of (MCK/2*div), where div is
   the divider supplied here
 */
void
ssc_set_clock_div (ssc_clock_div_t);


/** Enable either the tx or the rx module of the SSC
   @param the module to enable
 */
void 
ssc_enable_module (ssc_module_t);


/** Disable either the tx or the rx module of the SSC
   @param the module to disable
 */
void 
ssc_disable_module (ssc_module_t);


/** Check weather the tx/rx buffer is ready for read/write TODO (test)
   @param module to check (SSC_TX or SSC_RX)
   @return true = ready, false = not ready
 */
bool
ssc_buffer_ready_p (ssc_module_t);


/** Read the data in the rx buffer TODO (test)
   @return the data read from the buffer
   @param boolean wait for buffer ready, true = wait, false = don't wait
 */
ssc_data_t
ssc_read (bool wait);


/** Write to the tx buffer TODO (test)
   @param the data to write
   @param boolean wait for ready, true = wait, false = don't wait
 */
void
ssc_write (ssc_data_t data, bool wait);

#endif //SSC_H


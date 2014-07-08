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

typedef uint32_t ssc_data_t;

typedef uint32_t ssc_clock_speed_t;

typedef uint16_t  ssc_fs_period_t;      // RF signal period

typedef uint8_t   ssc_delay_t;       // Delay between start & receive

typedef uint8_t   ssc_data_length_t; // Data word length

typedef uint8_t   ssc_fs_length_t;   // Framesync Length

typedef uint8_t   ssc_data_num_t;    // Data words per frame



/* Clock sampling 'inversion'.  */
typedef enum
{
    SSC_CLOCK_FALLING = 0x0,         // Data are sampled/shifted on clock falling edge
    SSC_CLOCK_RISING  = SSC_RCMR_CKI // Data are sampled/shifted on clock rising edge
} ssc_clock_edge_t;



/* Stop condition select.  */
typedef enum 
{
    //Neither of these are in the header files?
    SSC_STOP_WAIT       = (0x0 << 12),       // Wait for new start
    SSC_STOP_CONTINUOUS = (0x1 << 12)        // Continuously transfer
} ssc_stop_t;


typedef enum 
{
   SSC_TX, SSC_RX
} ssc_module_t;


/* Clock source.  */
typedef enum 
{
    SSC_CLOCK_INTERNAL, SSC_CLOCK_OTHER, SSC_CLOCK_PIN
} ssc_clock_select_t;


/* Clock mode.  */
typedef enum 
{
    SSC_CLOCK_INPUT, SSC_CLOCK_CONTINUOUS, SSC_CLOCK_TRANSFER
} ssc_clock_out_mode_t;


/* Clock gating - determines inversion of the ready signal.  */
typedef enum 
{
    SSC_CLOCK_GATE_NONE, SSC_CLOCK_GATE_RF_LOW, SSC_CLOCK_GATE_RF_HIGH
} ssc_clock_gate_mode_t;



/* Receive start conditions - what's the difference between level
   changes and edges?  */

typedef enum
{
    SSC_START_CONTINUOUS    = SSC_RCMR_START_CONTINUOUS,  // Continuously receive
    SSC_START_TX            = SSC_RCMR_START_TRANSMIT,    // Transmit/Receive start
    SSC_START_LOW           = SSC_RCMR_START_RF_LOW,      // Low level RF/TF sig
    SSC_START_HIGH          = SSC_RCMR_START_RF_HIGH,     // High level RF/TF sig
    SSC_START_FALLING       = SSC_RCMR_START_RF_FALLING,  // RF/TF Falling Edge
    SSC_START_RISING        = SSC_RCMR_START_RF_RISING,   // RF/TF Rising Edge
    SSC_START_LEVEL_CHANGE  = SSC_RCMR_START_RF_LEVEL,    // Any level change of RF/TF
    SSC_START_ANY_EDGE      = SSC_RCMR_START_RF_EDGE,     // Any edge of RF/TF
    SSC_START_COMPARE0      = SSC_RCMR_START_CMP_0        
} ssc_start_mode_t;



/* Frame sync output mode.  */
typedef enum 
{
    SSC_FS_INPUT      = SSC_RFMR_FSOS_NONE,      // RF/TF pin is input
    SSC_FS_NEGATIVE   = SSC_RFMR_FSOS_NEGATIVE,  // Negative pulse
    SSC_FS_POSITIVE   = SSC_RFMR_FSOS_POSITIVE,  // Positive pulse
    SSC_FS_LOW        = SSC_RFMR_FSOS_LOW,       // Driven low during transfer
    SSC_FS_HIGH       = SSC_RFMR_FSOS_HIGH,      // Driven high during transfer
    SSC_FS_TOGGLE     = SSC_RFMR_FSOS_TOGGLING   // Toggle to begin transfer
}  ssc_fs_mode_t;


/* Frame sync edge detect which generates interrupts.  */
typedef enum 
{
    SSC_FS_EDGE_POSITIVE, SSC_FS_EDGE_NEGATIVE
} ssc_fs_edge_t;


/* What happens when the frame sync signal happens.  */
typedef enum 
{
    SSC_FSDEN_DEFAULT = 0x0u,
    SSC_FSDEN_SHIFT = SSC_TFMR_FSDEN
} ssc_tx_fs_data_enable_t;


/* Configuration structure for the SSC.  */
typedef struct 
{
    /* Period between fs assertions (clocks).  */
    ssc_fs_period_t      fs_period;   
    /* Delay after start event before data reception/transmission (clocks).  */
    ssc_delay_t          start_delay;
    /* Length of the frame sync pulse when it is pulsed.  */
    ssc_fs_length_t      fs_length;
    /* Number of bits per word.  */
    ssc_data_length_t    data_length;
    /* Number of words per frame.  */
    ssc_data_num_t       data_num;
    ssc_clock_edge_t     clock_edge;
    ssc_stop_t           stop_mode;
    ssc_clock_select_t   clock_select;
    ssc_clock_out_mode_t clock_out_mode;
    ssc_clock_gate_mode_t clock_gate_mode;
    ssc_start_mode_t     start_mode;
    ssc_fs_mode_t        fs_mode;
    ssc_fs_edge_t        fs_edge;
    bool                 loop_mode;
    bool                 data_msb_first;
    bool                 td_default;
    ssc_tx_fs_data_enable_t sync_data_enable;
} ssc_module_cfg_t;


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


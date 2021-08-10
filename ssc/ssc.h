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

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"

typedef uint16_t ssc_clock_divisor_t;

typedef uint32_t ssc_data_t;

typedef uint32_t ssc_clock_speed_t;

typedef uint16_t ssc_fs_period_t;   // RF signal period

typedef uint8_t  ssc_delay_t;       // Delay between start & receive

typedef uint8_t  ssc_data_length_t; // Data word length

typedef uint8_t  ssc_fs_length_t;   // Framesync Length

typedef uint8_t  ssc_data_num_t;    // Data words per frame



/* Clock sampling 'inversion'.  */
typedef enum
{
    // Data are sampled/shifted on clock falling edge
    SSC_CLOCK_FALLING = 0,         
    // Data are sampled/shifted on clock rising edge
    SSC_CLOCK_RISING  = SSC_RCMR_CKI 
} ssc_clock_edge_t;



/* Stop condition select.  This only applies if a data transfer has
   started with a compare register 0 match.  */
typedef enum 
{
    // Wait for next compare register 0 match
    SSC_STOP_WAIT       = (0 << 12),       
    // Operate continuously until a compare register 1 match
    SSC_STOP_CONTINUOUS = (1 << 12)        
} ssc_stop_mode_t;


typedef enum 
{
    SSC_TX, SSC_RX
} ssc_module_t;


/* Clock source.  */
typedef enum 
{
    // Use internal clock
    SSC_CLOCK_INTERNAL = 0,
    // Use TK or RK of other module
    SSC_CLOCK_OTHER = SSC_RCMR_CKS_TK,
    // Use TK or RK
    SSC_CLOCK_PIN = SSC_RCMR_CKS_RK
} ssc_clock_select_t;


/* Clock mode.  */
typedef enum 
{
    // Configure TK or RK as input
    SSC_CLOCK_INPUT = SSC_RCMR_CKO_NONE,
    // Drive TK or RK continuously
    SSC_CLOCK_CONTINUOUS = SSC_RCMR_CKO_CONTINUOUS,
    // Drive TK or RK during transfer
    SSC_CLOCK_TRANSFER = SSC_RCMR_CKO_TRANSFER
} ssc_clock_out_mode_t;


/* Clock gating.  */
typedef enum 
{
    // Receive clock always enabled
    SSC_CLOCK_GATE_NONE = 0,
    // Receive enabled when RF low
    SSC_CLOCK_GATE_RF_LOW = 1 << 7,
    // Receive enabled when RF high
    SSC_CLOCK_GATE_RF_HIGH = 2 << 7
} ssc_clock_gate_mode_t;


/* Frame sync start mode.  */
typedef enum
{
    // Start continuous transfer; either by writing to TTHR for transmit or
    // enabling receiver for receive
    SSC_START_CONTINUOUS    = SSC_RCMR_START_CONTINUOUS,
    // Transmit/Receive start
    SSC_START_TRANSMIT      = SSC_RCMR_START_TRANSMIT,
    // Start one clock after falling edge of RF
    SSC_START_LOW           = SSC_RCMR_START_RF_LOW,
    // Start one clock after rising edge of RF
    SSC_START_HIGH          = SSC_RCMR_START_RF_HIGH, 
    // Start after falling edge of RF
    SSC_START_FALLING       = SSC_RCMR_START_RF_FALLING,
    // Start after rising edge of RF
    SSC_START_RISING        = SSC_RCMR_START_RF_RISING,  
    // Start one clock after next rising or falling edge of RF
    SSC_START_LEVEL_CHANGE  = SSC_RCMR_START_RF_LEVEL,
    // Start after next rising or falling edge of RF
    SSC_START_ANY_EDGE      = SSC_RCMR_START_RF_EDGE,
    // Start when compare register 0 matches sync data
    SSC_START_COMPARE0      = SSC_RCMR_START_CMP_0        
} ssc_start_mode_t;


/* Frame sync output mode.  */
typedef enum 
{
    // RF/TF pin is input
    SSC_FS_INPUT      = SSC_RFMR_FSOS_NONE,    
    // Output negative pulse before data transfer
    SSC_FS_NEGATIVE   = SSC_RFMR_FSOS_NEGATIVE,
    // Output positive pulse before data transfer
    SSC_FS_POSITIVE   = SSC_RFMR_FSOS_POSITIVE,
    // Drive RF/TF low when first transfer starts (and it appears keep it low)
    SSC_FS_LOW        = SSC_RFMR_FSOS_LOW,
    // Drive RF/TF high when first transfer starts (and it appears keep it high)
    SSC_FS_HIGH       = SSC_RFMR_FSOS_HIGH,
    // Toggle RF/TF at start of each transfer
    SSC_FS_TOGGLE     = SSC_RFMR_FSOS_TOGGLING
}  ssc_fs_mode_t;


/* Frame sync edge which generates interrupts.  */
typedef enum 
{
    SSC_FS_EDGE_POSITIVE = 0,
    SSC_FS_EDGE_NEGATIVE = SSC_RFMR_FSEDGE
} ssc_fs_edge_t;


/* Frame sync data enable.  */
typedef enum 
{
    // The default value is output on TD during the frame sync period
    SSC_FSDEN_DEFAULT = 0,
    // The value in TSHR is output on TD during the frame sync period
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
    ssc_clock_select_t   clock_select;
    ssc_clock_out_mode_t clock_out_mode;
    ssc_clock_gate_mode_t clock_gate_mode;
    ssc_start_mode_t     start_mode;
    ssc_stop_mode_t      stop_mode;
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
} ssc_dev_t;


typedef ssc_dev_t *ssc_t;


/* SSC configuration structure, allows tx and rx modules to be independently
   configured.  */
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


/** Enable all the modules.   */
void
ssc_enable (ssc_t ssc);


/** Disable all the modules.   */
void
ssc_disable (ssc_t ssc);


bool
ssc_read_ready_p (ssc_t ssc);


bool
ssc_write_ready_p (ssc_t ssc);


/** Read the data in the rx buffer TODO (test)
   @return the data read from the buffer
   @param boolean wait for buffer ready, true = wait, false = don't wait
 */
uint32_t
ssc_read (ssc_t ssc, void *buffer, uint32_t length);


/** Write to the tx buffer TODO (test)
    @param the data to write
    @param boolean wait for ready, true = wait, false = don't wait
 */
uint16_t
ssc_write (ssc_t ssc, void *buffer, uint16_t length);


void
ssc_fs_period_set (ssc_t ssc, ssc_fs_period_t fs_period, ssc_module_t module);


ssc_fs_period_t 
ssc_fs_period_get (ssc_t ssc, ssc_module_t module);


Pdc *
ssc_pdc_get (ssc_t ssc);


/** Initialize/configure the SSC. You still need to enable the
    appropriate modules.  If a module is to be disabled then the
    pointer to its configuration from within the ssc config should be
    null. 
    @param pointer to the ssc configuration structure to apply
*/
ssc_t 
ssc_init (const ssc_cfg_t *);


void
ssc_shutdown (ssc_t ssc);


#ifdef __cplusplus
}
#endif    
#endif //SSC_H



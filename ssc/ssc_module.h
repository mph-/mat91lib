/**@file       ssc_module.h
 * @author     Stuart Duncan
 * @date       11 January 2012
 * @version    1.0
 * @brief      configuration definitions for ssc transmit and receive modules
 *
 * This is here rather than in ssc.h because there are so many options and it
 * made writing the code easier.
 */

#ifndef SSC_MODULE_H
#define SSC_MODULE_H

/* Length and other primitive typedefs.  */
typedef uint16_t  ssc_period_t;    // RF signal period
typedef uint8_t   ssc_delay_t;     // Delay between start & receive
typedef uint8_t   ssc_datalen_t;   // Data word length
typedef uint8_t   ssc_fslen_t;     // Framesync Length
typedef uint8_t   ssc_datanum_t;   // Data words per frame
typedef uint32_t  ssc_data_t;      // SSC data type - up to 32 bits per word



/* Clock sampling 'inversion'.  */
typedef enum
{
    SSC_CLKS_FALLING = 0x0,          // Data are sampled/shifted on clock falling edge
    SSC_CLKS_RISING  = SSC_RCMR_CKI // Data are sampled/shifted on clock rising edge
} ssc_clock_edge_sample_t;



/* Stop condition select.  */
typedef enum 
{
    //Neither of these are in the header files?
    SSC_STOP_WAIT       = (0x0 << 12),       // Wait for new start
    SSC_STOP_CONTINUOUS = (0x1 << 12)        // Continuously transfer
} ssc_stop_t;


/* Flag for weather the config is for the rx or tx module.  */
typedef enum {
   SSC_TX,SSC_RX
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
    SSC_FSOS_NONE       = SSC_RFMR_FSOS_NONE,      // RF/TF pin is input
    SSC_FSOS_NEGATIVE   = SSC_RFMR_FSOS_NEGATIVE,  // Negative pulse
    SSC_FSOS_POSITIVE   = SSC_RFMR_FSOS_POSITIVE,  // Positive pulse
    SSC_FSOS_DRIVE_LOW  = SSC_RFMR_FSOS_LOW,       // Driven low during transfer
    SSC_FSOS_DRIVE_HIGH = SSC_RFMR_FSOS_HIGH,      // Driven high during transfer
    SSC_FSOS_TOGGLE     = SSC_RFMR_FSOS_TOGGLING   // Toggle to begin transfer
}  ssc_fsos_mode_t;


/* Frame sync edge detect which generates interrupts.  */
typedef enum 
{
    SSC_FSEDGE_POSITIVE  = 0x0u,
    SSC_FSEDGE_NEGATIVE  = SSC_RFMR_FSEDGE
} ssc_fsedge_t;


/* What happens when the frame sync signal happens.  */
typedef enum 
{
    SSC_FSDEN_DEFAULT = 0x0u,
    SSC_FSDEN_SHIFT = SSC_TFMR_FSDEN
} ssc_tx_fs_data_enable_t;


/* Configuration structure for the SSC.  */
typedef struct 
{
   ssc_period_t         period;   
   ssc_delay_t          delay;
   ssc_datalen_t        word_size;
   ssc_fslen_t          fslen;
   ssc_datanum_t        words_per_frame;
   ssc_clock_edge_sample_t clock_sampling_edge;
   ssc_stop_t           stop_mode;
   ssc_module_t         tx_or_rx;
   ssc_clock_select_t   clock_select;
   ssc_clock_out_mode_t clock_out_mode;
   ssc_clock_gate_mode_t clock_gate_mode;
   ssc_start_mode_t     start_mode;
   ssc_fsos_mode_t      fsos_mode;
   ssc_fsedge_t         fsedge;
   bool                 loop_mode;
   bool                 msb_first;
   bool                 td_default;
   ssc_tx_fs_data_enable_t sync_data_enable;
} ssc_module_cfg_t;


#endif //SSC_MODULE_H

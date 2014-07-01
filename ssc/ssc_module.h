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
} ssc_clk_edge_sample_t;



/* Stop condition select.  */
typedef enum 
{
    //Neither of these are in the header files?
    SSC_STOP_WAIT       = (0x0 << 12),       // Wait for new start
    SSC_STOP_CONTINUOUS = (0x1 << 12)        // Continuously transfer
} ssc_stop_t;



/* Loopback enable 

   - only configurable via the rx module, requires both modules to be
     configured & enabled

   - Data sent via tx will appear in the rx data register
 */
typedef enum 
{
    SSC_LOOP_OFF = 0x0,          // Normal operation
    SSC_LOOP_ON  = SSC_RFMR_LOOP // RD=TD, RF = TF, RK = TK
} ssc_loop_t;



/* MSB first or last.  */
typedef enum 
{
    SSC_DATA_MSB_LAST    = 0x0,           // Least significant bit sampled first
    SSC_DATA_MSB_FIRST   = SSC_RFMR_MSBF  // Most significant bit sampled first
} ssc_msb_t;



/* Flag for weather the config is for the rx or tx module.  */
typedef enum {
   SSC_TX,SSC_RX
} ssc_module_t;



/* Clock source.  */
typedef enum 
{
    SSC_CS_DIVIDED = SSC_RCMR_CKS_MCK, // Divided clock
    SSC_CS_INT     = SSC_RCMR_CKS_TK,  // Internal clock (TX clock when configuring the RX module, and vice versa)
    SSC_CS_EXT     = SSC_RCMR_CKS_RK   // Clock on external pin (TK and RK pins for transmitter and receiver modules respectively)
} ssc_clock_select_t;



/* Clock mode - continous or not.  */
typedef enum 
{
    SSC_CM_INPUT        = SSC_RCMR_CKO_NONE,       // External clock - no clock control
    SSC_CM_CONTINUOUS   = SSC_RCMR_CKO_CONTINUOUS, // Continous clock output
    SSC_CM_TRANSFER     = SSC_RCMR_CKO_TRANSFER    // Clock output only for data transfers
} ssc_clock_out_mode_t;



/* Clock gating - determines inversion of the ready signal.  */
typedef enum 
{
    //Not in the header file either?
    SSC_CG_INPUT        = (0x0 << 7),     // Continuous clock
    SSC_CG_RFL          = (0x1 << 7),     // Clock when ready signal low
    SSC_CG_RFH          = (0x2 << 7)      // Clock when ready signal high
} ssc_clk_gate_mode_t;



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
} ssc_fsedge_mode_t;


/* Default level of the TD pin when not transmitting.  */
typedef enum 
{
    SSC_DATADEF_HIGH = SSC_TFMR_DATDEF,
    SSC_DATADEF_LOW = 0x0u
} ssc_tx_data_default_t;


/* what happens when the frame sync signal happens.  */
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
   ssc_clk_edge_sample_t clock_sampling_edge;
   ssc_stop_t           stop_mode;
   ssc_loop_t           loop;
   ssc_msb_t            msb;
   ssc_module_t         tx_or_rx;
   ssc_clock_select_t   clock_select;
   ssc_clock_out_mode_t clock_out_mode;
   ssc_clk_gate_mode_t  clock_gate_mode;
   ssc_start_mode_t     start_mode;
   ssc_fsos_mode_t      fsos_mode;
   ssc_fsedge_mode_t    fsedge_mode;
   ssc_tx_data_default_t td_default;
   ssc_tx_fs_data_enable_t sync_data_enable;
} ssc_module_cfg_t;


#endif //SSC_MODULE_H

/** @file   timer.h
    @author 
    @date   12 February 2008
    @brief  Timer counter routines for AT91SAM7 processors
*/

/* DOES NOT SUPPORT WAVEFORM MODE and does not include burst mode, clock invert or external clock configuration */

#ifndef TIMER_H
#define TIMER_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"

#define TIMER_CHANNEL_0     0
#define TIMER_CHANNEL_1     1
#define TIMER_CHANNEL_2     2

#define EX_TRG_NONE         0
#define EX_TRG_RISING       1
#define EX_TRG_FALLING      2
#define EX_TRG_EACH         3

#define EX_TRG_TIOB         0
#define EX_TRG_TIOA         1

#define RC_COMP_NONE        0
#define RC_COMP_TRG         1

#define LDRA_TIOA_NONE      0
#define LDRA_TIOA_RISING    1    
#define LDRA_TIOA_FALLING   2
#define LDRA_TIOA_EACH      3

#define LDRB_TIOA_NONE      0
#define LDRB_TIOA_RISING    1    
#define LDRB_TIOA_FALLING   2
#define LDRB_TIOA_EACH      3

#define LDRB_CLK_NONE       0
#define LDRB_CLK_STOP       1
#define LDRB_CLK_DIS        2
#define LDRB_CLK_STOP_DIS   3

#define TIMER_TC_CV         0
#define TIMER_TC_RA         1
#define TIMER_TC_RB         2
#define TIMER_TC_RC         3



/* Initialises the timer for capture mode */
uint8_t
timer_init (uint8_t channel);


/* Selects and applies a suitable prescale for the clock, based on the given period */
/* The period is in (master) clocks.                                                */
uint8_t
timer_prescaler_set (uint8_t channel, unsigned int period);



/* Set the timer to run in capture mode */
uint8_t
timer_capture_mode_set (uint8_t channel);


/*     Timer channel external trigger configuration (in capture mode only)      */
/*                                                                              */
/*                          --------OPTIONS--------                             */
/* ex_trg_edge:     EX_TRG_NONE     - No external trigger                       */
/*                  EX_TRG_RISING   - Trigger on signal rising edge             */ 
/*                  EX_TRG_FALLING  - Trigger on signal falling edge            */
/*                  EX_TRG_EACH     - Trigger on both rising and falling edges  */
/* ex_trg:          EX_TRG_TIOB     - Set external trigger to TIOB              */
/*                  EX_TRG_TIOA     - Set external trigger to TIOA              */
/* rc_comp_trg:     RC_COMP_NONE    - RC compare has no effect                  */
/*                  RC_COMP_TRG     - Trigger on RC compare                     */ 

uint8_t timer_capture_trigger_config (uint8_t channel, uint8_t ex_trg_edge, uint8_t ex_trg, uint8_t rc_comp_trg);



/*       Timer channel register loading configuration (in capture mode only)        */
/*                                                                                  */
/*                          --------OPTIONS--------                                 */
/* ldra:        LDRA_TIOA_NONE      - never load RA                                 */
/*              LDRA_TIOA_RISING    - load RA on TIOA signal rising edge            */
/*              LDRA_TIOA_FALLING   - load RA on TIOA signal falling edge           */
/*              LDRA_TIOA_EACH      - load RA on TIOA rising and falling edges      */
/* ldrb:        LDRB_TIOA_NONE      - never load RB                                 */
/*              LDRB_TIOA_RISING    - load RB on TIOA signal rising edge            */
/*              LDRB_TIOA_FALLING   - load RB on TIOA signal falling edge           */
/*              LDRB_TIOA_EACH      - load RB on TIOA rising and falling edges      */
/* ldrb_clk:    LDRB_CLK_STOP       - stop the clock when RB is loaded              */
/*              LDRB_CLK_DIS        - disable the clock when RB is loaded           */
/*              LDRB_CLK_STOP_DIS   - stop and disable the clock when RB is loaded  */
/*              LDRB_CLK_NONE       - clock continues as normal on RB loading       */ 

/* RA is loaded only if it has not been loaded since the last trigger or if RB has been loaded since    */
/* the last loading of RA.                                                                              */
/* RB is loaded only if RA has been loaded since the last trigger or the last loading of RB.            */
/* Loading RA or RB before the read of the last value loaded sets the Overrun Error Flag (LOVRS)        */
/* in TC_SR (Status Register). In this case, the old value is overwritten.                              */


uint8_t timer_capture_register_load_config (uint8_t channel, uint8_t ldra, uint8_t ldrb, uint8_t ldrb_clk);


/* Enables the timer clock */
uint8_t
timer_clock_enable (uint8_t channel);


/*                Timer channel register reading                */
/*                                                              */
/*                  --------OPTIONS--------                     */
/* timer_register:  TIMER_TC_CV       - Current counter value   */
/*                  TIMER_TC_RA       - Current RA value        */ 
/*                  TIMER_TC_RB       - Current RB value        */
/*                  TIMER_TC_RC       - Current RC value        */

uint16_t
timer_read (uint8_t channel, uint8_t timer_reg);

/* Performs a software trigger */
uint8_t
timer_reset (uint8_t channel);


/* Disables the timer */
uint8_t
timer_disable (uint8_t channel);



#ifdef __cplusplus
}
#endif    
#endif





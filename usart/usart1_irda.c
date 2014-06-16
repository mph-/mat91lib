/** @file   usart1_irda.c
    @author M. P. Hayes, UCECE
    @date   17 June 2007
    @brief 
*/

#include "config.h"

void
usart1_irda_init (void)
{
    uint16_t filter_period;
    AT91S_USART *pUSART = USART1;

    /* Reset and disable receiver and transmitter.  */
    pUSART->US_CR = AT91C_US_RSTRX | AT91C_US_RSTTX          
        | AT91C_US_RXDIS | AT91C_US_TXDIS;           

    /* Set IRDA mode, clock = MCK, 8-bit data, no parity, 1 stop bit.  */
    pUSART->US_MR = AT91C_US_USMODE_IRDA
        | AT91C_US_CLKS_CLOCK | AT91C_US_CHRL_8_BITS
        | AT91C_US_PAR_NONE | AT91C_US_NBSTOP_1_BIT;
    
    /* Set filter counter for demodulator.  This is an 8-bit counter.
       The datasheet is mighty vague.  

       IRDA uses RZI where a logic 0 is represented by a high pulse
       with a duration 3/16 of a bit period.  A logic 1 is represented
       by a low level.  The idea is to reduce the time that the LED is
       on.  If NRZ was used then the LED would be on continuously
       during frames.  On reception the waveform is inverted.  Thus
       the RXD signal is mostly low high with low going pulses to
       indicate a logic 0.  When a low transition is detected on RXD
       the filter counter is decremented.  If it reaches zero then a
       logic 0 has been found.  When a high transition is detected on
       RXD, the filter counter is reloaded from the filter register.
       Thus short low going pulses (say due to IR interference) will
       be rejected.

       Since bit_period = MCK / (16 * baud_divisor), if we want
       filter_period to be bit_period / 16 then filter_period needs to
       be equal to baud_divisor.  The baud_divisor is 16-bit but the
       filter counter is only 8-bit.  */

    filter_period = USART1->US_BRGR;
    if (filter_period > 255)
        filter_period = 255;

    pUSART->US_IF = filter_period;

    /* This is the default value.  It needs to be non-zero.  */
    pUSART->US_FIDI = 372;

    /* Enable receiver and transmitter.  */
    pUSART->US_CR = AT91C_US_RXEN | AT91C_US_TXEN; 
}


void
usart1_irda_transmit (void)
{
    AT91S_USART *pUSART = USART1;

    pUSART->US_CR = AT91C_US_RXDIS | AT91C_US_TXEN;
}


void
usart1_irda_receive (void)
{
    AT91S_USART *pUSART = USART1;

    pUSART->US_CR = AT91C_US_RXEN | AT91C_US_TXDIS;
}

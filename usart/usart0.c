/** @file   usart0.c
    @author Michael Hayes
    @date   10 March 2005
    @brief  Routines for interfacing with the usart0 on an AT91 ARM
*/


#include "usart0.h"
#include "usart0_defs.h"


void
usart0_baud_divisor_set (uint16_t baud_divisor)
{
    USART0_BAUD_DIVISOR_SET (baud_divisor);
}


uint8_t
usart0_init (uint16_t baud_divisor)
{
    AT91S_USART *pUSART = AT91C_BASE_US0;

    /* Disable interrupts.  */
    pUSART->US_IDR = ~0;

    /* Enable RxD0 and TxD0 pins.  */
    *AT91C_PIOA_PDR = AT91C_PA5_RXD0 | AT91C_PA6_TXD0;

    /* Disable pullups.  */
    *AT91C_PIOA_PPUDR = AT91C_PA5_RXD0 | AT91C_PA6_TXD0;

    /* Enable USART0 clock.  */
    AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_US0);
    
    /* Reset and disable receiver and transmitter.  */
    pUSART->US_CR = AT91C_US_RSTRX | AT91C_US_RSTTX          
        | AT91C_US_RXDIS | AT91C_US_TXDIS;           

    /* Set normal mode, clock = MCK, 8-bit data, no parity, 1 stop bit.  */
    pUSART->US_MR = AT91C_US_USMODE_NORMAL
        | AT91C_US_CLKS_CLOCK | AT91C_US_CHRL_8_BITS
        | AT91C_US_PAR_NONE | AT91C_US_NBSTOP_1_BIT;

    usart0_baud_divisor_set (baud_divisor);

    /* Enable receiver and transmitter.  */
    pUSART->US_CR = AT91C_US_RXEN | AT91C_US_TXEN; 
    
    return 1;
}


void
usart0_shutdown (void)
{
    AT91S_USART *pUSART = AT91C_BASE_US0;

    /* Disable RxD0 and TxD0 pins.  */
    *AT91C_PIOA_PER = AT91C_PA5_RXD0 | AT91C_PA6_TXD0;

    /* Disable pullups.  */
    *AT91C_PIOA_PPUDR = AT91C_PA5_RXD0 | AT91C_PA6_TXD0;

    /* Disable USART0 clock.  */
    AT91C_BASE_PMC->PMC_PCDR = BIT (AT91C_ID_US0);
    
    /* Reset and disable receiver and transmitter.  */
    pUSART->US_CR = AT91C_US_RSTRX | AT91C_US_RSTTX          
        | AT91C_US_RXDIS | AT91C_US_TXDIS;           
}


/* Return non-zero if there is a character ready to be read.  */
bool
usart0_read_ready_p (void)
{
#if HOSTED
    return 1;
#else
    return USART0_READ_READY_P ();
#endif
}


/* Return non-zero if a character can be written without blocking.  */
bool
usart0_write_ready_p (void)
{
    return USART0_WRITE_READY_P ();
}


/* Return non-zero if transmitter finished.  */
bool
usart0_write_finished_p (void)
{
    return USART0_WRITE_FINISHED_P ();
}


/* Write character to USART0.  */
int8_t 
usart0_putc (char ch)
{
    if (ch == '\n')
        usart0_putc ('\r');

    USART0_PUTC (ch);
    return ch;
}


/* Read character from USART0.  This blocks until a character is read.  */
int8_t
usart0_getc (void)
{
    /* Wait for something in receive buffer.  */
    while (!USART0_READ_READY_P ())
        continue;

    return USART0_READ ();
}


/* Write string to USART0.  This blocks until the string is written.  */
void 
usart0_puts (const char *str)
{
    while (*str)
        usart0_putc (*str++);
}


/** @file   usart1.h
    @author Michael Hayes
    @date   10 March 2005
    @brief  Routines for interfacing with the usart1 on an AT91 ARM
*/


#include "usart1.h"
#include "usart1_defs.h"


void
usart1_baud_divisor_set (uint16_t baud_divisor)
{
    USART1_BAUD_DIVISOR_SET (baud_divisor);
}


uint8_t usart1_init (uint16_t baud_divisor)
{
    AT91S_USART *pUSART = USART1;

    /* Disable interrupts.  */
    pUSART->US_IDR = ~0;

    /* Enable RxD1 and TxD1 pins.  */
    *AT91C_PIOA_PDR = AT91C_PA21_RXD1 | AT91C_PA22_TXD1;

    /* Disable pullups.  */
    *AT91C_PIOA_PPUDR = AT91C_PA21_RXD1 | AT91C_PA22_TXD1;

    /* Enable USART1 clock.  */
    PMC->PMC_PCER = BIT (AT91C_ID_USART1);
    
    /* Reset and disable receiver and transmitter.  */
    pUSART->US_CR = AT91C_US_RSTRX | AT91C_US_RSTTX          
        | AT91C_US_RXDIS | AT91C_US_TXDIS;           

    /* Set normal mode, clock = MCK, 8-bit data, no parity, 1 stop bit.  */
    pUSART->US_MR = AT91C_US_USMODE_NORMAL
        | AT91C_US_CLKS_CLOCK | AT91C_US_CHRL_8_BITS
        | AT91C_US_PAR_NONE | AT91C_US_NBSTOP_1_BIT;

    usart1_baud_divisor_set (baud_divisor);

    /* Enable receiver and transmitter.  */
    pUSART->US_CR = AT91C_US_RXEN | AT91C_US_TXEN; 
    
    return 1;
}


void
usart1_shutdown (void)
{
    AT91S_USART *pUSART = USART1;

    /* Disable RxD1 and TxD1 pins.  */
    *AT91C_PIOA_PER = AT91C_PA21_RXD1 | AT91C_PA22_TXD1;

    /* Disable USART1 clock.  */
    PMC->PMC_PCDR = BIT (AT91C_ID_USART1);
    
    /* Reset and disable receiver and transmitter.  */
    pUSART->US_CR = AT91C_US_RSTRX | AT91C_US_RSTTX          
        | AT91C_US_RXDIS | AT91C_US_TXDIS;           
}


/* Return non-zero if there is a character ready to be read.  */
bool
usart1_read_ready_p (void)
{
#if HOSTED
    return 1;
#else
    return USART1_READ_READY_P ();
#endif
}


/* Return non-zero if a character can be written without blocking.  */
bool
usart1_write_ready_p (void)
{
    return USART1_WRITE_READY_P ();
}


/* Return non-zero if transmitter finished.  */
bool
usart1_write_finished_p (void)
{
    return USART1_WRITE_FINISHED_P ();
}


/* Write character to USART1.  */
int8_t 
usart1_putc (char ch)
{
    if (ch == '\n')
        usart1_putc ('\r');

    USART1_PUTC (ch);
    return ch;
}


/* Read character from USART1.  This blocks until a character is read.  */
int8_t
usart1_getc (void)
{
    /* Wait for something in receive buffer.  */
    while (!USART1_READ_READY_P ())
        continue;

    return USART1_READ ();
}


/* Write string to USART1.  This blocks until the string is written.  */
void 
usart1_puts (const char *str)
{
    while (*str)
        usart1_putc (*str++);
}


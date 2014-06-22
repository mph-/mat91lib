/** @file   usart0.c
    @author Michael Hayes
    @date   10 March 2005
    @brief  Routines for interfacing with the usart0 on an AT91 ARM
*/

#include "mcu.h"
#include "pio.h"
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
    /* Disable interrupts.  */
    USART0->US_IDR = ~0;

    /* Enable RxD0 and TxD0 pins and disable pullups.  */
    pio_config_set (PA5_PIO, PIO_PERIPH_A);
    pio_config_set (PA6_PIO, PIO_PERIPH_A);

    /* Enable USART0 clock.  */
    mcu_pmc_enable (ID_USART0);
    
    /* Reset and disable receiver and transmitter.  */
    USART0->US_CR = US_CR_RSTRX | US_CR_RSTTX          
        | US_CR_RXDIS | US_CR_TXDIS;           

    /* Set normal mode, clock = MCK, 8-bit data, no parity, 1 stop bit.  */
    USART0->US_MR = US_MR_USART_MODE_NORMAL
        | US_MR_CHRL_8_BIT | US_MR_PAR_NO | US_MR_NBSTOP_1_BIT;

    usart0_baud_divisor_set (baud_divisor);

    /* Enable receiver and transmitter.  */
    USART0->US_CR = US_CR_RXEN | US_CR_TXEN; 
    
    return 1;
}


void
usart0_shutdown (void)
{
    /* Disable RxD0 and TxD0 pins.  */
    pio_config_set (PA5_PIO, PIO_PULLUP);
    pio_config_set (PA6_PIO, PIO_OUTPUT_LOW);

    /* Disable USART0 clock.  */
    mcu_pmc_disable (ID_USART0);
    
    /* Reset and disable receiver and transmitter.  */
    USART0->US_CR = US_CR_RSTRX | US_CR_RSTTX          
        | US_CR_RXDIS | US_CR_TXDIS;           
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


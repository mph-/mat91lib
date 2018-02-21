/** @file   uart1.h
    @author Michael Hayes
    @date   27 May 2015
    @brief  Routines for interfacing with the uart1 on an AT91 ARM
*/

#include "mcu.h"
#include "pio.h"
#include "uart1.h"
#include "uart1_defs.h"


void
uart1_baud_divisor_set (uint16_t baud_divisor)
{
    UART1_BAUD_DIVISOR_SET (baud_divisor);
}


int
uart1_init (uint16_t baud_divisor)
{
    /* Disable interrupts.  */
    UART1->UART_IDR = ~0;

    /* Enable RxD1 and TxD1 pins and disable pullups.  */
    pio_config_set (URXD1_PIO, URXD1_PERIPH);
    pio_config_set (UTXD1_PIO, UTXD1_PERIPH);

    /* Enable UART1 clock.  */
    mcu_pmc_enable (ID_UART1);
    
    /* Reset and disable receiver and transmitter.  */
    UART1->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX          
        | UART_CR_RXDIS | UART_CR_TXDIS;           

    /* Set normal mode, clock = MCK, 8-bit data, no parity, 1 stop bit.  */
    UART1->UART_MR = UART_MR_CHMODE_NORMAL | UART_MR_PAR_NO;

    uart1_baud_divisor_set (baud_divisor);

    /* Enable receiver and transmitter.  */
    UART1->UART_CR = UART_CR_RXEN | UART_CR_TXEN; 
    
    return 1;
}


void
uart1_shutdown (void)
{
    /* Disable RxD1 and TxD1 pins.  */
    pio_config_set (URXD1_PIO, PIO_PULLUP);
    pio_config_set (UTXD1_PIO, PIO_OUTPUT_LOW);

    /* Disable UART1 clock.  */
    mcu_pmc_disable (ID_UART1);
    
    /* Reset and disable receiver and transmitter.  */
    UART1->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX          
        | UART_CR_RXDIS | UART_CR_TXDIS;           
}


/* Return non-zero if there is a character ready to be read.  */
bool
uart1_read_ready_p (void)
{
#if HOSTED
    return 1;
#else
    return UART1_READ_READY_P ();
#endif
}


/* Return non-zero if a character can be written without blocking.  */
bool
uart1_write_ready_p (void)
{
    return UART1_WRITE_READY_P ();
}


/* Return non-zero if transmitter finished.  */
bool
uart1_write_finished_p (void)
{
    return UART1_WRITE_FINISHED_P ();
}


/* Write character to UART1.  This blocks.  */
int
uart1_putc (char ch)
{
    if (ch == '\n')
        uart1_putc ('\r');

    UART1_PUTC (ch);
    return ch;
}


/* Read character from UART1.  This does not block.  */
int
uart1_getc (void)
{
    if (! UART1_READ_READY_P ())
    {
        errno = EAGAIN;
        return -1;
    }

    return UART1_READ ();
}


/* Write string to UART1.  This blocks until the string is written.  */
int
uart1_puts (const char *str)
{
    while (*str)
    {
        if (uart1_putc (*str++) < 0)
            return -1;
    }
    return 1;
}


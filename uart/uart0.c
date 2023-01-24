/** @file   uart0.c
    @author Michael Hayes
    @date   27 May 2015
    @brief  Routines for interfacing with the uart0 on an AT91 ARM
*/

#include "mcu.h"
#include "pio.h"
#include "uart0.h"
#include "uart0_defs.h"


void
uart0_baud_divisor_set (uint16_t baud_divisor)
{
    UART0_BAUD_DIVISOR_SET (baud_divisor);
}


int
uart0_init (uint16_t baud_divisor)
{
    /* Disable interrupts.  */
    UART0->UART_IDR = ~0;

    /* Enable RxD0 and TxD0 pins and disable pullups.  */
    pio_config_set (UTXD0_PIO, UTXD0_PERIPH);
    pio_config_set (URXD0_PIO, URXD0_PERIPH);

    /* Enable UART0 clock.  */
    mcu_pmc_enable (ID_UART0);

    /* Reset and disable receiver and transmitter.  */
    UART0->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
        | UART_CR_RXDIS | UART_CR_TXDIS;

    /* Set normal mode, clock = MCK, 8-bit data, no parity, 1 stop
       bit.  Note, the OVER bit is set to 0 so the baud rate
       calculation is further divided by 16.  The UCLCKS field is 0 so
       the MCK is used as the clock source.  */
    UART0->UART_MR = UART_MR_CHMODE_NORMAL | UART_MR_PAR_NO;

    uart0_baud_divisor_set (baud_divisor);

    /* Enable receiver and transmitter.  */
    UART0->UART_CR = UART_CR_RXEN | UART_CR_TXEN;

    return 1;
}


void
uart0_shutdown (void)
{
    /* Disable RxD0 and TxD0 pins.  */
    pio_config_set (UTXD0_PIO, PIO_PULLUP);
    pio_config_set (URXD0_PIO, PIO_OUTPUT_LOW);

    /* Disable UART0 clock.  */
    mcu_pmc_disable (ID_UART0);

    /* Reset and disable receiver and transmitter.  */
    UART0->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
        | UART_CR_RXDIS | UART_CR_TXDIS;
}


/* Return non-zero if there is a character ready to be read.  */
bool
uart0_read_ready_p (void)
{
#if HOSTED
    return 1;
#else
    return UART0_READ_READY_P ();
#endif
}


/* Return non-zero if a character can be written without blocking.  */
bool
uart0_write_ready_p (void)
{
    return UART0_WRITE_READY_P ();
}


/* Return non-zero if transmitter finished.  */
bool
uart0_write_finished_p (void)
{
    return UART0_WRITE_FINISHED_P ();
}


/* Write character to UART0.  This does not block.  */
int
uart0_putc (char ch)
{
    if (! UART0_WRITE_READY_P ())
    {
        errno = EAGAIN;
        return -1;
    }

    UART0_PUTC (ch);
    return ch;
}


/* Read character from UART0.  This does not block.  */
int
uart0_getc (void)
{
    if (! UART0_READ_READY_P ())
    {
        errno = EAGAIN;
        return -1;
    }

    return UART0_READ ();
}


/* Write string to UART0.  This blocks until the string is written.  */
int
uart0_puts (const char *str)
{
    while (*str)
    {
        if (uart0_putc (*str++) < 0)
            return -1;
    }
    return 1;
}

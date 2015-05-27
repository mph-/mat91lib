/** @file   uart.c
    @author M. P. Hayes, UCECE
    @date   21 June 2007
    @brief  Unbuffered UART implementation.  */

#include "uart.h"
#include "peripherals.h"

/* This needs updating to be more general and to provide
   support for synchronous operation. 

   Note the UART does not provide hardware handshaking using RTS/CTS
   signals; only the USART supports this. 
*/


/* A UART can be disabled in the target.h file, e.g., using
   #define UART0_ENABLE 0.  */

#ifndef UART0_ENABLE
#define UART0_ENABLE (UART_NUM >= 1)
#endif

#ifndef UART1_ENABLE
#define UART1_ENABLE (UART_NUM >= 2)
#endif


struct uart_dev_struct
{
    int (*putch) (char ch);
    int (*getch) (void);
    bool (*read_ready_p) (void);
    bool (*write_ready_p) (void);
    bool (*write_finished_p) (void);
};


/* Include machine dependent uart definitions.  */
#if UART0_ENABLE
#include "uart0.h"

static uart_dev_t uart0_dev = {uart0_putc, uart0_getc,
                               uart0_read_ready_p, uart0_write_ready_p,
                               uart0_write_finished_p};
#endif

#if UART1_ENABLE
#include "uart1.h"

static uart_dev_t uart1_dev = {uart1_putc, uart1_getc,
                               uart1_read_ready_p, uart1_write_ready_p,
                               uart1_write_finished_p};
#endif


uart_t 
uart_init (uint8_t channel,
           uint16_t baud_divisor)
{
    uart_dev_t *dev = 0;

#if UART0_ENABLE
    if (channel == 0)
    {
        uart0_init (baud_divisor);
        dev = &uart0_dev;
    }
#endif

#if UART1_ENABLE
    if (channel == 1)
    {
        uart1_init (baud_divisor);
        dev = &uart1_dev;
    }
#endif

    return dev;
}


/* Return non-zero if there is a character ready to be read.  */
bool
uart_read_ready_p (uart_t uart)
{
    uart_dev_t *dev = uart;

    return dev->read_ready_p ();
}


/* Return non-zero if a character can be written without blocking.  */
bool
uart_write_ready_p (uart_t uart)
{
    uart_dev_t *dev = uart;

    return dev->write_ready_p ();
}


/* Return non-zero if transmitter finished.  */
bool
uart_write_finished_p (uart_t uart)
{
    uart_dev_t *dev = uart;

    return dev->write_finished_p ();
}


/* Read character.  This blocks.  */
int
uart_getc (uart_t uart)
{
    uart_dev_t *dev = uart;

    return dev->getch ();
}


/* Write character.  This blocks.  */
int
uart_putc (uart_t uart, char ch)
{
    uart_dev_t *dev = uart;

    return dev->putch (ch);
}


/* Write string.  This blocks.  */
int
uart_puts (uart_t uart, const char *str)
{
    while (*str)
    {
        if (uart_putc (uart, *str++) < 0)
            return -1;
    }

    return 1;
}

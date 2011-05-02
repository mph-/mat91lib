/** @file   uart.c
    @author M. P. Hayes, UCECE
    @date   21 June 2007
    @brief  Unbuffered UART implementation (this is mostly a wrapper 
    for the USART implementation).  */

#include "uart.h"
#include "peripherals.h"


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
    int8_t (*putch) (char ch);
    int8_t (*getch) (void);
    bool (*read_ready_p) (void);
    bool (*write_ready_p) (void);
    bool (*write_finished_p) (void);
};


/* Include machine dependent uart definitions.  */
#if UART0_ENABLE
#include "usart0.h"

static uart_dev_t uart0_dev = {usart0_putc, usart0_getc,
                                 usart0_read_ready_p, usart0_write_ready_p,
                                 usart0_write_finished_p};
#endif

#if UART1_ENABLE
#include "uart1.h"

static uart_dev_t uart1_dev = {usart1_putc, usart1_getc,
                                 usart1_read_ready_p, usart1_write_ready_p,
                                 usart1_write_finished_p};
#endif


uart_t 
uart_init (uart_cfg_t *cfg)
{
    uart_dev_t *dev = 0;

#if UART0_ENABLE
    if (cfg->channel == 0)
    {
        usart0_init (USART0_BAUD_DIVISOR (cfg->baud));
        dev = &uart0_dev;
    }
#endif

#if UART1_ENABLE
    if (cfg->channel == 1)
    {
        usart1_init (USART1_BAUD_DIVISOR (cfg->baud));
        dev = &uart1_dev;
    }
#endif

    return dev;
}


/** Return non-zero if there is a character ready to be read.  */
bool
uart_read_ready_p (uart_t uart)
{
    uart_dev_t *dev = uart;

    return dev->read_ready_p ();
}


/** Return non-zero if a character can be written without blocking.  */
bool
uart_write_ready_p (uart_t uart)
{
    uart_dev_t *dev = uart;

    return dev->write_ready_p ();
}


/** Return non-zero if transmitter finished.  */
bool
uart_write_finished_p (uart_t uart)
{
    uart_dev_t *dev = uart;

    return dev->write_finished_p ();
}


/** Read character.  This blocks.  */
int8_t
uart_getc (uart_t uart)
{
    uart_dev_t *dev = uart;

    return dev->getch ();
}


/** Write character.  This blocks.  */
int8_t
uart_putc (uart_t uart, char ch)
{
    uart_dev_t *dev = uart;

    return dev->putch (ch);
}


/** Write string.  This blocks.  */
int8_t
uart_puts (uart_t uart, const char *str)
{
    while (*str)
        uart_putc (uart, *str++);

    return 1;
}

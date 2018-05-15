/** @file   uart.c
    @author M. P. Hayes, UCECE
    @date   21 June 2007
    @brief  Unbuffered UART implementation.  */

#include "uart.h"
#include "peripherals.h"

/* This needs updating to be more general and to provide
   support for synchronous operation. 

   Note the UART does not provide hardware handshaking using RTS/CTS
   signals; only the UART supports this. 
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
    uint32_t read_timeout_us;
    uint32_t write_timeout_us;    
};


/* Include machine dependent uart definitions.  */
#if UART0_ENABLE
#include "uart0.h"

static uart_dev_t uart0_dev = {uart0_putc, uart0_getc,
                               uart0_read_ready_p, uart0_write_ready_p,
                               uart0_write_finished_p, 0, 0};
#endif

#if UART1_ENABLE
#include "uart1.h"

static uart_dev_t uart1_dev = {uart1_putc, uart1_getc,
                               uart1_read_ready_p, uart1_write_ready_p,
                               uart1_write_finished_p, 0, 0};
#endif


uart_t 
uart_init (const uart_cfg_t *cfg)
{
    uart_dev_t *dev = 0;
    uint16_t baud_divisor;

    if (cfg->baud_rate == 0)
        baud_divisor = cfg->baud_divisor;
    else
        baud_divisor = UART_BAUD_DIVISOR (cfg->baud_rate);

#if UART0_ENABLE
    if (cfg->channel == 0)
    {
        uart0_init (baud_divisor);
        dev = &uart0_dev;
    }
#endif

#if UART1_ENABLE
    if (cfg->channel == 1)
    {
        uart1_init (baud_divisor);
        dev = &uart1_dev;
    }
#endif

    dev->read_timeout_us = cfg->read_timeout_us;
    dev->write_timeout_us = cfg->write_timeout_us;

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


/** Read size bytes.  */
static int16_t
uart_read_nonblock (uart_t uart, void *data, uint16_t size)
{
    uint16_t count = 0;
    char *buffer = data;
    uart_dev_t *dev = uart;    

    for (count = 0; count < size; count++)
    {
        if (! uart_read_ready_p (uart))
        {
            if (count == 0)
            {
                errno = EAGAIN;
                return -1;
            }
            return count;
        }
        *buffer++ = dev->getch ();
    }
    return size;
}


/** Write size bytes.  */
int16_t
uart_write_nonblock (uart_t uart, const void *data, uint16_t size)
{
    uint16_t count = 0;
    const char *buffer = data;
    uart_dev_t *dev = uart;    

    for (count = 0; count < size; count++)
    {
        if (! uart_write_ready_p (uart))
        {
            if (count == 0)
            {
                errno = EAGAIN;
                return -1;
            }
            return count;
        }
        dev->putch (*buffer++);
    }
    return size;
}


/** Read size bytes.  Block until all the bytes have been read or
    until timeout occurs.  */
ssize_t
uart_read (void *uart, void *data, size_t size)
{
    uart_dev_t *dev = uart;
    
    return sys_read_timeout (uart, data, size, dev->read_timeout_us,
                             (void *)uart_read_nonblock);
}


/** Write size bytes.  Block until all the bytes have been transferred
    to the transmit ring buffer or until timeout occurs.  */
ssize_t
uart_write (void *uart, const void *data, size_t size)
{
    uart_dev_t *dev = uart;
    
    return sys_write_timeout (uart, data, size, dev->write_timeout_us,
                              (void *)uart_write_nonblock);
}


/** Read character.  */
int
uart_getc (uart_t uart)
{
    int ret;
    char ch;
    
    ret = uart_read (uart, &ch, 1);
    if (ret == 1)
        return ch;
    return ret;
}


/** Write character.  */
int
uart_putc (uart_t uart, char ch)
{
    int ret;
    
    ret = uart_write (uart, &ch, 1);
    if (ret == 1)
        return ch;
    return ret;
}


/** Write string.  In non-blocking mode this is likely to 
    ignore all but the first character.  */
int
uart_puts (uart_t uart, const char *str)
{
    while (*str)
    {
        int ret;
        
        ret = uart_putc (uart, *str++);
        if (ret < 1)
            return ret;
    }
    return 1;
}

const sys_file_ops_t uart_file_ops =
{
    .read = (void *)uart_read,
    .write = (void *)uart_write
};

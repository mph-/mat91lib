/** @file   usart.c
    @author M. P. Hayes, UCECE
    @date   21 June 2007
    @brief  Unbuffered USART implementation. 
    @note   This driver is a wrapper for USARTx and provides a USART
   independent interface.

   This needs updating to be more general and to provide
   support for synchronous operation. 

   If code memory is at a premium, a USART can be disabled in the
   target.h file, e.g., using #define USART0_ENABLE 0. 

   Both USART0 and USART1 have CTS/RTS flow control pins:

   * RTS is an output from the receiver.  It is driven low when the receiver
   is ready to read.

   * CTS is an input to the transmitter.  The transmitter does not transmit
   until it goes low.

   By default hardware handshaking is not used for flow control.  This
   can be enabled by defining USART0_USE_HANDSHAKING or
   USART1_USE_HANDSHAKING in target.h  */

#include "usart.h"
#include "sys.h"
#include "peripherals.h"

#ifndef USART0_ENABLE
#define USART0_ENABLE (USART_NUM >= 1)
#endif

#ifndef USART1_ENABLE
#define USART1_ENABLE (USART_NUM >= 2)
#endif


struct usart_dev_struct
{
    int (*putch) (char ch);
    int (*getch) (void);
    bool (*read_ready_p) (void);
    bool (*write_ready_p) (void);
    bool (*write_finished_p) (void);
    uint32_t read_timeout_us;
    uint32_t write_timeout_us;
};


/* Include machine dependent usart definitions.  */
#if USART0_ENABLE
#include "usart0.h"

static usart_dev_t usart0_dev = {usart0_putc, usart0_getc,
                                 usart0_read_ready_p, usart0_write_ready_p,
                                 usart0_write_finished_p, 0, 0};
#endif

#if USART1_ENABLE
#include "usart1.h"

static usart_dev_t usart1_dev = {usart1_putc, usart1_getc,
                                 usart1_read_ready_p, usart1_write_ready_p,
                                 usart1_write_finished_p, 0, 0};
#endif


usart_t 
usart_init (const usart_cfg_t *cfg)
{
    usart_dev_t *dev = 0;
    uint16_t baud_divisor;

    if (cfg->baud_rate == 0)
        baud_divisor = cfg->baud_divisor;
    else
        baud_divisor = USART_BAUD_DIVISOR (cfg->baud_rate);

#if USART0_ENABLE
    if (cfg->channel == 0)
    {
        usart0_init (baud_divisor);
        dev = &usart0_dev;
    }
#endif

#if USART1_ENABLE
    if (cfg->channel == 1)
    {
        usart1_init (baud_divisor);
        dev = &usart1_dev;
    }
#endif

    dev->read_timeout_us = cfg->read_timeout_us;
    dev->write_timeout_us = cfg->write_timeout_us;
    return dev;
}


/** Return non-zero if there is a character ready to be read.  */
bool
usart_read_ready_p (usart_t usart)
{
    usart_dev_t *dev = usart;

    return dev->read_ready_p ();
}


/** Return non-zero if a character can be written without blocking.  */
bool
usart_write_ready_p (usart_t usart)
{
    usart_dev_t *dev = usart;

    return dev->write_ready_p ();
}


/** Return non-zero if transmitter finished.  */
bool
usart_write_finished_p (usart_t usart)
{
    usart_dev_t *dev = usart;

    return dev->write_finished_p ();
}


/** Read size bytes.  */
static int16_t
usart_read_nonblock (usart_t usart, void *data, uint16_t size)
{
    uint16_t count = 0;
    char *buffer = data;
    usart_dev_t *dev = usart;    

    for (count = 0; count < size; count++)
    {
        if (! usart_read_ready_p (usart))
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
usart_write_nonblock (usart_t usart, const void *data, uint16_t size)
{
    uint16_t count = 0;
    const char *buffer = data;
    usart_dev_t *dev = usart;    

    for (count = 0; count < size; count++)
    {
        if (! usart_write_ready_p (usart))
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
usart_read (void *usart, void *data, size_t size)
{
    usart_dev_t *dev = usart;
    
    return sys_read_timeout (usart, data, size, dev->read_timeout_us,
                             (void *)usart_read_nonblock);
}


/** Write size bytes.  Block until all the bytes have been transferred
    to the transmit ring buffer or until timeout occurs.  */
ssize_t
usart_write (void *usart, const void *data, size_t size)
{
    usart_dev_t *dev = usart;
    
    return sys_write_timeout (usart, data, size, dev->write_timeout_us,
                              (void *)usart_write_nonblock);
}


/** Read character.  */
int
usart_getc (usart_t usart)
{
    int ret;
    char ch;
    
    ret = usart_read (usart, &ch, 1);
    if (ret == 1)
        return ch;
    return ret;
}


/** Write character.  */
int
usart_putc (usart_t usart, char ch)
{
    int ret;
    
    ret = usart_write (usart, &ch, 1);
    if (ret == 1)
        return ch;
    return ret;
}


/** Write string.  In non-blocking mode this is likely to 
    ignore all but the first character.  */
int
usart_puts (usart_t usart, const char *str)
{
    while (*str)
    {
        int ret;
        
        ret = usart_putc (usart, *str++);
        if (ret < 1)
            return ret;
    }
    return 1;
}


const sys_file_ops_t usart_file_ops =
{
    .read = (void *)usart_read,
    .write = (void *)usart_write
};

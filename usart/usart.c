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

#include "errno.h"
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
    bool block;
};


/* Include machine dependent usart definitions.  */
#if USART0_ENABLE
#include "usart0.h"

static usart_dev_t usart0_dev = {usart0_putc, usart0_getc,
                                 usart0_read_ready_p, usart0_write_ready_p,
                                 usart0_write_finished_p, 0};
#endif

#if USART1_ENABLE
#include "usart1.h"

static usart_dev_t usart1_dev = {usart1_putc, usart1_getc,
                                 usart1_read_ready_p, usart1_write_ready_p,
                                 usart1_write_finished_p, 0};
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

    dev->block = cfg->block;

    return dev;
}


/* Return non-zero if there is a character ready to be read.  */
bool
usart_read_ready_p (usart_t usart)
{
    usart_dev_t *dev = usart;

    return dev->read_ready_p ();
}


/* Return non-zero if a character can be written without blocking.  */
bool
usart_write_ready_p (usart_t usart)
{
    usart_dev_t *dev = usart;

    return dev->write_ready_p ();
}


/* Return non-zero if transmitter finished.  */
bool
usart_write_finished_p (usart_t usart)
{
    usart_dev_t *dev = usart;

    return dev->write_finished_p ();
}


/* Read character.  */
int
usart_getc (usart_t usart)
{
    usart_dev_t *dev = usart;

    if (! dev->block && ! usart_read_ready_p (usart))
    {
        errno = EAGAIN;
        return - 1;
    }

    return dev->getch ();
}


/* Write character.  */
int
usart_putc (usart_t usart, char ch)
{
    usart_dev_t *dev = usart;

    if (! dev->block && ! usart_write_ready_p (usart))
    {
        errno = EAGAIN;
        return - 1;
    }

    return dev->putch (ch);
}


/* Write string.  */
int
usart_puts (usart_t usart, const char *str)
{
    while (*str)
    {
        if (usart_putc (usart, *str++) < 0)
            return -1;
    }

    return 1;
}


/** Read size bytes.  */
int16_t
usart_read (usart_t usart, void *data, uint16_t size)
{
    uint16_t count = 0;
    char *buffer = data;

    for (count = 0; count < size; count++)
    {
        int ch;

        ch = usart_getc (usart);
        if (ch < 0)
        {
            if (count == 0 && errno == EAGAIN)
                return -1;
            return count;
        }
        *buffer++ = ch;
    }
    return size;
}


/** Write size bytes.  */
int16_t
usart_write (usart_t usart, const void *data, uint16_t size)
{
    uint16_t count = 0;
    const char *buffer = data;

    for (count = 0; count < size; count++)
    {
        int ret;

        ret = usart_putc (usart, *buffer++);
        if (ret < 0)
        {
            if (count == 0 && errno == EAGAIN)
                return -1;
            return count;
        }
    }
    return size;
}


const sys_file_ops_t usart_file_ops =
{
    .read = (void *)usart_read,
    .write = (void *)usart_write
};



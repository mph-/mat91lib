/** @file   usart.c
    @author M. P. Hayes, UCECE
    @date   21 June 2007
    @brief  Unbuffered USART implementation.  */

#include "usart.h"
#include "peripherals.h"

/* This needs updating to be more general and to provide
   support for synchronous operation.  */

/* A USART can be disabled in the target.h file, e.g., using
   #define USART0_ENABLE 0.  */

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
};


/* Include machine dependent usart definitions.  */
#if USART0_ENABLE
#include "usart0.h"

static usart_dev_t usart0_dev = {usart0_putc, usart0_getc,
                                 usart0_read_ready_p, usart0_write_ready_p,
                                 usart0_write_finished_p};
#endif

#if USART1_ENABLE
#include "usart1.h"

static usart_dev_t usart1_dev = {usart1_putc, usart1_getc,
                                 usart1_read_ready_p, usart1_write_ready_p,
                                 usart1_write_finished_p};
#endif


usart_t 
usart_init (uint8_t channel,
            uint16_t baud_divisor)
{
    usart_dev_t *dev = 0;

#if USART0_ENABLE
    if (channel == 0)
    {
        usart0_init (baud_divisor);
        dev = &usart0_dev;
    }
#endif

#if USART1_ENABLE
    if (channel == 1)
    {
        usart1_init (baud_divisor);
        dev = &usart1_dev;
    }
#endif

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


/* Read character.  This blocks.  */
int
usart_getc (usart_t usart)
{
    usart_dev_t *dev = usart;

    return dev->getch ();
}


/* Write character.  This blocks.  */
int
usart_putc (usart_t usart, char ch)
{
    usart_dev_t *dev = usart;

    return dev->putch (ch);
}


/* Write string.  This blocks.  */
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

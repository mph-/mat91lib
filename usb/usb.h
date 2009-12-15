#ifndef USB_H
#define USB_H

#include "config.h"


typedef struct usb_struct
{
    AT91PS_UDP pUDP;
    uint32_t rx_bank;
    uint16_t rx_bytes;
    uint8_t connection;
    bool configured;
} usb_dev_t;

typedef usb_dev_t *usb_t;

typedef uint16_t usb_size_t;


extern usb_size_t
usb_write (usb_t usb, const void *buffer, usb_size_t length);


extern usb_size_t
usb_read (usb_t usb, void *buffer, usb_size_t length);


extern bool
usb_configured_p (usb_t dev);


extern void
usb_connect (usb_t dev);


extern void
usb_shutdown (void);


extern usb_t 
usb_init (void);


extern bool
usb_read_ready_p (usb_t usb);


/* Read character.  */
extern int8_t
usb_getc (usb_t usb);


/* Write character.  */
extern int8_t
usb_putc (usb_t usb, char ch);


/* Write string.  This blocks until the string is buffered.  */
extern int8_t
usb_puts (usb_t usb, const char *str);

#endif

#include "config.h"


typedef struct usb_struct
{
    AT91PS_UDP pUDP;
    unsigned char currentConfiguration;
    unsigned char currentConnection;
    unsigned int  currentRcvBank;
} usb_dev_t;

typedef usb_dev_t *usb_t;

typedef uint16_t usb_size_t;


extern usb_size_t
usb_write (usb_t usb, const char *data, usb_size_t length);

extern usb_size_t
usb_read (usb_t usb, char *data, usb_size_t length);

extern bool
usb_configured_p (usb_t dev);

extern usb_t 
usb_init (void);

#ifndef USB_H
#define USB_H

#include "config.h"

enum {AT91C_EP_OUT_SIZE = 64, AT91C_EP_IN_SIZE = 64};

#define USB_EP_OUT_SIZE AT91C_EP_OUT_SIZE
#define USB_EP_IN_SIZE AT91C_EP_IN_SIZE


typedef struct usb_dev_struct
{
    AT91PS_UDP pUDP;
    uint32_t rx_bank;
    uint16_t rx_bytes;
    uint8_t connection;
    bool configured;
    const struct usb_cfg_struct *cfg;
} usb_dev_t;


typedef bool (*usb_request_callback_t) (usb_dev_t *usb, 
                                        uint16_t request, uint16_t value, 
                                        uint16_t index, uint16_t length);

typedef struct usb_cfg_struct
{
    const char *cfg_descriptor;
    uint16_t cfg_descriptor_size;
    usb_request_callback_t request_callback;
} usb_cfg_t;


typedef usb_dev_t *usb_t;

typedef uint16_t usb_size_t;

void usb_control_write (usb_t usb, const void *data, usb_size_t length);

void usb_control_write_zlp (usb_t usb);

void usb_control_stall (usb_t usb);

bool usb_read_ready_p (usb_t usb);

usb_size_t usb_read (usb_t usb, void *buffer, usb_size_t length);

usb_size_t usb_write (usb_t usb, const void *buffer, usb_size_t length);

bool usb_configured_p (usb_t usb);

void usb_connect (usb_t usb);

usb_t usb_init (const usb_cfg_t *cfg);

#endif

#ifndef USB_CDC_H
#define USB_CDC_H

#include <config.h>
#include <ring.h>


typedef struct
{
    uint8_t state;
    ring_t tx_ring;
    ring_t rx_ring;
} usb_cdc_dev_t;


typedef usb_cdc_dev_t *usb_cdc_t;



extern usb_cdc_t
usb_cdc_init (char *tx_buffer, ring_size_t tx_size,
              char *rx_buffer, ring_size_t rx_size);


extern bool 
usb_cdc_configured_p (usb_cdc_t dev);


extern void
usb_cdc_connect (usb_cdc_t dev);


/* Write size bytes.  Currently this only writes as many bytes (up to
   the desired size) that can currently fit in the ring buffer.   */
extern ring_size_t
usb_cdc_write (usb_cdc_t usb_cdc, const void *data, ring_size_t size);

#endif

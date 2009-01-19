#include "usb_cdc.h"

#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>

/* The CDC (communication device class) serial driver sits on top
   of the USB driver and provides a virtual com port.  The USB
   driver performs reads using the endpoint interrupt handler.
   USBD_InterruptHandler handles all USB interrupts.
   
   Ths USB clock needs to be 48, 96, or 192 MHz (PLL clock divided
   by 1, 2, or 4).
   
   UDP (USB device port)
   
   Endpoint     Dual-Bank Max. Endpoint Size   Endpoint Type
   EP0          No              8              Control/Bulk/Interrupt
   EP1          Yes            64              Bulk/Iso/Interrupt
   EP2          Yes            64              Bulk/Iso/Interrupt
   EP3          No             64              Control/Bulk/Interrupt
   
   Need to enable UDP clock in PMC before any operations to the
   UDP registers.
   
*/


enum {STATE_IDLE = 0, STATE_SUSPEND = 4, STATE_RESUME = 5};

static usb_cdc_dev_t devices[1];


//------------------------------------------------------------------------------
/// Invoked when the USB device leaves the suspended state.
//------------------------------------------------------------------------------
void USBDCallbacks_Resumed(void)
{
    devices[0].state = STATE_RESUME;
}

//------------------------------------------------------------------------------
/// Invoked when the USB device gets suspended. 
//------------------------------------------------------------------------------
void USBDCallbacks_Suspended(void)
{
    devices[0].state = STATE_SUSPEND;
}


usb_cdc_t
usb_cdc_init (char *tx_buffer, ring_size_t tx_size,
              char *rx_buffer, ring_size_t rx_size)
{
    usb_cdc_t dev = devices;

    dev->state = STATE_IDLE;

    CDCDSerialDriver_Initialize ();

    ring_init (&dev->tx_ring, tx_buffer, tx_size);
    
    ring_init (&dev->rx_ring, rx_buffer, rx_size);


    return dev;
}


bool 
usb_cdc_connected_p (usb_cdc_t dev __UNUSED__)
{
    return USBD_GetState () >= USBD_STATE_CONFIGURED;
}


void
usb_cdc_connect (usb_cdc_t dev __UNUSED__)
{
    /* Connect pull-up, wait for configuration.  This does nothing if the
       pull-up is always connected.  */
    USBD_Connect ();
}



#if 0
        if( USBState == STATE_SUSPEND )
        {
            // Switch to low power mode
            USBState = STATE_IDLE;
        }
        if( USBState == STATE_RESUME ) 
        {
            // Switch to normal power mode
            USBState = STATE_IDLE;
        }
#endif



/** Write size bytes.  Currently this only writes as many bytes (up to
    the desired size) that can currently fit in the ring buffer.   */
ring_size_t
usb_cdc_write (usb_cdc_t usb_cdc, const void *data, ring_size_t size)
{
#if 0
    int ret;
    usb_cdc_dev_t *dev = usb_cdc;

    ret = ring_write (&dev->tx_ring, data, size);

    /* Here we should only need to kickstart an interrupt and have the ISR
       empty the ring buffer.  */

    return ret;
#else
    
    while (CDCDSerialDriver_Write ((void *)data, size, 0, 0) 
           != USBD_STATUS_SUCCESS)
        continue;
#endif

    return size;
}

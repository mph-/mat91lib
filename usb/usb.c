#include "config.h"
#include "usb.h"
#include "pio.h"


/* CDC communication device class. 
   UDP USB device port.

   There are two communication endpoints and endpoint 0 is used for
   the enumeration process.  Endpoint 1 is a 64-byte Bulk OUT endpoint
   and endpoint 2 is a 64-byte Bulk IN endpoint.

   USB is a master-slave bus with one master and multiple slaves. The
   slaves are called peripherals or functions and the master is called
   the host.  Only the host can initiate USB transfers.

   USB uses differential signalling except for bus reset and end of
   packet signals where D+ and D- are both low.

   When no device is connected, the USB D+ and D- signals are tied to
   GND by 15 K pull-down resistors integrated in the hub downstream
   ports.   The USB device is in the not powered state.

   When a device is attached to a hub downstream port, the device
   connects a 1.5 K pull-up resistor on D+. The USB bus line goes into
   IDLE state, D+ is pulled up by the device 1.5 K resistor to 3.3 V
   and D- is pulled down by the 15 K resistor of the host.  The USB
   device is in the attached state.

   The device then waits for an end of bus reset from the host (both
   D+ and D- pulled low for at least 10 ms) and then enters the
   default state.  If the device supports high speed it sends a chirp
   K and the host responds with a sequnce of alternating chirp K and
   chirp J (KJKJKJ).

   In the default state the control endpoint (0) is enabled and the
   device waits to be enumerated by the host.  When it receives a set
   address standard device request the device enters the address
   state.

   Once a valid set configuration standard request has been received
   and acknowledged, the device enables endpoints corresponding to the
   current configuration.  The device then enters the configured
   state.

   Note for a full speed device D+ is pulled up while for a low speed device
   D- is pulled up.

   This code should probably be split into a UDP hardware abstraction layer
   plus a generic USB CDC driver.

   The default Vendor ID is Atmel's vendor ID 0x03EB.  The default
   product ID is 0x6124.  With these lsusb gives
   ``Atmel Corp. at91sam SAMBA bootloader''

   Using sudo modprobe usbserial vendor=0x03EB product=0x6124
   will create a tty device such as /dev/ttyUSB0
*/

#ifndef USB_VENDOR_ID
#define USB_VENDOR_ID 0x03EB
#endif


#ifndef USB_PRODUCT_ID
#define USB_PRODUCT_ID 0x6124
#endif


#ifndef USB_RELEASE_ID
#define USB_RELEASE_ID 0x110
#endif


#ifndef USB_CURRENT_MA
#define USB_CURRENT_MA 100
#endif


#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif


#ifndef HIGH_BYTE
#define HIGH_BYTE(v) (((v) >> 8) & 0xff)
#endif


#ifndef LOW_BYTE
#define LOW_BYTE(v) ((v) & 0xff)
#endif


/* IN and OUT are referred to the host so we transmit on the IN endpoint
   and receive on the OUT endpoint.  */
enum {AT91C_EP_OUT = 1, AT91C_EP_IN = 2};
enum {AT91C_EP_OUT_SIZE = 64, AT91C_EP_IN_SIZE = 64};


static const char devDescriptor[] =
{
    /* Device descriptor */
    0x12,   // bLength
    0x01,   // bDescriptorType
    0x10,   // bcdUSBL
    0x01,   //
    0x02,   // bDeviceClass:    CDC class code
    0x00,   // bDeviceSubclass: CDC class sub code
    0x00,   // bDeviceProtocol: CDC Device protocol
    0x08,   // bMaxPacketSize0
    LOW_BYTE (USB_VENDOR_ID),   // idVendorL
    HIGH_BYTE (USB_VENDOR_ID),   //
    LOW_BYTE (USB_PRODUCT_ID),   // idProductL
    HIGH_BYTE (USB_PRODUCT_ID),   //
    LOW_BYTE (USB_RELEASE_ID),   // bcdDeviceL
    HIGH_BYTE (USB_RELEASE_ID),   //
    0x00,   // iManufacturer    // 0x01
    0x00,   // iProduct
    0x00,   // SerialNumber
    0x01    // bNumConfigs
};


static const char cfgDescriptor[] = 
{
    /* ============== CONFIGURATION 1 =========== */
    /* Configuration 1 descriptor */
    0x09,   // CbLength
    0x02,   // CbDescriptorType
    0x43,   // CwTotalLength 2 EP + Control
    0x00,
    0x02,   // CbNumInterfaces
    0x01,   // CbConfigurationValue
    0x00,   // CiConfiguration
    0xC0,   // CbmAttributes 0xA0
    USB_CURRENT_MA / 2,   // CMaxPower
    
    /* Communication Class Interface Descriptor Requirement */
    0x09, // bLength
    0x04, // bDescriptorType
    0x00, // bInterfaceNumber
    0x00, // bAlternateSetting
    0x01, // bNumEndpoints
    0x02, // bInterfaceClass
    0x02, // bInterfaceSubclass
    0x00, // bInterfaceProtocol
    0x00, // iInterface
    
    /* Header Functional Descriptor */
    0x05, // bFunction Length
    0x24, // bDescriptor type: CS_INTERFACE
    0x00, // bDescriptor subtype: Header Func Desc
    0x10, // bcdCDC:1.1
    0x01,
    
    /* ACM Functional Descriptor */
    0x04, // bFunctionLength
    0x24, // bDescriptor Type: CS_INTERFACE
    0x02, // bDescriptor Subtype: ACM Func Desc
    0x00, // bmCapabilities
    
    /* Union Functional Descriptor */
    0x05, // bFunctionLength
    0x24, // bDescriptorType: CS_INTERFACE
    0x06, // bDescriptor Subtype: Union Func Desc
    0x00, // bMasterInterface: Communication Class Interface
    0x01, // bSlaveInterface0: Data Class Interface
    
    /* Call Management Functional Descriptor */
    0x05, // bFunctionLength
    0x24, // bDescriptor Type: CS_INTERFACE
    0x01, // bDescriptor Subtype: Call Management Func Desc
    0x00, // bmCapabilities: D1 + D0
    0x01, // bDataInterface: Data Class Interface 1
    
    /* Endpoint 1 descriptor */
    0x07,   // bLength
    0x05,   // bDescriptorType
    0x83,   // bEndpointAddress, Endpoint 03 - IN
    0x03,   // bmAttributes      INT
    0x08,   // wMaxPacketSize
    0x00,
    0xFF,   // bInterval
    
    /* Data Class Interface Descriptor Requirement */
    0x09, // bLength
    0x04, // bDescriptorType
    0x01, // bInterfaceNumber
    0x00, // bAlternateSetting
    0x02, // bNumEndpoints
    0x0A, // bInterfaceClass
    0x00, // bInterfaceSubclass
    0x00, // bInterfaceProtocol
    0x00, // iInterface
    
    /* First alternate setting */
    /* Endpoint 1 descriptor */
    0x07,   // bLength
    0x05,   // bDescriptorType
    0x01,   // bEndpointAddress, Endpoint 01 - OUT
    0x02,   // bmAttributes      BULK
    AT91C_EP_OUT_SIZE,   // wMaxPacketSize
    0x00,
    0x00,   // bInterval
    
    /* Endpoint 2 descriptor */
    0x07,   // bLength
    0x05,   // bDescriptorType
    0x82,   // bEndpointAddress, Endpoint 02 - IN
    0x02,   // bmAttributes      BULK
    AT91C_EP_IN_SIZE,   // wMaxPacketSize
    0x00,
    0x00    // bInterval
};


/* USB standard request code */
#define STD_GET_STATUS_ZERO           0x0080
#define STD_GET_STATUS_INTERFACE      0x0081
#define STD_GET_STATUS_ENDPOINT       0x0082

#define STD_CLEAR_FEATURE_ZERO        0x0100
#define STD_CLEAR_FEATURE_INTERFACE   0x0101
#define STD_CLEAR_FEATURE_ENDPOINT    0x0102

#define STD_SET_FEATURE_ZERO          0x0300
#define STD_SET_FEATURE_INTERFACE     0x0301
#define STD_SET_FEATURE_ENDPOINT      0x0302

#define STD_SET_ADDRESS               0x0500
#define STD_GET_DESCRIPTOR            0x0680
#define STD_SET_DESCRIPTOR            0x0700
#define STD_GET_CONFIGURATION         0x0880
#define STD_SET_CONFIGURATION         0x0900
#define STD_GET_INTERFACE             0x0A81
#define STD_SET_INTERFACE             0x0B01
#define STD_SYNCH_FRAME               0x0C82

/* CDC Class Specific Request Code */
#define GET_LINE_CODING               0x21A1
#define SET_LINE_CODING               0x2021
#define SET_CONTROL_LINE_STATE        0x2221


typedef struct 
{
    unsigned int dwDTERRate;
    char bCharFormat;
    char bParityType;
    char bDataBits;
} USB_CDC_LINE_CODING;


USB_CDC_LINE_CODING line =
{
    115200, // baudrate
    0,      // 1 Stop Bit
    0,      // None Parity
    8
};     // 8 Data bits


static usb_dev_t usb_dev;


static void 
usb_control_write (usb_t usb, const char *data, usb_size_t length)
{
    AT91PS_UDP pUDP = usb->pUDP;
    uint32_t cpt;
    AT91_REG csr;

    do {
        cpt = MIN (length, 8);
        length -= cpt;

        while (cpt--)
            pUDP->UDP_FDR[0] = *data++;

        if (pUDP->UDP_CSR[0] & AT91C_UDP_TXCOMP)
        {
            pUDP->UDP_CSR[0] &= ~AT91C_UDP_TXCOMP;
            while (pUDP->UDP_CSR[0] & AT91C_UDP_TXCOMP);
        }

        pUDP->UDP_CSR[0] |= AT91C_UDP_TXPKTRDY;
        do {
            csr = pUDP->UDP_CSR[0];

            // Data IN stage has been stopped by a status OUT
            if (csr & AT91C_UDP_RX_DATA_BK0) 
            {
                pUDP->UDP_CSR[0] &= ~AT91C_UDP_RX_DATA_BK0;
                return;
            }
        } while (! (csr & AT91C_UDP_TXCOMP));

    } while (length);

    if (pUDP->UDP_CSR[0] & AT91C_UDP_TXCOMP)
    {
        pUDP->UDP_CSR[0] &= ~AT91C_UDP_TXCOMP;
        while (pUDP->UDP_CSR[0] & AT91C_UDP_TXCOMP)
            continue;
    }
}


static void
usb_control_write_zlp (usb_t usb)
{
    AT91PS_UDP pUDP = usb->pUDP;

    pUDP->UDP_CSR[0] |= AT91C_UDP_TXPKTRDY;
    while (! (pUDP->UDP_CSR[0] & AT91C_UDP_TXCOMP))
        continue;

    pUDP->UDP_CSR[0] &= ~AT91C_UDP_TXCOMP;
    while (pUDP->UDP_CSR[0] & AT91C_UDP_TXCOMP)
        continue;
}


static void
usb_control_stall (usb_t usb)
{
    AT91PS_UDP pUDP = usb->pUDP;

    pUDP->UDP_CSR[0] |= AT91C_UDP_FORCESTALL;
    while (! (pUDP->UDP_CSR[0] & AT91C_UDP_ISOERROR))
        continue;

    pUDP->UDP_CSR[0] &= ~(AT91C_UDP_FORCESTALL | AT91C_UDP_ISOERROR);
    while (pUDP->UDP_CSR[0] & (AT91C_UDP_FORCESTALL | AT91C_UDP_ISOERROR))
        continue;
}


static void
usb_enumerate (usb_t usb)
{
    AT91PS_UDP pUDP = usb->pUDP;
    uint8_t request_type;
    uint8_t request;
    uint16_t value;
    uint16_t index;
    uint16_t length;
    uint16_t status;

    if (! (pUDP->UDP_CSR[0] & AT91C_UDP_RXSETUP))
        return;

    request_type = pUDP->UDP_FDR[0];
    request = pUDP->UDP_FDR[0];

    value = (pUDP->UDP_FDR[0] & 0xFF);
    value |= (pUDP->UDP_FDR[0] << 8);

    index = (pUDP->UDP_FDR[0] & 0xFF);
    index |= (pUDP->UDP_FDR[0] << 8);

    length = (pUDP->UDP_FDR[0] & 0xFF);
    length |= (pUDP->UDP_FDR[0] << 8);

    if (request_type & 0x80)
    {
        pUDP->UDP_CSR[0] |= AT91C_UDP_DIR;
        while (! (pUDP->UDP_CSR[0] & AT91C_UDP_DIR))
            continue;
    }
    pUDP->UDP_CSR[0] &= ~AT91C_UDP_RXSETUP;

    while ((pUDP->UDP_CSR[0]  & AT91C_UDP_RXSETUP))
        continue;

    switch ((request << 8) | request_type) 
    {
    case STD_GET_DESCRIPTOR:
        switch (value)
        {
        case 0x100:             // Return Device Descriptor
            usb_control_write (usb, devDescriptor, 
                               MIN (sizeof (devDescriptor), length));
            break;
        case 0x200:             // Return Configuration Descriptor
            usb_control_write (usb, cfgDescriptor, 
                               MIN (sizeof (cfgDescriptor), length));
            break;
        default:
            usb_control_stall (usb);
        }
        break;

    case STD_SET_ADDRESS:
        usb_control_write_zlp (usb);
        pUDP->UDP_FADDR = (AT91C_UDP_FEN | value);
        pUDP->UDP_GLBSTATE  = (value) ? AT91C_UDP_FADDEN : 0;
        break;

    case STD_SET_CONFIGURATION:
        usb->configured = value;
        usb_control_write_zlp (usb);
        pUDP->UDP_GLBSTATE  = (value) ? AT91C_UDP_CONFG : AT91C_UDP_FADDEN;

        pUDP->UDP_CSR[1] = (value) ? (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_OUT) : 0;
        pUDP->UDP_CSR[2] = (value) ? (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_IN)  : 0;
        pUDP->UDP_CSR[3] = (value) ? (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_ISO_IN)   : 0;
        break;

    case STD_GET_CONFIGURATION:
        usb_control_write (usb, (char *) &usb->configured,
                           sizeof (usb->configured));
        break;

    case STD_GET_STATUS_ZERO:
        status = 0;
        usb_control_write (usb, (char *) &status, sizeof (status));
        break;

    case STD_GET_STATUS_INTERFACE:
        status = 0;
        usb_control_write (usb, (char *) &status, sizeof (status));
        break;

    case STD_GET_STATUS_ENDPOINT:
        status = 0;
        index &= 0x0F;
        if ((pUDP->UDP_GLBSTATE & AT91C_UDP_CONFG) && (index <= 3))
        {
            status = (pUDP->UDP_CSR[index] & AT91C_UDP_EPEDS) ? 0 : 1;
            usb_control_write (usb, (char *) &status, sizeof (status));
        }
        else if ((pUDP->UDP_GLBSTATE & AT91C_UDP_FADDEN) && (index == 0))
        {
            status = (pUDP->UDP_CSR[index] & AT91C_UDP_EPEDS) ? 0 : 1;
            usb_control_write (usb, (char *) &status, sizeof (status));
        }
        else
            usb_control_stall (usb);
        break;

    case STD_SET_FEATURE_ZERO:
        usb_control_stall (usb);
        break;

    case STD_SET_FEATURE_INTERFACE:
        usb_control_write_zlp (usb);
        break;

    case STD_SET_FEATURE_ENDPOINT:
        index &= 0x0F;
        if ((value == 0) && index && (index <= 3))
        {
            pUDP->UDP_CSR[index] = 0;
            usb_control_write_zlp (usb);
        }
        else
            usb_control_stall (usb);
        break;

    case STD_CLEAR_FEATURE_ZERO:
        usb_control_stall (usb);
        break;

    case STD_CLEAR_FEATURE_INTERFACE:
        usb_control_write_zlp (usb);
        break;

    case STD_CLEAR_FEATURE_ENDPOINT:
        index &= 0x0F;
        if ((value == 0) && index && (index <= 3)) 
        {
            /* Configure and enable selected endpoint.  */
            switch (index)
            {
            case 1:
                pUDP->UDP_CSR[1] = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_OUT);
                break;
            case 2:
                pUDP->UDP_CSR[2] = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_IN);
                break;
            case 3:
                pUDP->UDP_CSR[3] = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_ISO_IN);
                break;
            }
            usb_control_write_zlp (usb);
        }
        else
            usb_control_stall (usb);
        break;

        // handle CDC class requests
    case SET_LINE_CODING:
        while (! (pUDP->UDP_CSR[0] & AT91C_UDP_RX_DATA_BK0));
        pUDP->UDP_CSR[0] &= ~AT91C_UDP_RX_DATA_BK0;
        usb_control_write_zlp (usb);
        break;

    case GET_LINE_CODING:
        usb_control_write (usb, (char *) &line, MIN (sizeof (line), length));
        break;

    case SET_CONTROL_LINE_STATE:
        usb->connection = value;
        usb_control_write_zlp (usb);
        break;

    default:
        usb_control_stall (usb);
        break;
    }
}


usb_size_t
usb_read (usb_t usb, void *buffer, usb_size_t length)
{
    AT91PS_UDP pUDP = usb->pUDP;
    unsigned int rx_bytes;
    unsigned int total;
    uint32_t rx_bank = usb->rx_bank;
    uint8_t *data;

    data = buffer;
    total = 0;

    while (length)
    {
        if (usb->rx_bytes)
        {
            rx_bytes = MIN (usb->rx_bytes, length);
            length -= rx_bytes;
            usb->rx_bytes -= rx_bytes;

            /* Transfer data from FIFO.  */
            while (rx_bytes--)
                data[total++] = pUDP->UDP_FDR[AT91C_EP_OUT];

            if (!usb->rx_bytes)
            {
                /* Indicate finished reading current bank.  */
                pUDP->UDP_CSR[AT91C_EP_OUT] &= ~rx_bank;
                
                /* Switch to other bank.  */
                if (rx_bank == AT91C_UDP_RX_DATA_BK0)
                    rx_bank = AT91C_UDP_RX_DATA_BK1;
                else
                    rx_bank = AT91C_UDP_RX_DATA_BK0;
            }
        }

        if (!length)
            break;

        if (! usb_configured_p (usb))
            break;

        if (pUDP->UDP_CSR[AT91C_EP_OUT] & rx_bank) 
        {
            /* It appears that the received byte count is not
               decremented after reads from the FIFO so we keep our
               own count.  */
            usb->rx_bytes = pUDP->UDP_CSR[AT91C_EP_OUT] >> 16;
        }
    }
    usb->rx_bank = rx_bank;
    return total;
}


bool
usb_read_ready_p (usb_t usb)
{
    AT91PS_UDP pUDP = usb->pUDP;
    uint32_t rx_bank = usb->rx_bank;

    if (usb->rx_bytes)
        return 1;

    if (! usb_configured_p (usb))
        return 0;

    if (! (pUDP->UDP_CSR[AT91C_EP_OUT] & rx_bank))
        return 0;
    
    return (pUDP->UDP_CSR[AT91C_EP_OUT] >> 16) != 0;
}


usb_size_t
usb_write (usb_t usb, const void *buffer, usb_size_t length)
{
    AT91PS_UDP pUDP = usb->pUDP;
    unsigned int tx_bytes = 0;
    unsigned int total;
    const uint8_t *data;

    if (! usb_configured_p (usb))
        return 0;

    data = buffer;
    total = 0;

    tx_bytes = MIN (length, AT91C_EP_IN_SIZE);
    length -= tx_bytes;
    total += tx_bytes;

    while (tx_bytes--) 
        pUDP->UDP_FDR[AT91C_EP_IN] = *data++;

    pUDP->UDP_CSR[AT91C_EP_IN] |= AT91C_UDP_TXPKTRDY;

    while (length)
    {
        // Fill the second bank
        tx_bytes = MIN (length, AT91C_EP_IN_SIZE);
        total += tx_bytes;
        length -= tx_bytes;
        while (tx_bytes--)
            pUDP->UDP_FDR[AT91C_EP_IN] = *data++;

        // Wait for the the first bank to be sent
        while (! (pUDP->UDP_CSR[AT91C_EP_IN] & AT91C_UDP_TXCOMP))
            if (! usb_configured_p (usb))
                return total;

        pUDP->UDP_CSR[AT91C_EP_IN] &= ~AT91C_UDP_TXCOMP;
        while (pUDP->UDP_CSR[AT91C_EP_IN] & AT91C_UDP_TXCOMP);
        pUDP->UDP_CSR[AT91C_EP_IN] |= AT91C_UDP_TXPKTRDY;
    }

    // Wait for the end of transfer
    while (! (pUDP->UDP_CSR[AT91C_EP_IN] & AT91C_UDP_TXCOMP))
        if (! usb_configured_p (usb))
            return total;

    pUDP->UDP_CSR[AT91C_EP_IN] &= ~AT91C_UDP_TXCOMP;
    while (pUDP->UDP_CSR[AT91C_EP_IN] & AT91C_UDP_TXCOMP)
        continue;

    return total;
}


bool
usb_configured_p (usb_t usb)
{
    AT91PS_UDP pUDP = usb->pUDP;
    AT91_REG isr = pUDP->UDP_ISR;
    
    if (isr & AT91C_UDP_ENDBUSRES) 
    {
        /* When the host has detected D+ being high it sends an end bus
           reset.  */

        pUDP->UDP_ICR = AT91C_UDP_ENDBUSRES;
        // Reset all endpoints
        pUDP->UDP_RSTEP  = ~0;
        pUDP->UDP_RSTEP  = 0;
        // Enable the function
        pUDP->UDP_FADDR = AT91C_UDP_FEN;
        // Configure endpoint 0
        pUDP->UDP_CSR[0] = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_CTRL);
        usb->configured = 0;
    }
    else if (isr & AT91C_UDP_EPINT0)
    {
        /* We have received something on the control endpoint (0);
           hopefully it is the host trying to enumerate us.  */

        pUDP->UDP_ICR = AT91C_UDP_EPINT0;
        usb_enumerate (usb);
    }

    return usb->configured;
}


void
usb_connect (usb_t dev __UNUSED__)
{
    /* Connect pull-up, wait for configuration.  This does nothing if the
       pull-up is always connected.  */

#ifdef USB_PIO_PULLUP
    // Enable UDP PullUp (USB_DP_PUP) : enable and clear of the
    // corresponding PIO.  Set in PIO mode and configure as output.
    pio_config (USB_PIO_PULLUP, PIO_OUTPUT);
    /* Set low to enable pullup.  */
    pio_output_low (USB_PIO_PULLUP);
#endif
}


void
usb_shutdown (void)
{
    AT91PS_UDP pUDP = AT91C_BASE_UDP;

    /* The USB transceiver is enabled by default.  */

    /* Enable System Peripheral USB Clock.  */
    AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_UDP);

    /* Disable transceiver.  */
    pUDP->UDP_TXVC = 0x100;

    /* Disable System Peripheral USB Clock.  */
    AT91C_BASE_PMC->PMC_PCDR = BIT (AT91C_ID_UDP);
}


usb_t
usb_init (void)
{
    usb_t usb = &usb_dev;

    /* Set the PLL USB Divider (PLLCK / 2).  This assumes that the PLL
       clock is 96 MHz since the USB clock must be 48 MHz.  */
    AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1 ;

    /* Enable the 48 MHz USB clock UDPCK.  */
    AT91C_BASE_PMC->PMC_SCER = AT91C_PMC_UDP;

    /* Enable System Peripheral USB Clock.  */
    AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_UDP);

    /* Signal the host by pulling D+ high.  This might be pulled high
       with an external 1k5 resistor.  */
    usb_connect (usb);

    usb->pUDP = AT91C_BASE_UDP;
    usb->configured = 0;
    usb->connection = 0;
    usb->rx_bytes = 0;
    usb->rx_bank = AT91C_UDP_RX_DATA_BK0;

    return usb;
}



/** Read character.  This blocks until the character can be read.  */
int8_t
usb_getc (usb_t usb)
{
    uint8_t ch;

    usb_read (usb, &ch, sizeof (ch));
    return ch;
}


/** Write character.  This blocks until the character can be
    written.  */
int8_t
usb_putc (usb_t usb, char ch)
{
    if (ch == '\n')
        usb_putc (usb, '\r');    

    usb_write (usb, &ch, sizeof (ch));
    return ch;
}


/** Write string.  This blocks until the string is buffered.  */
int8_t
usb_puts (usb_t usb, const char *str)
{
    while (*str)
        usb_putc (usb, *str++);
    return 1;
}

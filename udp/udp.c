#include "irq.h"
#include "trace.h"
#include "udp.h"


/* 
   UDP USB device port.

   The AT91 UDP has two communication endpoints (EP1 and EP2) that
   support bulk and isochronous transfers using 64-byte ping-pong
   FIFOs.  EP1 is the bulk IN endpoint (to host) and EP2 is the bulk
   OUT endpoint (from host).

   EP0 is the control endpoint used for the enumeration process.  This
   is bidirectional with an 8-byte FIFO.

   USB is a master-slave bus with one master and multiple slaves. The
   slaves are called peripherals or functions and the master is called
   the host.  Only the host can initiate USB transfers.

   USB uses differential signalling except for bus reset and end of
   packet signals where D+ and D- are both low.

   When no device is connected, the USB D+ and D- signals are tied to
   GND by 15 K pull-down resistors integrated in the hub downstream
   ports.   The USB device is in the not powered state.

   When a device is connected to a hub downstream port, VBUS goes to 5V
   and the device is in the powered state.

   The device then connects a 1.5 K pull-up resistor on D+ (this might
   be switched using a MOSFET or connected to VBUS).  The USB bus line
   goes into IDLE state, D+ is pulled up by the device 1.5 K resistor
   to 3.3 V and D- is pulled down by the 15 K resistor of the host.
   The USB device is in the attached state.

   The device then waits for an end of bus reset from the host (both
   D+ and D- pulled low for at least 10 ms) and then enters the
   default state.  If the device supports high speed it sends a chirp
   K and the host responds with a sequnce of alternating chirp K and
   chirp J (KJKJKJ).

   In the default state the control endpoint (0) is enabled and the
   device waits to be enumerated by the host; the host sends a number
   of setup requests on the control endpoint and the device responds
   sending data back on the control endpoint.

   The first host request is for the device descriptor?

   The host then sets the address to use and the the device enters the
   address state.

   Once a valid set configuration standard request has been received
   and acknowledged, the device enables endpoints corresponding to the
   current configuration.  The device then enters the configured
   state.  If configuration 0 is selected then we need to go ack to
   the address state.

   Note, if there is a setup request the device does not know (for
   example, a request for a device qualifier descriptor for a non
   high-speed device), the device responds by stalling the control
   endpoint.

   GET_DESC 0x100   Device
   SET_ADDR addr
   GET_DESC 0x100   Device
   GET_DESC 0x600   Device qualifier
   GET_DESC 0x600   Device qualifier
   GET_DESC 0x600   Device qualifier
   GET_DESC 0x200   Config 
   GET_DESC 0x200   Config
   GET_DESC 0x300   String 0
   GET_DESC 0x302   String 2  product
   GET_DESC 0x301   String 1  manufacturer
   GET_DESC 0x303   String 1  serial
   SET_CONFIG 1

   Note for a full speed device D+ is pulled up while for a low speed device
   D- is pulled up.

   The default Vendor ID is Atmel's vendor ID 0x03EB.  The default
   product ID is 0x6124.  With these lsusb gives
   ``Atmel Corp. at91sam SAMBA bootloader''

   
   FIXME, FIXME, FIXME.  This implementation does far too much
   processing in the context of interrupt handlers.  The callbacks are
   all called from interrupts.  The dilemma is that we need to process
   setup transations rapidly.  These transations only use the control
   endpoint and are only a few bytes so fancy buffering should not be
   needed.

   Setup transactions only occur on EP0; they cannot use EPs with
   ping-pong buffers.  They must be handled as quickly as possible.
   When we get a setup request on the control EP, we respond with either:
    1. A stall if the request is unrecognised
    2. A zlp to acknowledge the request if no data is to be returned
    3. The requested data
   In each case we write to the control EP.

   To make the handling of setup requests responsive, we can use the
   interrupt handler to read from the control EP.  To process the
   request we can invoke a callback function but this will run in the
   interrupt context.  Alternatively, we can buffer the request and
   process it later when we poll this driver.  However, this can
   introduce considerable latency if the polling rate is slow.

   If we process the setup request in the interrupt handler we need to
   write to the control EP.  This operation must be completed before
   we can set the address state.  We can poll the TXCOMP interrupt
   that indicates that the host has received the data payload.  But we
   don't really want to poll during the interrupt handler.  One
   solution is to register a callback function that is invoked when
   the TXCOMP interrupt occurs.  The only functions that need to be
   called are udp_address_set and udp_configuration_set.  Another
   solution is to add some states and have the interrupt handler
   call these functions as required.

   If there is a protocol error an endpoint can be halted.  The host
   can use the standard control messages get-status, set-feature and
   clear-feature to examine and manipulate the halted status of a
   given endpoint.  The the halted condition can be cleared.

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


#ifndef TRACE_UDP_ERROR
#define TRACE_UDP_ERROR(...)
#endif


#ifndef TRACE_UDP_INFO
#define TRACE_UDP_INFO(...)
#endif


#ifndef TRACE_UDP_DEBUG
#define TRACE_UDP_DEBUG(...)
#endif




/** Set flag(s) in a register */
#define SET(register, flags)        ((register) = (register) | (flags))
/** Clear flag(s) in a register */
#define CLEAR(register, flags)      ((register) &= ~(flags))

/** Poll the status of flags in a register */
#define ISSET(register, flags)      (((register) & (flags)) == (flags))
/** Poll the status of flags in a register */
#define ISCLEARED(register, flags)  (((register) & (flags)) == 0)


#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif


#ifndef HIGH_BYTE
#define HIGH_BYTE(v) (((v) >> 8) & 0xff)
#endif


#ifndef LOW_BYTE
#define LOW_BYTE(v) ((v) & 0xff)
#endif

/** Clear flags of UDP UDP_CSR register and waits for synchronization */
#define udp_ep_flag_clear(pInterface, endpoint, flags) { \
while (pInterface->UDP_CSR[endpoint] & (flags)) \
    pInterface->UDP_CSR[endpoint] &= ~ (flags); \
}

/** Set flags of UDP UDP_CSR register and waits for synchronization */
#define udp_ep_flag_set(pInterface, endpoint, flags) { \
while ((pInterface->UDP_CSR[endpoint] & (flags)) != (flags) ) \
    pInterface->UDP_CSR[endpoint] |= (flags); \
}


typedef enum
{
    UDP_STATE_NOT_POWERED,
    UDP_STATE_ATTACHED,
    UDP_STATE_POWERED,
    UDP_STATE_DEFAULT,
    UDP_STATE_ADDRESS,
    UDP_STATE_CONFIGURED,
    UDP_STATE_SUSPENDED
} udp_state_t;


/**
 * Possible endpoint states
 * 
 */
typedef enum 
{
    UDP_EP_STATE_DISABLED, 
    UDP_EP_STATE_IDLE, 
    UDP_EP_STATE_WRITE, 
    UDP_EP_STATE_READ, 
    UDP_EP_STATE_HALTED
} udp_ep_state_t;


static const char devDescriptor[] =
{
    /* Device descriptor */
    0x12,   // bLength
    0x01,   // bDescriptorType
    0x10,   // bcdUSBL
    0x01,   //
    0x00,   // bDeviceClass (use class specified by interface)
    0x00,   // bDeviceSubclass
    0x00,   // bDeviceProtocol
    0x08,   // bMaxPacketSize0
    LOW_BYTE (USB_VENDOR_ID),    // idVendorL
    HIGH_BYTE (USB_VENDOR_ID),   //
    LOW_BYTE (USB_PRODUCT_ID),   // idProductL
    HIGH_BYTE (USB_PRODUCT_ID),  //
    LOW_BYTE (USB_RELEASE_ID),   // bcdDeviceL
    HIGH_BYTE (USB_RELEASE_ID),  //
    0x00,   // iManufacturer
    0x00,   // iProduct
    0x00,   // SerialNumber
    0x01    // bNumConfigs
};


static udp_dev_t udp_dev;

/**
 * Structure for endpoint transfer parameters
 * 
 */
typedef struct
{
    /* Pointer to where data is stored.  */
    uint8_t *pData;
    udp_transfer_t transfer;
    udp_callback_t callback;
    /* Callback argument.  */
    void *arg;
    uint16_t max_packet_size;
    uint8_t num_fifo;
    udp_ep_state_t state;
    uint8_t bank;
} udp_ep_info_t;


struct udp_dev_struct
{
    AT91PS_UDP pUDP;
    uint32_t rx_bank;
    uint16_t rx_bytes;
    uint8_t connection;
    /* Chosen configuration, 0 if not configured.  */
    uint8_t configuration;
    udp_request_handler_t request_handler;
    void *request_handler_arg;
    volatile udp_state_t state;
    volatile udp_state_t prev_state;
    udp_setup_t setup;
    udp_ep_info_t eps[UDP_EP_NUM];
};


static udp_dev_t udp_dev;

/** Interrupt mask */
#define ISR_MASK 0x00003FFF


static void udp_bus_reset_handler (udp_t udp);

static void udp_endpoint_handler (udp_t udp, udp_ep_t endpoint);

/**
 * Returns the index of the last set (1) bit in an integer
 * 
 * \param  value Integer value to parse
 * \return Position of the leftmost set bit in the integer
 * 
 */
static inline signed char 
last_set_bit (unsigned int value)
{
    signed char locindex = -1;

    if (value & 0xFFFF0000)
    {
        locindex += 16;
        value >>= 16;
    }

    if (value & 0xFF00)
    {
        locindex += 8;
        value >>= 8;
    }

    if (value & 0xF0)
    {
        locindex += 4;
        value >>= 4;
    }

    if (value & 0xC)
    {
        locindex += 2;
        value >>= 2;
    }

    if (value & 0x2)
    {
        locindex += 1;
        value >>= 1;
    }

    if (value & 0x1)
    {
        locindex++;
    }

    return locindex;
}


/**
 * UDP interrupt service routine
 * 
 * Handles all UDP peripheral interrupts
 * 
 */
static void
udp_irq_handler (void)
{
    udp_ep_t endpoint;
    udp_t udp = &udp_dev;
    AT91PS_UDP pUDP = udp->pUDP;
    uint32_t status;

#if 0
    // End interrupt if we are not attached to UDP bus
    if (udp->state == UDP_STATE_NOT_POWERED)
        return;
#endif

    status = pUDP->UDP_ISR & pUDP->UDP_IMR & ISR_MASK;

    while (status != 0)
    {
        // Start of frame (SOF)
        if (ISSET (status, AT91C_UDP_SOFINT))
        {
            // TRACE_INFO (UDP, "UDP:SOF\n");
            // Acknowledge interrupt
            SET (pUDP->UDP_ICR, AT91C_UDP_SOFINT);
            CLEAR (status, AT91C_UDP_SOFINT);
        }        

        // Suspend
        if (ISSET (status, AT91C_UDP_RXSUSP))
        {
            // TRACE_INFO (UDP, "UDP:Susp\n");
           
            if (udp->state != UDP_STATE_SUSPENDED)
            {
                // The device enters the Suspended state
                //      MCK + UDPCK must be off
                //      Pull-Up must be connected
                //      Transceiver must be disabled

                // Enable wakeup
                SET (pUDP->UDP_IER, AT91C_UDP_WAKEUP | AT91C_UDP_RXRSM);

                // Acknowledge interrupt
                SET (pUDP->UDP_ICR, AT91C_UDP_RXSUSP);

                // Set suspended state
                udp->prev_state = udp->state;
                udp->state = UDP_STATE_SUSPENDED;

                // Disable transceiver
                SET (pUDP->UDP_TXVC, AT91C_UDP_TXVDIS);
                // Disable master clock
                AT91C_BASE_PMC->PMC_PCDR |= (1 << AT91C_ID_UDP);
                // Disable peripheral clock for UDP
                AT91C_BASE_PMC->PMC_SCDR |= AT91C_PMC_UDP;

                // TODO, we need to pull less than 500 uA from the 5V VBUS
                // so need a callback or the user should poll
                // for the suspend state.
            }            
        }
        // Resume
        else if (ISSET (status, AT91C_UDP_WAKEUP) 
                 || ISSET (status, AT91C_UDP_RXRSM))
        {
            // TRACE_INFO (UDP, "UDP:Resm\n");

            // The device enters configured state
            //      MCK + UDPCK must be on
            //      Pull-Up must be connected
            //      Transceiver must be enabled
            // Powered state
            // Enable master clock
            AT91C_BASE_PMC->PMC_PCER |= (1 << AT91C_ID_UDP);
            // Enable peripheral clock for UDP
            AT91C_BASE_PMC->PMC_SCER |= AT91C_PMC_UDP;

            if (udp->prev_state == UDP_STATE_DEFAULT)
            {
                // Enable transceiver
                CLEAR (pUDP->UDP_TXVC, AT91C_UDP_TXVDIS);
            }

            udp->state = udp->prev_state;

            SET (pUDP->UDP_ICR, AT91C_UDP_WAKEUP | AT91C_UDP_RXRSM
                 | AT91C_UDP_RXSUSP);
            SET (pUDP->UDP_IDR, AT91C_UDP_WAKEUP | AT91C_UDP_RXRSM);
        }        
        // End of bus reset (non maskable)
        else if (ISSET (status, AT91C_UDP_ENDBUSRES))
        {
            // TRACE_INFO (UDP, "UDP:EoBres\n");

            // Initialize UDP peripheral device
            udp_bus_reset_handler (udp);

            udp->state = UDP_STATE_DEFAULT;

            // Flush and enable the Suspend interrupt
            SET (pUDP->UDP_ICR, AT91C_UDP_WAKEUP 
                 | AT91C_UDP_RXRSM | AT91C_UDP_RXSUSP);

            // Acknowledge end of bus reset interrupt.  Is this
            // required since it is done in udp_bus_reset_handler?
            SET (pUDP->UDP_ICR, AT91C_UDP_ENDBUSRES);
        }
        // Endpoint interrupts
        else 
        {
            while (status != 0)
            {
                // Get endpoint index
                endpoint = last_set_bit (status);
                udp_endpoint_handler (&udp_dev, endpoint);
                
                CLEAR (pUDP->UDP_ICR, (1 << endpoint));
                
                CLEAR (status, 1 << endpoint);
            }
        }          

        status = pUDP->UDP_ISR & pUDP->UDP_IMR & ISR_MASK;
       
        // Mask unneeded interrupts
        if (udp->state != UDP_STATE_DEFAULT)
        {
            status &= AT91C_UDP_ENDBUSRES | AT91C_UDP_SOFINT;
        }
        
    }
}


/**
 * End transfer of given endpoint.  This is called when an
 * interrupt indicates that a transfer has finished.
 * 
 * \param   endpoint    Endpoint where to end transfer
 * \param   status      Status code returned by the transfer operation
 * 
 */
static __inline__
void udp_completed (udp_t udp, udp_ep_t endpoint, udp_status_t status)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];

    if ((pep->state == UDP_EP_STATE_WRITE)
        || (pep->state == UDP_EP_STATE_READ)) 
    {
        TRACE_DEBUG (UDP, "UDP:EoT%d %d\n",
                     endpoint, pep->transfer.transferred);

        pep->state = UDP_EP_STATE_IDLE;

        pep->transfer.status = status;
        
        // Invoke callback if present
        if (pep->callback != 0) 
            pep->callback (pep->arg, &pep->transfer);
    }
}


/**
 * Configure specific endpoint
 * \param   endpoint Endpoint to configure
 * 
 */
static void
udp_endpoint_configure (udp_t udp, udp_ep_t endpoint)
{
    AT91PS_UDP pUDP = udp->pUDP;
    udp_ep_info_t *pep = &udp->eps[endpoint];

    // Abort the current transfer if the endpoint was configured and in
    // write or read state
    if ((pep->state == UDP_EP_STATE_READ)
        || (pep->state == UDP_EP_STATE_WRITE))
    {
        udp_completed (udp, endpoint, UDP_STATUS_RESET);
    }

    pep->state = UDP_EP_STATE_IDLE;

    // Reset endpoint transfer descriptor
    pep->pData = 0;
    pep->transfer.remaining = 0;
    pep->transfer.transferred = 0;
    pep->transfer.buffered = 0;
    pep->callback = 0;
    pep->arg = 0;

    // Reset endpoint FIFOs
    SET (pUDP->UDP_RSTEP, 1 << endpoint);
    CLEAR (pUDP->UDP_RSTEP, 1 << endpoint);

    switch (endpoint)
    {
    case UDP_EP_CONTROL:
        udp->eps[0].max_packet_size = UDP_EP_CONTROL_SIZE;
        udp->eps[0].num_fifo = 1;
        pUDP->UDP_CSR[0] = AT91C_UDP_EPTYPE_CTRL | AT91C_UDP_EPEDS;
        break;
    
    case UDP_EP_OUT:
        udp->eps[1].max_packet_size = UDP_EP_OUT_SIZE;
        udp->eps[1].num_fifo = 2;
        pUDP->UDP_CSR[1] = AT91C_UDP_EPTYPE_BULK_OUT | AT91C_UDP_EPEDS;
        break;

    case UDP_EP_IN:
        udp->eps[2].max_packet_size = UDP_EP_IN_SIZE;
        udp->eps[2].num_fifo = 2;
        pUDP->UDP_CSR[2] = AT91C_UDP_EPTYPE_BULK_IN | AT91C_UDP_EPEDS;         
        break;
    }

    TRACE_DEBUG (UDP, "UDP:Cfg%d\n", endpoint);
}


/**
 * Clear transmission status flag and swap banks for dual FIFO endpoints.
 * \param   endpoint    Endpoint where to clear flag
 */
static void 
udp_rx_flag_clear (udp_t udp, udp_ep_t endpoint)
{
    AT91PS_UDP pUDP = udp->pUDP;
    udp_ep_info_t *pep = &udp->eps[endpoint];

    // Clear flag
    udp_ep_flag_clear (pUDP, endpoint, pep->bank);

    // Swap banks
    if (pep->bank == AT91C_UDP_RX_DATA_BK0) 
    {
        if (pep->num_fifo > 1)
        {
            // Swap bank if in dual-fifo mode
            pep->bank = AT91C_UDP_RX_DATA_BK1;
        }
    }
    else 
    {
        pep->bank = AT91C_UDP_RX_DATA_BK0;
    }    
}


/**
 * Writes data to UDP FIFO
 * \param   endpoint    Endpoint to write data
 * \return  Number of bytes written
 * 
 */
static unsigned int
udp_write_payload (udp_t udp, udp_ep_t endpoint)
{
    AT91PS_UDP pUDP = udp->pUDP;
    udp_ep_info_t *pep = &udp->eps[endpoint];
    unsigned int bytes;
    unsigned int i;
    uint8_t *src;

    // Get the number of bytes to send
    bytes = MIN (pep->max_packet_size, 
                 pep->transfer.remaining);

    // Transfer one packet to the FIFO buffer
    src = pep->pData;
    for (i = 0; i < bytes; i++)
        pUDP->UDP_FDR[endpoint] = *src++;

    pep->transfer.buffered += bytes;
    pep->transfer.remaining -= bytes;

    return bytes;
}


/**
 * Send data packet via given endpoint
 * \param   endpoint    Endpoint to send data through
 * \param   pData       Pointer to data buffer
 * \param   len         Packet size
 * \param   callback   Optional callback to invoke when the read finishes
 * \param   arg   Optional callback argument 
 * \return  Operation result code
 * 
 */
udp_status_t
udp_write_async (udp_t udp, udp_ep_t endpoint, const void *pData, 
                 unsigned int len, udp_callback_t callback, void *arg)
{
    AT91PS_UDP pUDP = udp->pUDP;
    udp_ep_info_t *pep = &udp->eps[endpoint];

    if (pep->state != UDP_EP_STATE_IDLE) 
        return UDP_STATUS_LOCKED;

    TRACE_INFO (UDP, "UDP:Write%d %d\n", endpoint, len);

    pep->pData = (uint8_t *)pData;
    pep->transfer.status = UDP_STATUS_PENDING;
    pep->transfer.remaining = len;
    pep->transfer.buffered = 0;
    pep->transfer.transferred = 0;
    pep->callback = callback;
    pep->arg = arg;
    pep->state = UDP_EP_STATE_WRITE;

    // Send one packet
    udp_write_payload (udp, endpoint);

    // Say that there is data ready.  If writing a zero length packet
    // then udp_write_payload is a nop.
    udp_ep_flag_set (pUDP, endpoint, AT91C_UDP_TXPKTRDY);

    // If double buffering is enabled and there is data remaining, 
    // prepare another packet
    if ((pep->num_fifo > 1) && (pep->transfer.remaining > 0))
        udp_write_payload (udp, endpoint);

    // Enable interrupt on endpoint
    SET (pUDP->UDP_IER, 1 << endpoint);
    
    return UDP_STATUS_SUCCESS;
}


/**
 * Read data from UDP FIFO
 * \param   endpoint    Endpoint to read data from
 * \param   packetsize  Maximum size of packet to receive
 * \return  Number of bytes read.
 * 
 */
static unsigned int 
udp_read_payload (udp_t udp, udp_ep_t endpoint, unsigned int packetsize)
{
    AT91PS_UDP pUDP = udp->pUDP;
    udp_ep_info_t *pep = &udp->eps[endpoint];
    unsigned int bytes;
    unsigned int i;
    uint8_t *dst;

    // Get number of bytes to retrieve
    bytes = MIN (pep->transfer.remaining, packetsize);

    // Read packet from FIFO
    dst = pep->pData;
    for (i = 0; i < bytes; i++) 
        *dst++ = pUDP->UDP_FDR[endpoint];

    pep->transfer.remaining -= bytes;
    pep->transfer.transferred += bytes;
    pep->transfer.buffered += packetsize - bytes;

    return bytes;
}


/**
 * Read data packet from given endpoint
 * 
 * Read a data packet with specific size from given endpoint.
 * 
 * \param   endpoint    Endpoint to send data through
 * \param   pData       Pointer to data buffer
 * \param   len         Packet size
 * \param   callback    Optional callback to invoke when the read finishes
 * \param   arg         Optional callback argument 
 * \return  Operation result code
 * 
 */
udp_status_t
udp_read_async (udp_t udp, udp_ep_t endpoint, void *pData, unsigned int len, 
                udp_callback_t callback, void *arg)
{
    AT91PS_UDP pUDP = udp->pUDP;
    udp_ep_info_t *pep = &udp->eps[endpoint];

    if (pep->state != UDP_EP_STATE_IDLE)
        return UDP_STATUS_LOCKED;

    TRACE_INFO (UDP, "UDP:Read%d %d\n", endpoint, len);

    pep->pData = pData;
    pep->transfer.status = UDP_STATUS_PENDING;
    pep->transfer.remaining = len;
    pep->transfer.buffered = 0;
    pep->transfer.transferred = 0;
    pep->callback = callback;
    pep->arg = arg;
    pep->state = UDP_EP_STATE_READ;
    
    // Enable interrupt on endpoint
    SET (pUDP->UDP_IER, 1 << endpoint);
    
    return UDP_STATUS_SUCCESS;
}


static void
udp_setup_read (udp_t udp, udp_ep_t endpoint)
{
    AT91PS_UDP pUDP = udp->pUDP;
    udp_setup_t *setup = &udp->setup;

    // Read setup request packet and squirrel away for later
    setup->type = pUDP->UDP_FDR[0];
    setup->request = pUDP->UDP_FDR[0];
    setup->value = pUDP->UDP_FDR[0] & 0xFF;
    setup->value |= (pUDP->UDP_FDR[0] << 8);
    setup->index = pUDP->UDP_FDR[0] & 0xFF;
    setup->index |= (pUDP->UDP_FDR[0] << 8);
    setup->length = pUDP->UDP_FDR[0] & 0xFF;
    setup->length |= (pUDP->UDP_FDR[0] << 8);
    
    // Set the DIR bit before clearing RXSETUP in Control IN sequence
    if (setup->type & 0x80)
        udp_ep_flag_set (pUDP, endpoint, AT91C_UDP_DIR);
    
    // Acknowledge that setup data packet has been read from FIFO
    udp_ep_flag_clear (pUDP, endpoint, AT91C_UDP_RXSETUP);
}

    
/**
 * UDP endpoint handler
 * \param   endpoint    Endpoint to use
 * 
 * Called from UDP interrupt handler in response to endpoint interrupt.
 */
static void 
udp_endpoint_handler (udp_t udp, udp_ep_t endpoint)
{
    AT91PS_UDP pUDP = udp->pUDP;    
    udp_ep_info_t *pep = &udp->eps[endpoint];
    unsigned int status = pUDP->UDP_CSR[endpoint];
    
    // IN packet sent
    if (ISSET (status, AT91C_UDP_TXCOMP))
    {
        // Check that endpoint is in Write state
        if (pep->state == UDP_EP_STATE_WRITE)
        {

            // End of transfer ?
            if ((pep->transfer.buffered < pep->max_packet_size)
                || (!ISCLEARED (status, AT91C_UDP_EPTYPE)
                    && (pep->transfer.remaining == 0)
                    && (pep->transfer.buffered == pep->max_packet_size)))
            {
                TRACE_DEBUG (UDP, "UDP:Write%d %d\n", endpoint,
                             pep->transfer.buffered);

                pep->transfer.transferred += pep->transfer.buffered;
                pep->transfer.buffered = 0;

                // Disable interrupt if this is not a control endpoint
                if (!ISCLEARED (status, AT91C_UDP_EPTYPE))
                    SET (pUDP->UDP_IDR, 1 << endpoint);

                udp_completed (udp, endpoint, UDP_STATUS_SUCCESS);
            }
            else
            {
                // Transfer remaining data
                TRACE_DEBUG (UDP, "UDP:Write%d +%d\n", endpoint,
                            pep->transfer.buffered);

                pep->transfer.transferred += pep->max_packet_size;
                pep->transfer.buffered -= pep->max_packet_size;

                // Send next packet
                if (pep->num_fifo == 1)
                {
                    // No double buffering
                    udp_write_payload (udp, endpoint);
                    udp_ep_flag_set (pUDP, endpoint, AT91C_UDP_TXPKTRDY);
                }
                else
                {
                    // Double buffering
                    udp_ep_flag_set (pUDP, endpoint, AT91C_UDP_TXPKTRDY);
                    udp_write_payload (udp, endpoint);
                }
            }
        }
                
        // Acknowledge interrupt
        udp_ep_flag_clear (pUDP, endpoint, AT91C_UDP_TXCOMP);
    }

    // OUT packet received
    if (ISSET (status, AT91C_UDP_RX_DATA_BK0) 
        || ISSET (status, AT91C_UDP_RX_DATA_BK1))
    {
        // Check that the endpoint is in Read state
        if (pep->state != UDP_EP_STATE_READ)
        {
            /* Endpoint is NOT in Read state so check the endpoint
               type to see if it is a control endpoint.  */
            if (ISCLEARED (status, AT91C_UDP_EPTYPE)
                && ISCLEARED (status, 0xFFFF0000))
            {
                /* Control endpoint, 0 bytes received.  This is the
                   host sending an Ack in response to us sending a
                   zlp.  Acknowledge the data and finish the current
                   transfer.  */
                TRACE_INFO (UDP, "UDP:Ack%d\n", endpoint);
                udp_rx_flag_clear (udp, endpoint);

                udp_completed (udp, endpoint, UDP_STATUS_SUCCESS);
            }
            else if (ISSET (status, AT91C_UDP_FORCESTALL))
            {
                /* Non-control endpoint so discard stalled data.  */
                TRACE_INFO (UDP, "UDP:Disc%d\n", endpoint);
                udp_rx_flag_clear (udp, endpoint);
            }
            else
            {
                /* Non-control endpoint so Nak data.  */
                TRACE_INFO (UDP, "UDP:Nak%d\n", endpoint);
                SET (pUDP->UDP_IDR, 1 << endpoint);
            }
        }
        else
        {
            /* Endpoint is in Read state so retrieve data and store it
               into the current transfer buffer.  */
            unsigned short packetsize = (unsigned short) (status >> 16);

            TRACE_DEBUG (UDP, "UDP:Read%d %d\n", endpoint, packetsize);

            udp_read_payload (udp, endpoint, packetsize);

            udp_rx_flag_clear (udp, endpoint);

            if ((pep->transfer.remaining == 0)
                || (packetsize < pep->max_packet_size))
            {
                // Disable interrupt if this is not a control endpoint
                if (!ISCLEARED (status, AT91C_UDP_EPTYPE))
                    SET (pUDP->UDP_IDR, 1 << endpoint);

                udp_completed (udp, endpoint, UDP_STATUS_SUCCESS);
            }
        }
    }

    // Setup packet received
    if (ISSET (status, AT91C_UDP_RXSETUP))
    {
        // This interrupt occurs as a result of receiving a setup packet
        TRACE_DEBUG (UDP, "UDP:Setup\n");

        // If a transfer was pending, complete it
        // Handle the case where during the status phase of a control write
        // transfer, the host receives the device ZLP and acks it, but the ack
        // is not received by the device
        if ((pep->state == UDP_EP_STATE_WRITE)
            || (pep->state == UDP_EP_STATE_READ))
        {
            udp_completed (udp, endpoint, UDP_STATUS_SUCCESS);
        }

        udp_setup_read (udp, endpoint);

        if (!udp->request_handler
            || !udp->request_handler (udp->request_handler_arg, &udp->setup))
            udp_stall (udp, UDP_EP_CONTROL);
    }

    // STALL sent
    if (ISSET (status, AT91C_UDP_STALLSENT))
    {
        // This interrupt occurs as a result of setting FORCESTALL

        TRACE_INFO (UDP, "UDP:Stallsent%d\n", endpoint);
        
        // Acknowledge interrupt
        udp_ep_flag_clear (pUDP, endpoint, AT91C_UDP_STALLSENT);
        
        // If the endpoint is not halted, clear the stall condition
        if (pep->state != UDP_EP_STATE_HALTED)
            udp_ep_flag_clear (pUDP, endpoint, AT91C_UDP_FORCESTALL);
    }
}


/**
 * Send stall condition
 * \param   endpoint    Endpoint where to send stall condition
 * 
 */
void 
udp_stall (udp_t udp, udp_ep_t endpoint)
{
    AT91PS_UDP pUDP = udp->pUDP;
    udp_ep_info_t *pep = &udp->eps[endpoint];

    if (pep->state != UDP_EP_STATE_IDLE) 
        return;

    TRACE_INFO (UDP, "UDP:Stall%d\n", endpoint);

    udp_ep_flag_set (pUDP, endpoint, AT91C_UDP_FORCESTALL);
}


/**
 * Handle halt feature request
 * 
 * Set or clear a halt feature request for given endpoint.
 * 
 * \param   endpoint    Endpoint to handle
 * \param   halt        Non zero to halt, zero to unhalt
 * \return  Current endpoint halt status
 * 
 */
bool
udp_halt (udp_t udp, udp_ep_t endpoint, bool halt)
{
    AT91PS_UDP pUDP = udp->pUDP;
    udp_ep_info_t *pep;

    // Mask endpoint number, direction bit is not used
    // see UDP v2.0 chapter 9.3.4
    endpoint &= 0x0F;  

    pep = &udp->eps[endpoint];  
    
    // Clear the halt feature of the endpoint if it is enabled
    if (!halt)
    {
        TRACE_INFO (UDP, "UDP:Unhalt%d\n", endpoint);

        pep->state = UDP_EP_STATE_IDLE;

        // Clear FORCESTALL flag
        udp_ep_flag_clear (pUDP, endpoint, AT91C_UDP_FORCESTALL);

        // Reset endpoint FIFOs, beware this is a 2 step operation
        SET (pUDP->UDP_RSTEP, 1 << endpoint);
        CLEAR (pUDP->UDP_RSTEP, 1 << endpoint);
    }
    // Set the halt feature on the endpoint if it is not already enabled
    // and the endpoint is not disabled
    else if ((pep->state != UDP_EP_STATE_HALTED)
             && (pep->state != UDP_EP_STATE_DISABLED))
    {

        TRACE_INFO (UDP, "UDP:Halt%d\n", endpoint);

        // Abort the current transfer if necessary
        udp_completed (udp, endpoint, UDP_STATUS_ABORTED);

        // Put endpoint into halt state
        udp_ep_flag_set (pUDP, endpoint, AT91C_UDP_FORCESTALL);
        pep->state = UDP_EP_STATE_HALTED;

        // Enable the endpoint interrupt
        SET (pUDP->UDP_IER, 1 << endpoint);
    }
    
    // Return the endpoint halt status
    return pep->state == UDP_EP_STATE_HALTED;
}


bool
udp_idle_p (udp_t udp, udp_ep_t endpoint)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];

    return pep->state == UDP_EP_STATE_IDLE;
}


bool
udp_halt_p (udp_t udp, udp_ep_t endpoint)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];

    return pep->state == UDP_EP_STATE_HALTED;
}


/**
 * Sets or unsets the device address
 *
 */ 
void
udp_address_set (void *arg, udp_transfer_t *ptransfer __unused__)
{
    udp_t udp = arg;
    AT91PS_UDP pUDP = udp->pUDP;
    unsigned int address = udp->setup.value;

    TRACE_DEBUG (UDP, "UDP:SetAddr 0x%2x\n", address);

    // Set address
    SET (pUDP->UDP_FADDR, AT91C_UDP_FEN | address);

    if (address == 0) 
    {
        // Inform UDP that out of address state
        SET (pUDP->UDP_GLBSTATE, 0);

        // Device returns to the Default state.  Is this a valid
        // transition?
        udp->state = UDP_STATE_DEFAULT;
    }
    else
    {
        // Inform UDP that in Address state.  Note, we must have achieved
        // the Status IN transaction of the control transfer before
        // doing this, i.e., once the TXCOMP flag in the UDP_CSR[0] register
        // has been received and cleared.
        SET (pUDP->UDP_GLBSTATE, AT91C_UDP_FADDEN);

        // The device enters the Address state
        udp->state = UDP_STATE_ADDRESS;
    }
}


/**
 * Changes the device state from Address to Configured, or from
 * Configured to Address.
 * 
 * This method directly accesses the last received setup packet to
 * decide what to do.
 * 
 */
void 
udp_configuration_set (void *arg, udp_transfer_t *ptransfer __unused__)
{
    udp_t udp = arg;
    AT91PS_UDP pUDP = udp->pUDP;
    unsigned int config = udp->setup.value;
    unsigned int ep;

    TRACE_DEBUG (UDP, "UDP:SetCfg %d\n", config);

    // Check the request
    if (config != 0)
    {
        // TODO, if have multiple configurations, use selected one

        // Enter configured state
        udp->state = UDP_STATE_CONFIGURED;
        SET (pUDP->UDP_GLBSTATE, AT91C_UDP_CONFG);

        // Configure endpoints
        for (ep = 1; ep < UDP_EP_NUM; ep++)
            udp_endpoint_configure (udp, ep);
    }
    else
    {
        // Go back to Address state
        udp->state = UDP_STATE_ADDRESS;
        SET (pUDP->UDP_GLBSTATE, AT91C_UDP_FADDEN);

        // For each endpoint, if it is enabled, disable it
        for (ep = 1; ep < UDP_EP_NUM; ep++)
        {
            udp_completed (udp, ep, UDP_STATUS_RESET);
            udp->eps[ep].state = UDP_EP_STATE_DISABLED;
        }
    }
}


void 
udp_control_write (udp_t udp, const void *buffer, udp_size_t length)
{
    AT91PS_UDP pUDP = udp->pUDP;
    uint32_t cpt;
    AT91_REG csr;
    const char *data = buffer;

    do 
    {
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
        do 
        {
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


void
udp_control_gobble (udp_t udp)
{
    AT91PS_UDP pUDP = udp->pUDP;

    /* I'm not too sure what the point of this is!.  */
    while (! (pUDP->UDP_CSR[0] & AT91C_UDP_RX_DATA_BK0));

    pUDP->UDP_CSR[0] &= ~AT91C_UDP_RX_DATA_BK0;
}


bool
udp_read_ready_p (udp_t udp)
{
    AT91PS_UDP pUDP = udp->pUDP;
    uint32_t rx_bank = udp->rx_bank;

    if (udp->rx_bytes)
        return 1;

    if (! udp_configured_p (udp))
        return 0;

    if (! (pUDP->UDP_CSR[UDP_EP_OUT] & rx_bank))
        return 0;
    
    return (pUDP->UDP_CSR[UDP_EP_OUT] >> 16) != 0;
}


udp_size_t
udp_read (udp_t udp, void *buffer, udp_size_t length)
{
    AT91PS_UDP pUDP = udp->pUDP;
    unsigned int rx_bytes;
    unsigned int total;
    uint32_t rx_bank = udp->rx_bank;
    uint8_t *data;

    data = buffer;
    total = 0;

    while (length)
    {
        if (udp->rx_bytes)
        {
            rx_bytes = MIN (udp->rx_bytes, length);
            length -= rx_bytes;
            udp->rx_bytes -= rx_bytes;

            /* Transfer data from FIFO.  */
            while (rx_bytes--)
                data[total++] = pUDP->UDP_FDR[UDP_EP_OUT];

            if (!udp->rx_bytes)
            {
                /* Indicate finished reading current bank.  */
                pUDP->UDP_CSR[UDP_EP_OUT] &= ~rx_bank;
                
                /* Switch to other bank.  */
                if (rx_bank == AT91C_UDP_RX_DATA_BK0)
                    rx_bank = AT91C_UDP_RX_DATA_BK1;
                else
                    rx_bank = AT91C_UDP_RX_DATA_BK0;
            }
        }

        if (!length)
            break;

        if (! udp_configured_p (udp))
            break;

        if (pUDP->UDP_CSR[UDP_EP_OUT] & rx_bank) 
        {
            /* It appears that the received byte count is not
               decremented after reads from the FIFO so we keep our
               own count.  */
            udp->rx_bytes = pUDP->UDP_CSR[UDP_EP_OUT] >> 16;
        }
    }
    udp->rx_bank = rx_bank;
    return total;
}


udp_size_t
udp_write (udp_t udp, const void *buffer, udp_size_t length)
{
    AT91PS_UDP pUDP = udp->pUDP;
    unsigned int tx_bytes = 0;
    unsigned int total;
    const uint8_t *data;

    if (! udp_configured_p (udp))
        return 0;

    data = buffer;
    total = 0;

    tx_bytes = MIN (length, UDP_EP_IN_SIZE);
    length -= tx_bytes;
    total += tx_bytes;

    while (tx_bytes--) 
        pUDP->UDP_FDR[UDP_EP_IN] = *data++;

    pUDP->UDP_CSR[UDP_EP_IN] |= AT91C_UDP_TXPKTRDY;

    while (length)
    {
        // Fill the second bank
        tx_bytes = MIN (length, UDP_EP_IN_SIZE);
        total += tx_bytes;
        length -= tx_bytes;
        while (tx_bytes--)
            pUDP->UDP_FDR[UDP_EP_IN] = *data++;

        // Wait for the the first bank to be sent
        while (! (pUDP->UDP_CSR[UDP_EP_IN] & AT91C_UDP_TXCOMP))
            if (! udp_configured_p (udp))
                return total;

        pUDP->UDP_CSR[UDP_EP_IN] &= ~AT91C_UDP_TXCOMP;
        while (pUDP->UDP_CSR[UDP_EP_IN] & AT91C_UDP_TXCOMP);
        pUDP->UDP_CSR[UDP_EP_IN] |= AT91C_UDP_TXPKTRDY;
    }

    // Wait for the end of transfer
    while (! (pUDP->UDP_CSR[UDP_EP_IN] & AT91C_UDP_TXCOMP))
        if (! udp_configured_p (udp))
            return total;

    pUDP->UDP_CSR[UDP_EP_IN] &= ~AT91C_UDP_TXCOMP;
    while (pUDP->UDP_CSR[UDP_EP_IN] & AT91C_UDP_TXCOMP)
        continue;

    return total;
}


/* Signal the host by pulling D+ high (drive a K state).  */
static void
udp_signal (udp_t udp __unused__)
{
#ifdef UDP_PIO_PULLUP
    /* Enable UDP PullUp (UDP_DP_PUP) : enable and clear of the
       corresponding PIO.  Set in PIO mode and configure as
       output.  */
    pio_config (UDP_PIO_PULLUP, PIO_OUTPUT);

    /* Set low to enable pullup (driven by p-channel MOSFET connected
       to 3V3).  */
    pio_output_low (UDP_PIO_PULLUP);
#endif
}


/* Check if VBUS connected.  */
static bool
udp_attached_p (udp_t udp __unused__)
{
#ifdef USB_PIO_DETECT
    return pio_input_get (USB_PIO_DETECT) != 0;
#else
    /* Assume that we are connected.  */
    return 1;
#endif
}


/**
 * Enable UDP device
 */
static void
udp_enable (udp_t udp)
{
    unsigned int i;

    // Set the PLL USB divider
    AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1 ;
    // Enables the 48MHz USB clock UDPCK and System Peripheral USB clock
    AT91C_BASE_PMC->PMC_SCER |= AT91C_PMC_UDP;
    AT91C_BASE_PMC->PMC_PCER |= (1 << AT91C_ID_UDP);

    // Init data banks
    for (i = 0; i < UDP_EP_NUM; i++)
        udp->eps[i].bank = AT91C_UDP_RX_DATA_BK0;

    TRACE_INFO (UDP, "UDP:Enabled\n");
}


/**
 * Disable UDP device
 */
static void
udp_disable (udp_t udp)
{
    AT91PS_UDP pUDP = udp->pUDP;
    AT91PS_AIC pAIC = AT91C_BASE_AIC;

#ifdef UDP_PIO_PULLUP
    // Configure UDP pullup.
    pio_config (UDP_PIO_PULLUP, PIO_OUTPUT);
    /* Set high to disable pullup.  */
    pio_output_high (UDP_PIO_PULLUP);
#endif

    // Disable the 48MHz USB clock UDPCK and System Peripheral USB Clock
    AT91C_BASE_PMC->PMC_SCDR |= AT91C_PMC_UDP;
    AT91C_BASE_PMC->PMC_PCDR |= (1 << AT91C_ID_UDP);

    // Disable the interrupt on the interrupt controller
    pAIC->AIC_IDCR |= 1 << AT91C_ID_UDP;

    // Disable all UDP interrupts
    pUDP->UDP_IDR = 0;

    // Disable UDP transceiver
    pUDP->UDP_TXVC |= AT91C_UDP_TXVDIS;

    TRACE_INFO (UDP, "UDP:Disabled\n");
}


/**
 * UDP check bus status
 * 
 * This routine enables/disables the UDP module by monitoring
 * the USB power signal.
 * 
 */
static void
udp_bus_status_check (udp_t udp)
{
    AT91PS_UDP pUDP = udp->pUDP;

    if (udp_attached_p (udp))
    {
        //  If UDP is deactivated enable it
        if (udp->state == UDP_STATE_NOT_POWERED)
        {
            udp->state = UDP_STATE_ATTACHED;

            udp_enable (udp);

            /* Signal the host by pulling D+ high.  This might be pulled high
               with an external 1k5 resistor.  */
            udp_signal (udp);            

            // Clear all UDP interrupts
            pUDP->UDP_ICR = 0;

            irq_config (AT91C_ID_UDP, 7, 
                        AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE, udp_irq_handler);
            
            irq_enable (AT91C_ID_UDP);

            // Enable UDP peripheral interrupts
            pUDP->UDP_IER = AT91C_UDP_ENDBUSRES | AT91C_UDP_RMWUPE
                | AT91C_UDP_RXSUSP;

            udp->state = UDP_STATE_POWERED;
        }   
    }
    else
    {
        if (udp->state != UDP_STATE_NOT_POWERED)
        {
            udp_disable (udp);
            udp->state = UDP_STATE_NOT_POWERED;
        }         
    }
}


void
udp_poll (udp_t udp __unused__)
{
    udp_bus_status_check (udp);

#if 0
    /* Perhaps could poll interrupts here.  */

    if (!udp->setup.request)
        return;

    if (!udp->request_handler
        || !udp->request_handler (udp->request_handler_arg, &udp->setup))
        udp_stall (udp, UDP_EP_CONTROL);

    udp->setup.request = 0;
#endif
}


bool
udp_configured_p (udp_t udp)
{
    return udp->state == UDP_STATE_CONFIGURED;
}


bool
udp_awake_p (udp_t udp)
{
    return udp->state == UDP_STATE_POWERED;
}


void
udp_shutdown (void)
{
    udp_disable (&udp_dev);
    udp_dev.state = UDP_STATE_NOT_POWERED;
}


/**
 * UDP Protocol reset handler
 *
 * Called when a USB bus reset is received from the host.
 * 
 */
static void 
udp_bus_reset_handler (udp_t udp)
{
    AT91PS_UDP pUDP = udp->pUDP;

    TRACE_DEBUG (UDP, "UDP:Reset\n");

    // Clear UDP peripheral interrupt
    pUDP->UDP_ICR |= AT91C_UDP_ENDBUSRES;
    
    // Reset all endpoints
    pUDP->UDP_RSTEP  = ~0;
    pUDP->UDP_RSTEP  = 0;
    
    // Enable the UDP
    pUDP->UDP_FADDR = AT91C_UDP_FEN;
    
    udp_endpoint_configure (udp, 0);

    // Configure UDP peripheral interrupts, here only EP0 and bus reset
    pUDP->UDP_IER = AT91C_UDP_EPINT0 | AT91C_UDP_ENDBUSRES | AT91C_UDP_RXSUSP;
    
    // Flush and enable the suspend interrupt
    pUDP->UDP_ICR |= AT91C_UDP_WAKEUP | AT91C_UDP_RXRSM | AT91C_UDP_RXSUSP;
    
    // Enable UDP transceiver 
    pUDP->UDP_TXVC &= ~AT91C_UDP_TXVDIS;
}


udp_t udp_init (udp_request_handler_t request_handler, void *arg)
{
    udp_t udp = &udp_dev;

    udp->request_handler = request_handler;
    udp->request_handler_arg = arg;
    udp->setup.request = 0;

    udp->pUDP = AT91C_BASE_UDP;
    udp->configuration = 0;
    udp->connection = 0;
    udp->rx_bytes = 0;
    udp->rx_bank = AT91C_UDP_RX_DATA_BK0;
    udp->prev_state = UDP_STATE_NOT_POWERED;
    udp->state = UDP_STATE_NOT_POWERED;

    return udp;
}

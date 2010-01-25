#include "irq.h"
#include "trace.h"
#include "udp.h"


/* 
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
#define udp_ep_clr_flag(pInterface, endpoint, flags) { \
while (pInterface->UDP_CSR[endpoint] & (flags)) \
    pInterface->UDP_CSR[endpoint] &= ~ (flags); \
}

/** Set flags of UDP UDP_CSR register and waits for synchronization */
#define udp_ep_set_flag(pInterface, endpoint, flags) { \
while ((pInterface->UDP_CSR[endpoint] & (flags)) != (flags) ) \
    pInterface->UDP_CSR[endpoint] |= (flags); \
}



/* IN and OUT are referred to the host so we transmit on the IN endpoint
   and receive on the OUT endpoint.  */
enum {AT91C_EP_OUT = 1, AT91C_EP_IN = 2};


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


#define USB_CLEAR_FEATURE              0x01
#define USB_SET_FEATURE                0x03


static udp_dev_t udp_dev;

/**
 * Structure for endpoint transfer parameters
 * 
 */
typedef struct
{
    // Transfer descriptor
    char                    *pData;             //!< \brief Transfer descriptor
                                                //!< pointer to a buffer where
                                                //!< the data is read/stored
    unsigned int            dBytesRemaining;    //!< \brief Number of remaining
                                                //!< bytes to transfer
    unsigned int            dBytesBuffered;     //!< \brief Number of bytes
                                                //!< which have been buffered
                                                //!< but not yet transferred
    unsigned int            dBytesTransferred;  //!< \brief Number of bytes
                                                //!< transferred for the current
                                                //!< operation
    udp_callback_t          fCallback;          //!< \brief Callback to invoke
                                                //!< after the current transfer
                                                //!< is complete
    void                    *pArgument;         //!< \brief Argument to pass to
                                                //!< the callback function                                                
    // Hardware information
    unsigned int            wMaxPacketSize;     //!< \brief Maximum packet size
                                                //!< for this endpoint
    unsigned int            dFlag;              //!< \brief Hardware flag to
                                                //!< clear upon data reception
    unsigned char           dNumFIFO;           //!< \brief Number of FIFO
                                                //!< buffers defined for this
                                                //!< endpoint
    unsigned int   dState;                      //!< Endpoint internal state
} __packed__ s_usb_endpoint;




//! Device status, connected/disconnected etc.
static udp_dev_t udp_dev;
//! Struct for each endpoint holding endpoint parameters
static s_usb_endpoint udp_epdata[UDP_EP_NUM];
//! Pointers array to enpoint parameters
s_usb_endpoint *pEndpoint[UDP_EP_NUM] =
{
    &udp_epdata[0], 
    &udp_epdata[1], 
    &udp_epdata[2]
};


/** Interrupt mask */
#define ISR_MASK            0x00003FFF


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
udp_irq_service (void)
{
    udp_ep_t endpoint;
    udp_t udp = &udp_dev;
    AT91PS_UDP pUDP = udp->pUDP;

    // End interrupt if we are not attached to UDP bus
    if (ISCLEARED (udp->device_state, UDP_STATE_ATTACHED))
        return;

    unsigned int status = pUDP->UDP_ISR & pUDP->UDP_IMR & ISR_MASK;

    while (status != 0)
    {
        // Start Of Frame (SOF)
        if (ISSET (status, AT91C_UDP_SOFINT))
        {
            TRACE_INFO (UDP, "UDP:SOF\n");
            // Acknowledge interrupt
            SET (pUDP->UDP_ICR, AT91C_UDP_SOFINT);
            CLEAR (status, AT91C_UDP_SOFINT);
        }        

        // Suspend
        if (ISSET (status, AT91C_UDP_RXSUSP))
        {
            TRACE_INFO (UDP, "UDP:Susp\n");
           
            if (ISCLEARED (udp->device_state, UDP_STATE_SUSPENDED))
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
                SET (udp->device_state, UDP_STATE_SUSPENDED);
                // Disable transceiver
                SET (pUDP->UDP_TXVC, AT91C_UDP_TXVDIS);
                // Disable master clock
                AT91C_BASE_PMC->PMC_PCDR |= (1 << AT91C_ID_UDP);
                // Disable peripheral clock for UDP
                AT91C_BASE_PMC->PMC_SCDR |= AT91C_PMC_UDP;
            }            
        }
        // Resume
        else if (ISSET (status, AT91C_UDP_WAKEUP) 
                 || ISSET (status, AT91C_UDP_RXRSM))
        {
            TRACE_INFO (UDP, "UDP:Resm\n");

            // The device enters Configured state
            //      MCK + UDPCK must be on
            //      Pull-Up must be connected
            //      Transceiver must be enabled
            // Powered state
            // Enable master clock
            AT91C_BASE_PMC->PMC_PCER |= (1 << AT91C_ID_UDP);
            // Enable peripheral clock for UDP
            AT91C_BASE_PMC->PMC_SCER |= AT91C_PMC_UDP;

            // Default state
            if (ISSET (udp->device_state, UDP_STATE_DEFAULT))
            {
                // Enable transceiver
                CLEAR (pUDP->UDP_TXVC, AT91C_UDP_TXVDIS);
            }

            CLEAR (udp->device_state, UDP_STATE_SUSPENDED);

            SET (pUDP->UDP_ICR, AT91C_UDP_WAKEUP | AT91C_UDP_RXRSM
                 | AT91C_UDP_RXSUSP);
            SET (pUDP->UDP_IDR, AT91C_UDP_WAKEUP | AT91C_UDP_RXRSM);
        }        
        // End of bus reset
        else if (ISSET (status, AT91C_UDP_ENDBUSRES))
        {
            TRACE_INFO (UDP, "UDP:EoBres\n");

            // Initialize UDP peripheral device
            udp_bus_reset_handler (udp);

            // Flush and enable the Suspend interrupt
            SET (pUDP->UDP_ICR, AT91C_UDP_WAKEUP 
                 | AT91C_UDP_RXRSM | AT91C_UDP_RXSUSP);

            // Acknowledge end of bus reset interrupt
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
        if (ISCLEARED (udp->device_state, UDP_STATE_DEFAULT))
        {
            status &= AT91C_UDP_ENDBUSRES | AT91C_UDP_SOFINT;
        }
        
    }   // end while status != 0
}


/**
 * End transfer
 * 
 * End transfer of given endpoint.
 * 
 * \param   endpoint    Endpoint where to end transfer
 * \param   bStatus     Status code returned by the transfer operation
 * 
 */
static __inline__
void udp_end_of_transfer (udp_t udp __unused__, 
                          udp_ep_t endpoint, char bStatus)
{
    if ((pEndpoint[endpoint]->dState == UDP_EP_STATE_WRITE)
        || (pEndpoint[endpoint]->dState == UDP_EP_STATE_READ)) 
    {
        TRACE_DEBUG (UDP, "UDP:EoT %d\n",
                     pEndpoint[endpoint]->dBytesTransferred);

        // Endpoint returns in Idle state
        pEndpoint[endpoint]->dState = UDP_EP_STATE_IDLE;
        
        // Invoke callback if present
        if (pEndpoint[endpoint]->fCallback != 0) 
        {
            pEndpoint[endpoint]->fCallback
                ((unsigned int) pEndpoint[endpoint]->pArgument, 
                 (unsigned int) bStatus, 
                 pEndpoint[endpoint]->dBytesTransferred, 
                 pEndpoint[endpoint]->dBytesRemaining
                 + pEndpoint[endpoint]->dBytesBuffered);
        }
    }
}


/**
 * Configure specific endpoint
 * 
 * \param   endpoint Endpoint to configure
 * 
 */
void
udp_configure_endpoint (udp_t udp, udp_ep_t endpoint)
{
    AT91PS_UDP pUDP = udp->pUDP;

    // Abort the current transfer if the endpoint was configured and in
    // Write or Read state
    if ((pEndpoint[endpoint]->dState == UDP_EP_STATE_READ)
        || (pEndpoint[endpoint]->dState == UDP_EP_STATE_WRITE))
    {
        udp_end_of_transfer (udp, endpoint, UDP_STATUS_RESET);
    }

    // Enter IDLE state
    pEndpoint[endpoint]->dState = UDP_EP_STATE_IDLE;

    // Reset endpoint transfer descriptor
    pEndpoint[endpoint]->pData = 0;
    pEndpoint[endpoint]->dBytesRemaining = 0;
    pEndpoint[endpoint]->dBytesTransferred = 0;
    pEndpoint[endpoint]->dBytesBuffered = 0;
    pEndpoint[endpoint]->fCallback = 0;
    pEndpoint[endpoint]->pArgument = 0;

    // Reset Endpoint FIFOs
    SET (pUDP->UDP_RSTEP, 1 << endpoint);
    CLEAR (pUDP->UDP_RSTEP, 1 << endpoint);

    switch (endpoint)
    {
    case UDP_EP_CONTROL:
        pEndpoint[0]->wMaxPacketSize = UDP_EP_CONTROL_SIZE;
        pEndpoint[0]->dNumFIFO = 1;
        pUDP->UDP_CSR[0] = AT91C_UDP_EPTYPE_CTRL | AT91C_UDP_EPEDS;
        break;
    
    case UDP_EP_OUT:
        pEndpoint[1]->wMaxPacketSize = UDP_EP_OUT_SIZE;
        pEndpoint[1]->dNumFIFO = 2;
        pUDP->UDP_CSR[1] = AT91C_UDP_EPTYPE_BULK_OUT | AT91C_UDP_EPEDS;
        break;

    case UDP_EP_IN:
        pEndpoint[2]->wMaxPacketSize = UDP_EP_IN_SIZE;
        pEndpoint[2]->dNumFIFO = 2;
        pUDP->UDP_CSR[2] = AT91C_UDP_EPTYPE_BULK_IN | AT91C_UDP_EPEDS;         
        break;
    }

    TRACE_DEBUG (UDP, "UDP:CfgEp%d\n", endpoint);
}


/**
 * Sets or unsets the device address
 *
 */ 
void
udp_set_address (udp_t udp, udp_setup_t *setup)
{
    AT91PS_UDP pUDP = udp->pUDP;
    unsigned int address = setup->value;

    TRACE_DEBUG (UDP, "UDP:SetAddr %d\n", address);

    // Set address
    SET (pUDP->UDP_FADDR, AT91C_UDP_FEN | address);

    if (address == 0) 
    {
        SET (pUDP->UDP_GLBSTATE, 0);

        // Device enters the Default state
        CLEAR (udp->device_state, UDP_STATE_ADDRESS);
    }
    else
    {
        SET (pUDP->UDP_GLBSTATE, AT91C_UDP_FADDEN);

        // The device enters the Address state
        SET (udp->device_state, UDP_STATE_ADDRESS);
    }
}


/**
 * Clear transmission status flag
 * 
 * This routine clears the transmission status flag and
 * swaps banks for dual FIFO endpoints.
 * 
 * \param   endpoint    Endpoint where to clear flag
 * 
 */
static void 
udp_clear_rx_flag (udp_t udp, udp_ep_t endpoint)
{
    AT91PS_UDP pUDP = udp->pUDP;

    // Clear flag
    udp_ep_clr_flag (pUDP, endpoint, pEndpoint[endpoint]->dFlag);

    // Swap banks
    if (pEndpoint[endpoint]->dFlag == AT91C_UDP_RX_DATA_BK0) 
    {
        if (pEndpoint[endpoint]->dNumFIFO > 1)
        {
            // Swap bank if in dual-fifo mode
            pEndpoint[endpoint]->dFlag = AT91C_UDP_RX_DATA_BK1;
        }
    }
    else 
    {
        pEndpoint[endpoint]->dFlag = AT91C_UDP_RX_DATA_BK0;
    }    
}


/**
 * Writes data to UDP FIFO
 * 
 * This routine writes data from specific buffer to given endpoint FIFO.
 * 
 * \param   endpoint    Endpoint to write data
 * \return  Number of bytes write
 * 
 */
static unsigned int
udp_write_payload (udp_t udp, udp_ep_t endpoint)
{
    AT91PS_UDP pUDP = udp->pUDP;
    unsigned int bytes;
    unsigned int ctr;

    // Get the number of bytes to send
    bytes = MIN (pEndpoint[endpoint]->wMaxPacketSize, 
                 pEndpoint[endpoint]->dBytesRemaining);

    // Transfer one packet in the FIFO buffer
    for (ctr = 0; ctr < bytes; ctr++)
    {
        pUDP->UDP_FDR[endpoint] = * (pEndpoint[endpoint]->pData);
        pEndpoint[endpoint]->pData++;
    }
    // track status of byte transmission
    pEndpoint[endpoint]->dBytesBuffered += bytes;
    pEndpoint[endpoint]->dBytesRemaining -= bytes;

    return bytes;
}


/**
 * Send data packet via given endpoint
 * 
 * Send a data packet with specific size via given endpoint.
 * 
 * \param   endpoint    Endpoint to send data through
 * \param   pData       Pointer to data buffer
 * \param   len         Packet size
 * \param   fCallback   Optional callback to invoke when the read finishes
 * \param   pArgument   Optional callback argument 
 * \return  Operation result code
 * 
 */
udp_status_t
udp_write_async (udp_t udp __unused__, const void *pData, 
                 unsigned int len, 
                 udp_callback_t fCallback, 
                 void *pArgument)
{
    AT91PS_UDP pUDP = udp->pUDP;
    udp_ep_t endpoint = UDP_EP_IN;

    // Check that the endpoint is in Idle state
    if (pEndpoint[endpoint]->dState != UDP_EP_STATE_IDLE) 
        return UDP_STATUS_LOCKED;

    TRACE_INFO (UDP, "UDP:Write%d %d\n", endpoint, len);

    // Setup the transfer descriptor
    pEndpoint[endpoint]->pData = (char *) pData;
    pEndpoint[endpoint]->dBytesRemaining = len;
    pEndpoint[endpoint]->dBytesBuffered = 0;
    pEndpoint[endpoint]->dBytesTransferred = 0;
    pEndpoint[endpoint]->fCallback = fCallback;
    pEndpoint[endpoint]->pArgument = pArgument;

    // Send one packet
    pEndpoint[endpoint]->dState = UDP_EP_STATE_WRITE;
    udp_write_payload (udp, endpoint);
    udp_ep_set_flag (pUDP, endpoint, AT91C_UDP_TXPKTRDY);

    // If double buffering is enabled and there is data remaining, 
    // prepare another packet
    if ((pEndpoint[endpoint]->dNumFIFO > 1) 
         && (pEndpoint[endpoint]->dBytesRemaining > 0))
        udp_write_payload (udp, endpoint);

    // Enable interrupt on endpoint
    SET (pUDP->UDP_IER, 1 << endpoint);
    
    return UDP_STATUS_SUCCESS;
}


/**
 * Read data from UDP FIFO
 * 
 * This routine reads data from given endpoint FIFO with given size to specific buffer.
 * 
 * \param   endpoint    Endpoint to read data from
 * \param   packetsize  Maximum size of packet to receive
 * \return  Number of bytes read.
 * 
 */
static unsigned int 
udp_read_payload (udp_t udp, udp_ep_t endpoint, unsigned int packetsize)
{
    AT91PS_UDP pUDP = udp->pUDP;
    unsigned int bytes;
    unsigned int ctr;

    // Get number of bytes to retrieve
    bytes = MIN (pEndpoint[endpoint]->dBytesRemaining, packetsize);

    // Retrieve packet
    for (ctr = 0; ctr < bytes; ctr++) 
    {
        *pEndpoint[endpoint]->pData = (char) pUDP->UDP_FDR[endpoint];
        pEndpoint[endpoint]->pData++;
    }
    // track bytes transmission status
    pEndpoint[endpoint]->dBytesRemaining -= bytes;
    pEndpoint[endpoint]->dBytesTransferred += bytes;
    pEndpoint[endpoint]->dBytesBuffered += packetsize - bytes;

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
 * \param   fCallback   Optional callback to invoke when the read finishes
 * \param   pArgument   Optional callback argument 
 * \return  Operation result code
 * 
 */
udp_status_t
udp_read_async (udp_t udp __unused__, void *pData, 
                unsigned int len, 
                udp_callback_t fCallback, 
                void *pArgument)
{
    AT91PS_UDP pUDP = udp->pUDP;
    udp_ep_t endpoint = UDP_EP_OUT;

    // Check that the endpoint is in Idle state
    if (pEndpoint[endpoint]->dState != UDP_EP_STATE_IDLE)
        return UDP_STATUS_LOCKED;

    TRACE_INFO (UDP, "UDP:Read%d %d\n", endpoint, len);

    // Endpoint enters Read state
    pEndpoint[endpoint]->dState = UDP_EP_STATE_READ;

    // Set the transfer descriptor
    pEndpoint[endpoint]->pData = (char *) pData;
    pEndpoint[endpoint]->dBytesRemaining = len;
    pEndpoint[endpoint]->dBytesBuffered = 0;
    pEndpoint[endpoint]->dBytesTransferred = 0;
    pEndpoint[endpoint]->fCallback = fCallback;
    pEndpoint[endpoint]->pArgument = pArgument;
    
    // Enable interrupt on endpoint
    SET (pUDP->UDP_IER, 1 << endpoint);
    
    return UDP_STATUS_SUCCESS;
}



static void
udp_setup (udp_t udp, udp_ep_t endpoint)
{
    AT91PS_UDP pUDP = udp->pUDP;
    udp_setup_t setup;

    // Get request parameters
    setup.type = pUDP->UDP_FDR[0];
    setup.request = pUDP->UDP_FDR[0];
    setup.value = pUDP->UDP_FDR[0] & 0xFF;
    setup.value |= (pUDP->UDP_FDR[0] << 8);
    setup.index = pUDP->UDP_FDR[0] & 0xFF;
    setup.index |= (pUDP->UDP_FDR[0] << 8);
    setup.length = pUDP->UDP_FDR[0] & 0xFF;
    setup.length |= (pUDP->UDP_FDR[0] << 8);
    
    // Set the DIR bit before clearing RXSETUP in Control IN sequence
    if (setup.type & 0x80)
        udp_ep_set_flag (pUDP, endpoint, AT91C_UDP_DIR);
    
    // Acknowledge interrupt
    udp_ep_clr_flag (pUDP, endpoint, AT91C_UDP_RXSETUP);
    
    if (!udp->request_handler
        || !udp->request_handler (udp->request_handler_arg, &setup))
        udp_control_stall (udp);
}

    
/**
 * UDP endpoint handler
 *
 * Service routine for all endpoint communication
 *  
 * \param   endpoint    Endpoint for which to handle communication
 * 
 * Called from UDP interrupt service routine in case of
 * endpoint interrupt.
 * 
 */
static void 
udp_endpoint_handler (udp_t udp, udp_ep_t endpoint)
{
    AT91PS_UDP pUDP = udp->pUDP;    
    unsigned int status = pUDP->UDP_CSR[endpoint];
    
    // Handle interrupts
    // IN packet sent
    if (ISSET (status, AT91C_UDP_TXCOMP))
    {
        // Check that endpoint was in Write state
        if (pEndpoint[endpoint]->dState == UDP_EP_STATE_WRITE)
        {

            // End of transfer ?
            if ((pEndpoint[endpoint]->dBytesBuffered 
                 < pEndpoint[endpoint]->wMaxPacketSize)
                || (!ISCLEARED (status, AT91C_UDP_EPTYPE)
                    && (pEndpoint[endpoint]->dBytesRemaining == 0)
                    && (pEndpoint[endpoint]->dBytesBuffered
                        == pEndpoint[endpoint]->wMaxPacketSize)))
            {
                TRACE_DEBUG (UDP, "UDP:Wr E%d %d\n", endpoint,
                            pEndpoint[endpoint]->dBytesBuffered);

                pEndpoint[endpoint]->dBytesTransferred 
                    += pEndpoint[endpoint]->dBytesBuffered;
                pEndpoint[endpoint]->dBytesBuffered = 0;

                // Disable interrupt if this is not a control endpoint
                if (!ISCLEARED (status, AT91C_UDP_EPTYPE))
                    SET (pUDP->UDP_IDR, 1 << endpoint);

                udp_end_of_transfer (udp, endpoint, UDP_STATUS_SUCCESS);
            }
            else
            {

                // Transfer remaining data
                TRACE_DEBUG (UDP, "UDP:Wr E%d +%d\n", endpoint,
                            pEndpoint[endpoint]->dBytesBuffered);

                pEndpoint[endpoint]->dBytesTransferred 
                    += pEndpoint[endpoint]->wMaxPacketSize;
                pEndpoint[endpoint]->dBytesBuffered
                    -= pEndpoint[endpoint]->wMaxPacketSize;

                // Send next packet
                if (pEndpoint[endpoint]->dNumFIFO == 1)
                {
                    // No double buffering
                    udp_write_payload (udp, endpoint);
                    udp_ep_set_flag (pUDP, endpoint, AT91C_UDP_TXPKTRDY);
                }
                else
                {
                    // Double buffering
                    udp_ep_set_flag (pUDP, endpoint, AT91C_UDP_TXPKTRDY);
                    udp_write_payload (udp, endpoint);
                }
            }
        }
                
        // Acknowledge interrupt
        udp_ep_clr_flag (pUDP, endpoint, AT91C_UDP_TXCOMP);
    }

    // OUT packet received
    if (ISSET (status, AT91C_UDP_RX_DATA_BK0) 
        || ISSET (status, AT91C_UDP_RX_DATA_BK1))
    {
        // Check that the endpoint is in Read state
        if (pEndpoint[endpoint]->dState != UDP_EP_STATE_READ)
        {
            // Endpoint is NOT in Read state
            if (ISCLEARED (status, AT91C_UDP_EPTYPE)
                && ISCLEARED (status, 0xFFFF0000))
            {
                // Control endpoint, 0 bytes received
                // Acknowledge the data and finish the current transfer
                TRACE_INFO (UDP, "UDP:Ack\n");
                udp_clear_rx_flag (udp, endpoint);

                udp_end_of_transfer (udp, endpoint, UDP_STATUS_SUCCESS);
            }
            else if (ISSET (status, AT91C_UDP_FORCESTALL))
            {
                // Non-control endpoint
                // Discard stalled data
                TRACE_INFO (UDP, "UDP:Disc\n");
                udp_clear_rx_flag (udp, endpoint);
            }
            else
            {
                // Non-control endpoint
                // Nak data
                TRACE_INFO (UDP, "UDP:Nak\n");
                SET (pUDP->UDP_IDR, 1 << endpoint);
            }
        }
        else
        {
            // Endpoint is in Read state
            // Retrieve data and store it into the current transfer buffer
            unsigned short packetsize = (unsigned short) (status >> 16);

            TRACE_DEBUG (UDP, "UDP:Rd E%d %d\n", endpoint, packetsize);

            udp_read_payload (udp, endpoint, packetsize);

            udp_clear_rx_flag (udp, endpoint);

            if ((pEndpoint[endpoint]->dBytesRemaining == 0)
                || (packetsize < pEndpoint[endpoint]->wMaxPacketSize))
            {
                // Disable interrupt if this is not a control endpoint
                if (!ISCLEARED (status, AT91C_UDP_EPTYPE))
                    SET (pUDP->UDP_IDR, 1 << endpoint);

                udp_end_of_transfer (udp, endpoint, UDP_STATUS_SUCCESS);
            }
        }
    }

    // SETUP packet received
    if (ISSET (status, AT91C_UDP_RXSETUP))
    {
        TRACE_DEBUG (UDP, "UDP:Setup\n");

        // If a transfer was pending, complete it
        // Handle the case where during the status phase of a control write
        // transfer, the host receives the device ZLP and ack it, but the ack
        // is not received by the device
        if ((pEndpoint[endpoint]->dState == UDP_EP_STATE_WRITE)
            || (pEndpoint[endpoint]->dState == UDP_EP_STATE_READ))
        {
            udp_end_of_transfer (udp, endpoint, UDP_STATUS_SUCCESS);
        }

        udp_setup (udp, endpoint);
    }

    // STALL sent
    if (ISSET (status, AT91C_UDP_STALLSENT))
    {
        TRACE_INFO (UDP, "UDP:Stall\n");
        
        // Acknowledge interrupt
        udp_ep_clr_flag (pUDP, endpoint, AT91C_UDP_STALLSENT);
        
        // If the endpoint is not halted, clear the stall condition
        if (pEndpoint[endpoint]->dState != UDP_EP_STATE_HALTED)
            udp_ep_clr_flag (pUDP, endpoint, AT91C_UDP_FORCESTALL);
    }
}


/**
 * Send stall condition
 * 
 * Send stall condition to given endpoint
 * 
 * \param   endpoint    Endpoint where to send stall condition
 * 
 */
void 
udp_stall (udp_t udp, udp_ep_t endpoint)
{
    AT91PS_UDP pUDP = udp->pUDP;

    // Check that endpoint is in Idle state
    if (pEndpoint[endpoint]->dState != UDP_EP_STATE_IDLE) 
        return;

    TRACE_INFO (UDP, "UDP:Stall%d\n", endpoint);

    udp_ep_set_flag (pUDP, endpoint, AT91C_UDP_FORCESTALL);
}


/**
 * Handle halt feature request
 * 
 * Set or clear a halt feature request for given endpoint.
 * 
 * \param   endpoint    Endpoint to handle
 * \param   request     Set/Clear feature flag
 * \return  Current endpoint halt status
 * 
 */
bool
udp_halt (udp_t udp, udp_ep_t endpoint, uint8_t request)
{
    AT91PS_UDP pUDP = udp->pUDP;

    // Mask endpoint number, direction bit is not used
    // see UDP v2.0 chapter 9.3.4
    endpoint &= 0x0F;    
    
    // Clear the Halt feature of the endpoint if it is enabled
    if (request == USB_CLEAR_FEATURE)
    {
        TRACE_INFO (UDP, "UDP:Unhalt%d\n", endpoint);

        // Return endpoint to Idle state
        pEndpoint[endpoint]->dState = UDP_EP_STATE_IDLE;

        // Clear FORCESTALL flag
        udp_ep_clr_flag (pUDP, endpoint, AT91C_UDP_FORCESTALL);

        // Reset endpoint FIFOs, beware this is a 2 step operation
        SET (pUDP->UDP_RSTEP, 1 << endpoint);
        CLEAR (pUDP->UDP_RSTEP, 1 << endpoint);
    }
    // Set the Halt feature on the endpoint if it is not already enabled
    // and the endpoint is not disabled
    else if ((request == USB_SET_FEATURE)
             && (pEndpoint[endpoint]->dState != UDP_EP_STATE_HALTED)
             && (pEndpoint[endpoint]->dState != UDP_EP_STATE_DISABLED))
    {

        TRACE_INFO (UDP, "UDP:Halt%d\n", endpoint);

        // Abort the current transfer if necessary
        udp_end_of_transfer (udp, endpoint, UDP_STATUS_ABORTED);

        // Put endpoint into Halt state
        udp_ep_set_flag (pUDP, endpoint, AT91C_UDP_FORCESTALL);
        pEndpoint[endpoint]->dState = UDP_EP_STATE_HALTED;

        // Enable the endpoint interrupt
        SET (pUDP->UDP_IER, 1 << endpoint);
    }
    
    // Return the endpoint halt status
    return pEndpoint[endpoint]->dState == UDP_EP_STATE_HALTED;
}


/**
 * Changes the device state from Address to Configured, or from
 * Configured to Address.
 * 
 * This method directly accesses the last received SETUP packet to
 * decide what to do.
 * 
 */
void 
udp_set_configuration (udp_t udp, udp_setup_t *setup)
{
    AT91PS_UDP pUDP = udp->pUDP;
    unsigned int config = setup->value;
    unsigned int ep;

    TRACE_DEBUG (UDP, "UDP:SetCfg %d\n", config);

    // Check the request
    if (config != 0)
    {
        // TODO, if have multiple configurations, use selected one

        // Enter Configured state
        SET (udp->device_state, UDP_STATE_CONFIGURED);
        SET (pUDP->UDP_GLBSTATE, AT91C_UDP_CONFG);

        // Configure endpoints
        for (ep = 0; ep < UDP_EP_NUM; ep++)
            udp_configure_endpoint (udp, ep);
    }
    else
    {
        // Go back to Address state
        CLEAR (udp->device_state, UDP_STATE_CONFIGURED);
        SET (pUDP->UDP_GLBSTATE, AT91C_UDP_FADDEN);

        // For each endpoint, if it is enabled, disable it
        // Control endpoint 0 is not disabled
        for (ep = 1; ep < UDP_EP_NUM; ep++)
        {
            udp_end_of_transfer (udp, ep, UDP_STATUS_RESET);
            pEndpoint[ep]->dState = UDP_EP_STATE_DISABLED;
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


void
udp_control_write_zlp (udp_t udp)
{
    AT91PS_UDP pUDP = udp->pUDP;

    pUDP->UDP_CSR[0] |= AT91C_UDP_TXPKTRDY;
    while (! (pUDP->UDP_CSR[0] & AT91C_UDP_TXCOMP))
        continue;

    pUDP->UDP_CSR[0] &= ~AT91C_UDP_TXCOMP;
    while (pUDP->UDP_CSR[0] & AT91C_UDP_TXCOMP)
        continue;
}


void
udp_control_stall (udp_t udp)
{
    AT91PS_UDP pUDP = udp->pUDP;

    pUDP->UDP_CSR[0] |= AT91C_UDP_FORCESTALL;
    while (! (pUDP->UDP_CSR[0] & AT91C_UDP_ISOERROR))
        continue;

    pUDP->UDP_CSR[0] &= ~(AT91C_UDP_FORCESTALL | AT91C_UDP_ISOERROR);
    while (pUDP->UDP_CSR[0] & (AT91C_UDP_FORCESTALL | AT91C_UDP_ISOERROR))
        continue;
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

    if (! (pUDP->UDP_CSR[AT91C_EP_OUT] & rx_bank))
        return 0;
    
    return (pUDP->UDP_CSR[AT91C_EP_OUT] >> 16) != 0;
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
                data[total++] = pUDP->UDP_FDR[AT91C_EP_OUT];

            if (!udp->rx_bytes)
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

        if (! udp_configured_p (udp))
            break;

        if (pUDP->UDP_CSR[AT91C_EP_OUT] & rx_bank) 
        {
            /* It appears that the received byte count is not
               decremented after reads from the FIFO so we keep our
               own count.  */
            udp->rx_bytes = pUDP->UDP_CSR[AT91C_EP_OUT] >> 16;
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
        pUDP->UDP_FDR[AT91C_EP_IN] = *data++;

    pUDP->UDP_CSR[AT91C_EP_IN] |= AT91C_UDP_TXPKTRDY;

    while (length)
    {
        // Fill the second bank
        tx_bytes = MIN (length, UDP_EP_IN_SIZE);
        total += tx_bytes;
        length -= tx_bytes;
        while (tx_bytes--)
            pUDP->UDP_FDR[AT91C_EP_IN] = *data++;

        // Wait for the the first bank to be sent
        while (! (pUDP->UDP_CSR[AT91C_EP_IN] & AT91C_UDP_TXCOMP))
            if (! udp_configured_p (udp))
                return total;

        pUDP->UDP_CSR[AT91C_EP_IN] &= ~AT91C_UDP_TXCOMP;
        while (pUDP->UDP_CSR[AT91C_EP_IN] & AT91C_UDP_TXCOMP);
        pUDP->UDP_CSR[AT91C_EP_IN] |= AT91C_UDP_TXPKTRDY;
    }

    // Wait for the end of transfer
    while (! (pUDP->UDP_CSR[AT91C_EP_IN] & AT91C_UDP_TXCOMP))
        if (! udp_configured_p (udp))
            return total;

    pUDP->UDP_CSR[AT91C_EP_IN] &= ~AT91C_UDP_TXCOMP;
    while (pUDP->UDP_CSR[AT91C_EP_IN] & AT91C_UDP_TXCOMP)
        continue;

    return total;
}


bool
udp_configured_p (udp_t udp)
{
    // Could poll interrupts here...

    // Check if device configured
    return ! ISCLEARED (udp->device_state, UDP_STATE_CONFIGURED);
}


void
udp_connect (udp_t udp __unused__)
{
   /* Connect pull-up, wait for configuration.  This does nothing if the
       pull-up is always connected.  */

#ifdef UDP_PIO_PULLUP
    // Enable UDP PullUp (UDP_DP_PUP) : enable and clear of the
    // corresponding PIO.  Set in PIO mode and configure as output.
    pio_config (UDP_PIO_PULLUP, PIO_OUTPUT);
    /* Set low to enable pullup.  */
    pio_output_low (UDP_PIO_PULLUP);
#endif
}


void
udp_shutdown (void)
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


static bool
udp_detect_p (udp_t udp __unused__)
{
#ifdef USB_PIO_DETECT
    return pio_input_get (USB_PIO_DETECT) != 0;
#else
    return 1;
#endif
}


/**
 * Enable UDP device
 * 
 * This routine enables the UDP peripheral device.
 * Function should never called manually!
 * 
 */
void udp_enable_device (udp_t udp __unused__)
{
    unsigned int i;

    // Set the PLL USB Divider
    AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1 ;
    // Enables the 48MHz USB clock UDPCK and System Peripheral USB Clock
    AT91C_BASE_PMC->PMC_SCER |= AT91C_PMC_UDP;
    AT91C_BASE_PMC->PMC_PCER |= (1 << AT91C_ID_UDP);

    // Init data banks
    for (i = 0; i < UDP_EP_NUM; i++)
        pEndpoint[i]->dFlag = AT91C_UDP_RX_DATA_BK0;

    // We are in attached state now
    SET (udp->device_state, UDP_STATE_ATTACHED);

    TRACE_INFO (UDP, "UDP:UDP enabled\n");
}


/**
 * Disable UDP device
 *
 * This routine disables the UDP peripheral device.
 * Function should never be called manually!
 * 
 */
void udp_disable_device (udp_t udp)
{
    AT91PS_UDP pUDP = udp->pUDP;
    AT91PS_AIC pAIC = AT91C_BASE_AIC;

#ifdef UDP_PIO_PULLUP
    // Enable UDP PullUp (UDP_DP_PUP) : enable and clear of the
    // corresponding PIO.  Set in PIO mode and configure as output.
    pio_config (UDP_PIO_PULLUP, PIO_OUTPUT);
    /* Set high to disable pullup.  */
    pio_output_high (UDP_PIO_PULLUP);
#endif

    // Disables the 48MHz USB clock UDPCK and System Peripheral USB Clock
    AT91C_BASE_PMC->PMC_SCDR |= AT91C_PMC_UDP;
    AT91C_BASE_PMC->PMC_PCDR |= (1 << AT91C_ID_UDP);

    // Disable the interrupt on the interrupt controller
    pAIC->AIC_IDCR |= 1 << AT91C_ID_UDP;
    pUDP->UDP_IDR = 0;                   // Disable all UDP interrupts
    pUDP->UDP_TXVC |= AT91C_UDP_TXVDIS;  // Disable UDP tranceiver

    udp->device_state = 0;
    SET (udp->device_state, UDP_STATE_DETACHED);

    TRACE_INFO (UDP, "UDP:UDP disabled\n");
}


/**
 * Disable UDP device by software
 *
 * This routine disables the UDP peripheral device
 * through software.
 * 
 */
void udp_soft_disable_device (udp_t udp)
{
    udp_disable_device (udp);   
}


/**
 * UDP Protocol reset handler
 *
 * A USB bus reset is received from the host.
 * 
 */
static void 
udp_bus_reset_handler (udp_t udp)
{
    AT91PS_UDP pUDP = udp->pUDP;

    TRACE_DEBUG (UDP, "UDP:Rst\n");
    // Clear UDP peripheral interrupt
    pUDP->UDP_ICR |= AT91C_UDP_ENDBUSRES;
    
    // Reset all endpoints
    pUDP->UDP_RSTEP  = ~0;
    pUDP->UDP_RSTEP  = 0;
    
    // Enable the UDP
    pUDP->UDP_FADDR = AT91C_UDP_FEN;
    
    udp_configure_endpoint (udp, 0);

    // Configure UDP peripheral interrupts, here only EP0 and bus reset
    pUDP->UDP_IER = AT91C_UDP_EPINT0 | AT91C_UDP_ENDBUSRES | AT91C_UDP_RXSUSP;
    
    // Flush and enable the suspend interrupt
    pUDP->UDP_ICR |= AT91C_UDP_WAKEUP | AT91C_UDP_RXRSM | AT91C_UDP_RXSUSP;
    
    // Enable UDP tranceiver 
    pUDP->UDP_TXVC &= ~AT91C_UDP_TXVDIS;
    
    // The device leaves the Address & Configured states
    CLEAR (udp->device_state, UDP_STATE_ADDRESS | UDP_STATE_CONFIGURED);
    
    // We are in default state now
    SET (udp->device_state, UDP_STATE_DEFAULT);
}


/**
 * UDP check bus status
 * 
 * This routine enables/disables the UDP module by monitoring
 * the USB power signal.
 * 
 */
bool
udp_check_bus_status (udp_t udp)
{
    AT91PS_UDP pUDP = udp->pUDP;

    // Check udp_init called
    if (!udp->pUDP)
    {
        TRACE_ERROR (UDP, "UDP:udp_init not called\n");
        return 0;
    }

    if (udp_detect_p (udp))
    {
        //  If UDP is deactivated enable it
        if (ISCLEARED (udp->device_state, UDP_STATE_ATTACHED))
        {
            udp_enable_device (udp);
        }
           
        if (ISSET (udp->device_state, UDP_STATE_ATTACHED))
        {
            // Clear all interrupts
            pUDP->UDP_ICR = 0;

            irq_config (AT91C_ID_UDP, 7, 
                        AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE, udp_irq_service);
            
            irq_enable (AT91C_ID_UDP);

            // Enable UDP peripheral interrupts
            pUDP->UDP_IER = AT91C_UDP_ENDBUSRES | AT91C_UDP_RMWUPE
                | AT91C_UDP_RXSUSP;
            // We are in powered state now
            SET (udp->device_state, UDP_STATE_POWERED);
        }   
    }
    else
    {
        // Check if UDP is activated
        if (ISSET (udp->device_state, UDP_STATE_ATTACHED))
        {
            udp_disable_device (udp);
        }         
    }
    
    return ISSET (udp->device_state, UDP_STATE_POWERED);
}


bool
udp_awake_p (udp_t udp)
{
    // Could poll interrupts here...

    return udp_check_bus_status (udp)
        && ISCLEARED (udp->device_state, UDP_STATE_SUSPENDED);
}


udp_t udp_init (udp_request_handler_t request_handler, void *arg)
{
    udp_t udp = &udp_dev;

    udp->request_handler = request_handler;
    udp->request_handler_arg = arg;

    /* Signal the host by pulling D+ high.  This might be pulled high
       with an external 1k5 resistor.  */
    udp_connect (udp);

    udp->pUDP = AT91C_BASE_UDP;
    udp->configuration = 0;
    udp->connection = 0;
    udp->rx_bytes = 0;
    udp->rx_bank = AT91C_UDP_RX_DATA_BK0;
    udp->device_state = UDP_STATE_DETACHED;

    return udp;
}

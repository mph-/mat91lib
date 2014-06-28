#include "delay.h"
#include "irq.h"
#include "mcu.h"
#include "pio.h"
#include "trace.h"
#include "udp.h"


/* 
   UDP USB device port.

   This driver is overly complicated.  It could be simplified by using
   only EP0 for control, EP1 for bulk OUT, and EP2 for bulk IN.

   The AT91 UDP has two communication endpoints (EP1 and EP2) that
   support bulk and isochronous transfers using 64-byte ping-pong
   FIFOs.  EP1 is the bulk IN endpoint (to host) and EP2 is the bulk
   OUT endpoint (from host).

   EP0 is the default control endpoint used for the enumeration process.  This
   is bidirectional with an 8-byte FIFO.

   EP0 and EP3 cannot be ping-ponged.

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
    2. A ZLP to acknowledge the request if no data is to be returned
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
   can use the standard control messages get-status, set-feature, and
   clear-feature to examine and manipulate the halted status of a
   given endpoint.  The halted condition can be cleared.


   Ping-pong operation (reads on IN):

   1. The host sends a data packet.  This goes into FIFO 0.  The
      number of bytes in the packet is loaded into RXBYTECNT (note,
      this not affected by reads from the FIFO) and the BK0 flag is
      set (this can generate an interrupt).

   2. The host can now send another data packet.  This goes into FIFO
      1.  The host cannot send any more packets until the BK0 flag
      gets cleared by us.

   3. When we have finished reading FIFO 0, we clear BK0 flag.  If a
      packet is pending in FIFO 1 then BK1 will be set and RXBYTECNT
      updated.


   Transmission errors (see http://wiki.osdev.org/Universal_Serial_Bus):

   1. Too Much Data

   When transferring from host to device, if the host sends more data
   than negotiated during the SETUP transaction (i.e., the device
   receives more data than it expects; specifically, the host does not
   advance to the STATUS stage when the device expects), the device
   endpoint halts the pipe.

   When transferring from device to host, if the device sends more
   data than negotiated during the SETUP transaction (i.e., the host
   receives an extra data payload, or the final data payload is larger
   than it should be), the host considers it an error and aborts the
   transfer.

   2. Bus Errors

   In the event of a bus error or anomaly, an endpoint may receive a
   SETUP packet in the middle of a control transfer. In such a case, the
   endpoint must abort the current transfer and handle the unexpected
   SETUP packet. This behavior should be completely transparent to the
   host; the host should neither expect nor take advantage of this
   behavior.  

   3. Halt Conditions

   A control endpoint may recover from a halt condition upon receiving
   a SETUP packet. If the endpoint does not recover from a SETUP packet,
   it may need to be recovered via a different pipe. If an endpoint with
   the endpoint number 0 does not recover with a SETUP packet, the host
   should issue a device reset.


   Bulk Data Transfer:

   Like control transfers, a bulk transfer endpoint must transmit data
   payloads of the maximum data payload size for that endpoint with
   the exception of the last data payload in a particular
   transfer. The last data payload should not be padded out to the
   maximum data payload size.

   The bulk transfer is considered complete when the endpoint has
   transferred exactly as much data as expected, the endpoint
   transfers a packet with a data payload size less than the
   endpoint's maximum data payload size, or the endpoint transfers a
   zero-length packet.

   
   Setups:

   A SETUP transaction's data payload is always 8 bytes and thus
   receivable by the endpoint of any USB device. Consequently, the
   host may query the appropriate descriptor from a newly-attached
   full-speed device during configuration in order to determine the
   maximum data payload size for any endpoint; the host can then
   adhere to that maximum for any future transmissions.


   Stalls:

   A function uses the STALL handshake packet to indicate that it is
   unable to transmit or receive data.  Besides the default control
   pipe, all of a function's endpoints are in an undefined state after
   the device issues a STALL handshake packet.  The host never issues
   a STALL handshake packet.

   Typically, the STALL handshake indicates a functional stall.  A
   functional stall occurs when the halt feature of an endpoint is
   set. In this circumstance, host intervention is required via the
   default control pipe to clear the halt feature of the halted
   endpoint.

   Less often, the function returns a STALL handshake during a SETUP
   or DATA stage of a control transfer.  This is called a protocol
   stall and is resolved when the host issues the next SETUP
   transaction.
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


#define UDP_TIMEOUT_US 500000


/** Set flag(s) in a register.  */
#define UDP_REG_SET(REG, FLAGS) ((REG) = (REG) | (FLAGS))

/** Clear flag(s) in a register.  */
#define UDP_REG_CLR(REG, FLAGS) ((REG) &= ~(FLAGS))

/** Poll the status of flags in a register.  */
#define UDP_REG_ISSET(REG, FLAGS) (((REG) & (FLAGS)) == (FLAGS))

/** Poll the status of flags in a register.  */
#define UDP_REG_ISCLR(REG, FLAGS) (((REG) & (FLAGS)) == 0)

/** Clear flags of UDP UDP_CSR register and wait for synchronization.  */
#define UDP_CSR_CLR(REG, FLAGS) { \
while ((REG) & (FLAGS))           \
    (REG) &= ~(FLAGS);            \
}

/** Set flags of UDP UDP_CSR register and wait for synchronization.  */
#define UDP_CSR_SET(REG, FLAGS) {      \
while (((REG) & (FLAGS)) != (FLAGS))   \
    (REG) |= (FLAGS);                  \
}



#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif


#ifndef HIGH_BYTE
#define HIGH_BYTE(v) (((v) >> 8) & 0xff)
#endif


#ifndef LOW_BYTE
#define LOW_BYTE(v) ((v) & 0xff)
#endif


typedef enum
{
    UDP_ERROR_WEIRD,
    UDP_ERROR_FISHY,
    UDP_ERROR_READ_TIMEOUT,
    UDP_ERROR_WRITE_TIMEOUT,
} udp_error_t;


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
 * Structure for endpoint transfer parameters.
 * 
 */
typedef struct
{
    volatile udp_ep_state_t state;
    /* There can be a race-condition with the following five variables
       since they are written by both the ISR and main code.
       transferred should not be read until the endpoint is in the
       idle state.  */
    udp_status_t status;
    uint16_t remaining;
    uint16_t buffered;
    uint16_t transferred;

    /* For debugging.  */
    uint16_t requested;
    uint16_t requested_prev;

    /* Pointer to where data is stored.  */
    uint8_t *pdata;

    /* Callback function.  */
    udp_callback_t callback;
    /* Callback argument.  */
    void *arg;

    uint16_t fifo_size;
    uint8_t num_fifo;
    uint8_t bank;
    uint16_t timeouts;
    uint16_t spurious;
    uint16_t fishy;
    uint16_t weird;
} udp_ep_info_t;


struct udp_dev_struct
{
    uint8_t connection;
    /* Chosen configuration, 0 if not configured.  */
    uint8_t configuration;
    uint16_t address;
    udp_request_handler_t request_handler;
    void *request_handler_arg;
    volatile udp_state_t state;
    volatile udp_state_t prev_state;
    udp_setup_t setup;
    uint32_t rx_bank;
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


/* Possible interrupts:
   EP0INT: Endpoint 0 Interrupt
   EP1INT: Endpoint 1 Interrupt
   EP2INT: Endpoint 2Interrupt
   EP3INT: Endpoint 3 Interrupt
   RXSUSP: UDP Suspend Interrupt
   RXRSM:  UDP Resume Interrupt
   SOFINT: Start Of Frame Interrupt
   WAKEUP: UDP bus Wakeup Interrupt
   ENDBUSRES: End of BUS Reset Interrupt
   
   
   Several signals can generate an endpoint interupt:
   RXSETUP set to 1 (incoming setup request)
   RX_DATA_BK0 set to 1 (receive on bank 0
   RX_DATA_BK1 set to 1 (receive on bnak 1)
   TXCOMP set to 1 (transmit complete)
   STALLSENT set to 1
*/

/* UDP interrupt service routine.  Handle all UDP peripheral
   interrupts.  */
static void
udp_interrupt_handler (void)
{
    udp_ep_t endpoint;
    udp_t udp = &udp_dev;
    uint32_t status;

    status = UDP->UDP_ISR & UDP->UDP_IMR & ISR_MASK;

    while (status != 0)
    {
        // Start of frame (SOF)
        if (UDP_REG_ISSET (status, UDP_IER_SOFINT))
        {
            // TRACE_INFO (UDP, "UDP:SOF\n");
            // Acknowledge interrupt
            UDP_REG_SET (UDP->UDP_ICR, UDP_IER_SOFINT);
            UDP_REG_CLR (status, UDP_IER_SOFINT);
        }        

        // Suspend
        if (UDP_REG_ISSET (status, UDP_IER_RXSUSP))
        {
            // TRACE_INFO (UDP, "UDP:Susp\n");
           
            if (udp->state != UDP_STATE_SUSPENDED)
            {
                // The device enters the Suspended state
                //      MCK + UDPCK must be off
                //      Pull-Up must be connected
                //      Transceiver must be disabled

                // Enable wakeup
                UDP_REG_SET (UDP->UDP_IER, UDP_IER_WAKEUP | UDP_IER_RXRSM);

                // Acknowledge interrupt
                UDP_REG_SET (UDP->UDP_ICR, UDP_IER_RXSUSP);

                // Set suspended state
                udp->prev_state = udp->state;
                udp->state = UDP_STATE_SUSPENDED;

                // Disable transceiver
                UDP_REG_SET (UDP->UDP_TXVC, UDP_TXVC_TXVDIS);
                // Disable peripheral clock
                mcu_pmc_disable (ID_UDP);
                // Disable UDPCK
                PMC->PMC_SCDR |= PMC_SCDR_UDP;

                // TODO, we need to pull less than 500 uA from the 5V VBUS
                // so need a callback or the user should poll
                // for the suspend state.
            }            
        }
        // Resume
        else if (UDP_REG_ISSET (status, UDP_IER_WAKEUP) 
                 || UDP_REG_ISSET (status, UDP_IER_RXRSM))
        {
            // TRACE_INFO (UDP, "UDP:Resm\n");

            // The device enters configured state
            //      MCK + UDPCK must be on
            //      Pull-Up must be connected
            //      Transceiver must be enabled
            // Powered state
            // Enable master clock
            mcu_pmc_enable (ID_UDP);
            // Enable peripheral clock for UDP
            PMC->PMC_SCER |= PMC_SCER_UDP;

            if (udp->prev_state == UDP_STATE_DEFAULT)
            {
                // Enable transceiver
                UDP_REG_CLR (UDP->UDP_TXVC, UDP_TXVC_TXVDIS);
            }

            udp->state = udp->prev_state;

            UDP_REG_SET (UDP->UDP_ICR, UDP_IER_WAKEUP | UDP_IER_RXRSM
                 | UDP_IER_RXSUSP);
            UDP_REG_SET (UDP->UDP_IDR, UDP_IER_WAKEUP | UDP_IER_RXRSM);
        }        
        // End of bus reset (non maskable)
        else if (UDP_REG_ISSET (status, UDP_ISR_ENDBUSRES))
        {
            // TRACE_INFO (UDP, "UDP:EoBres\n");

            // Initialize UDP peripheral device
            udp_bus_reset_handler (udp);

            udp->state = UDP_STATE_DEFAULT;

            // Flush and enable the suspend interrupt
            UDP_REG_SET (UDP->UDP_ICR, UDP_IER_WAKEUP 
                 | UDP_IER_RXRSM | UDP_IER_RXSUSP);

            // Acknowledge end of bus reset interrupt.  Is this
            // required since it is done in udp_bus_reset_handler?
            UDP_REG_SET (UDP->UDP_ICR, UDP_ISR_ENDBUSRES);
        }
        // Endpoint interrupts
        else 
        {
            while (status != 0)
            {
                // Get endpoint index
                endpoint = last_set_bit (status);
                udp_endpoint_handler (&udp_dev, endpoint);
                UDP_REG_CLR (status, 1 << endpoint);
            }
        }          

        // Why re-read?
        status = UDP->UDP_ISR & UDP->UDP_IMR & ISR_MASK;
       
        // Mask unneeded interrupts
        if (udp->state != UDP_STATE_DEFAULT)
        {
            status &= UDP_ISR_ENDBUSRES | UDP_IER_SOFINT;
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
void udp_endpoint_complete (udp_t udp, udp_ep_t endpoint, udp_status_t status)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];

    if ((pep->state == UDP_EP_STATE_WRITE) || (pep->state == UDP_EP_STATE_READ)) 
    {
        TRACE_DEBUG (UDP, "UDP:EoT%d %d\n",
                     endpoint, pep->transferred);

        pep->status = status;
        
        /* Invoke callback if present.  */
        if (pep->callback != 0) 
        {
            udp_transfer_t transfer;

            transfer.status = pep->status;
            transfer.transferred = pep->transferred;

            pep->callback (pep->arg, &transfer);
        }
        pep->state = UDP_EP_STATE_IDLE;
    }
}


static void
udp_endpoint_reset (udp_t udp, udp_ep_t endpoint)
{
    /* Reset endpoint FIFOs.  Thie clears RXBYTECNT.  It does not clear
       the other CSR flags.  */
    UDP_REG_SET (UDP->UDP_RST_EP, 1 << endpoint);
    UDP_REG_CLR (UDP->UDP_RST_EP, 1 << endpoint);
}


/* This is for debugging.  */
void
udp_endpoint_error (udp_t udp, udp_ep_t endpoint, udp_error_t error)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];

    switch (error)
    {
    case UDP_ERROR_FISHY:
        pep->fishy++;
        break;

    case UDP_ERROR_WEIRD:
        pep->weird++;
        break;

    case UDP_ERROR_READ_TIMEOUT:
    case UDP_ERROR_WRITE_TIMEOUT:
        pep->timeouts++;
        break;
    }

    udp_endpoint_reset (udp, endpoint);
}


static void
udp_endpoint_interrupt_disable (udp_t udp, udp_ep_t endpoint)
{
    UDP_REG_SET (UDP->UDP_IDR, 1 << endpoint);
}


static void
udp_endpoint_interrupt_enable (udp_t udp, udp_ep_t endpoint)
{
    UDP_REG_SET (UDP->UDP_IER, 1 << endpoint);
}


/**
 * Configure specific endpoint
 * \param   endpoint Endpoint to configure
 * 
 */
static void
udp_endpoint_configure (udp_t udp, udp_ep_t endpoint)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];

    /* Abort the current transfer if the endpoint was configured and
       in read or write state.  */
    if ((pep->state == UDP_EP_STATE_READ)
        || (pep->state == UDP_EP_STATE_WRITE))
    {
        udp_endpoint_complete (udp, endpoint, UDP_STATUS_RESET);
    }

    pep->state = UDP_EP_STATE_IDLE;

    pep->pdata = 0;
    pep->remaining = 0;
    pep->transferred = 0;
    pep->buffered = 0;
    pep->callback = 0;
    pep->arg = 0;

    udp_endpoint_reset(udp, endpoint);

    switch (endpoint)
    {
    case UDP_EP_CONTROL:
        udp->eps[0].fifo_size = UDP_EP_CONTROL_SIZE;
        udp->eps[0].num_fifo = 1;
        UDP->UDP_CSR[0] = UDP_CSR_EPTYPE_CTRL | UDP_CSR_EPEDS;
        break;
    
    case UDP_EP_OUT:
        udp->eps[1].fifo_size = UDP_EP_OUT_SIZE;
        udp->eps[1].num_fifo = 2;
        UDP->UDP_CSR[1] = UDP_CSR_EPTYPE_BULK_OUT | UDP_CSR_EPEDS;
        break;

    case UDP_EP_IN:
        udp->eps[2].fifo_size = UDP_EP_IN_SIZE;
        udp->eps[2].num_fifo = 2;
        UDP->UDP_CSR[2] = UDP_CSR_EPTYPE_BULK_IN | UDP_CSR_EPEDS;         
        break;
    }

    UDP_CSR_CLR (UDP->UDP_CSR[endpoint], 
                 UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1);

    TRACE_DEBUG (UDP, "UDP:Cfg%d\n", endpoint);
}


/**
 * Clear transmission status flag and also swap banks for dual FIFO endpoints.
 * \param   endpoint    Endpoint where to clear flag
 */
static void 
udp_rx_flag_clear (udp_t udp, udp_ep_t endpoint)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];

    /* Clear previous bank flag, BK0 or BK1.  When we set this zero we
       tell the controller that we have read all the data in the
       specified bank.  */
    UDP_CSR_CLR (UDP->UDP_CSR[endpoint], pep->bank);

    // Swap banks
    if (pep->bank == UDP_CSR_RX_DATA_BK0) 
    {
        if (pep->num_fifo > 1)
        {
            // Swap bank if in dual-fifo mode
            pep->bank = UDP_CSR_RX_DATA_BK1;
        }
    }
    else 
    {
        pep->bank = UDP_CSR_RX_DATA_BK0;
    }    
}


/**
 * Writes data to UDP FIFO
 * \param   endpoint    Endpoint to write data
 * 
 */
void
udp_endpoint_fifo_write (udp_t udp, udp_ep_t endpoint)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];
    unsigned int bytes;
    unsigned int i;
    uint8_t *src;

    /* Determine the number of bytes to send.  */
    bytes = MIN (pep->fifo_size, pep->remaining);

    TRACE_DEBUG (UDP, "UDP:Write%d %d\n", endpoint, bytes);

    /* Transfer one packet to the FIFO buffer.  */
    src = pep->pdata;
    for (i = 0; i < bytes; i++)
        UDP->UDP_FDR[endpoint] = *src++;
    pep->pdata = src;

    pep->buffered += bytes;
    pep->remaining -= bytes;
}


/**
 * Send data packet via given endpoint
 * \param   endpoint    Endpoint to send data through
 * \param   pdata       Pointer to data buffer
 * \param   len         Packet size
 * \param   callback    Optional callback to invoke when the read finishes
 * \param   arg         Optional callback argument 
 * \return  Operation result code
 * 
 * This may be called within an ISR for handling setup requests.
 * There is no limit to the buffer size.   The callback function is
 * called when the transfer is fully complete.
 */
udp_status_t
udp_write_async (udp_t udp, udp_ep_t endpoint, const void *pdata, 
                 unsigned int len, udp_callback_t callback, void *arg)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];

    if (pep->state != UDP_EP_STATE_IDLE) 
        return UDP_STATUS_BUSY;

    TRACE_DEBUG (UDP, "UDP:AWrite%d %d\n", endpoint, len);

    pep->pdata = (uint8_t *)pdata;
    pep->status = UDP_STATUS_PENDING;
    pep->remaining = len;
    pep->buffered = 0;
    pep->transferred = 0;
    pep->callback = callback;
    pep->arg = arg;
    pep->state = UDP_EP_STATE_WRITE;

    /* For debugging.  */
    pep->requested_prev = pep->requested;
    pep->requested = len;

    if (UDP_REG_ISSET (UDP->UDP_CSR[endpoint], UDP_CSR_TXCOMP))
        udp_endpoint_error (udp, endpoint, UDP_ERROR_FISHY);
    
    /* This should be automatically cleared when a FIFO contents are
       sent to the host.  */
    if (UDP_REG_ISSET (UDP->UDP_CSR[endpoint], UDP_CSR_TXPKTRDY))
        udp_endpoint_error (udp, endpoint, UDP_ERROR_WEIRD);
    
    udp_endpoint_interrupt_disable (udp, endpoint);
        
    /* Write the first packet to the FIFO (this may be a ZLP).
       Interrupts should be disabled for this endpoint at this
       point.  */
    udp_endpoint_fifo_write (udp, endpoint);
    
    /* Tell the controller there is data ready to send.  */
    UDP_CSR_SET (UDP->UDP_CSR[endpoint], UDP_CSR_TXPKTRDY);
    
    /* If there is data remaining and if double buffering is enabled
       then load the other FIFO with a packet.  */
    if ((pep->remaining > 0) && (pep->num_fifo > 1))
        udp_endpoint_fifo_write (udp, endpoint);
    
    udp_endpoint_interrupt_enable (udp, endpoint);
        
    return UDP_STATUS_SUCCESS;
}


static unsigned int 
udp_endpoint_read_bytes (udp_t udp, udp_ep_t endpoint)
{
    /* Return RXBYTECNT.  */
    return UDP->UDP_CSR[endpoint] >> 16;
}


bool
udp_endpoint_read_ready_p (udp_t udp, udp_ep_t endpoint)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];

    /* Check if have some data in the FIFO.  */
    return UDP_REG_ISSET (UDP->UDP_CSR[endpoint],
                          pep->bank);
}


/**
 * Read data from UDP FIFO
 * \param   endpoint    Endpoint to read data from
 * \param   packetsize  Maximum size of packet to receive
 * \return  Number of bytes read.
 * 
 */
static unsigned int 
udp_endpoint_fifo_read (udp_t udp, udp_ep_t endpoint)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];
    unsigned int bytes;
    unsigned int i;
    uint8_t *dst;

    /* Assume that something in buffer.  */

    if (pep->buffered == 0
        && udp_endpoint_read_ready_p (udp, endpoint))
        pep->buffered = udp_endpoint_read_bytes (udp, endpoint);

    /* Get number of bytes to retrieve.  */
    bytes = MIN (pep->remaining, pep->buffered);

    TRACE_DEBUG (UDP, "UDP:Read%d %d\n", endpoint, bytes);

    /* Read from FIFO buffer.  */
    dst = pep->pdata;
    for (i = 0; i < bytes; i++) 
        *dst++ = UDP->UDP_FDR[endpoint];
    pep->pdata = dst;

    pep->remaining -= bytes;
    pep->transferred += bytes;

    pep->buffered -= bytes;
    return bytes;
}


/**
 * Read data packet from given endpoint
 * 
 * Read a data packet with specific size from given endpoint.
 * 
 * \param   endpoint    Endpoint to send data through
 * \param   pdata       Pointer to data buffer
 * \param   len         Packet size
 * \param   callback    Optional callback to invoke when the read finishes
 * \param   arg         Optional callback argument 
 * \return  Operation result code
 * 
 */
udp_status_t
udp_read_async (udp_t udp, udp_ep_t endpoint, void *pdata, unsigned int len, 
                udp_callback_t callback, void *arg)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];

    if (pep->state != UDP_EP_STATE_IDLE)
        return UDP_STATUS_BUSY;

    TRACE_DEBUG (UDP, "UDP:ARead%d %d\n", endpoint, len);

    udp_endpoint_interrupt_disable (udp, endpoint);

    pep->pdata = pdata;
    pep->status = UDP_STATUS_PENDING;
    pep->remaining = len;
    pep->transferred = 0;
    pep->callback = callback;
    pep->arg = arg;
    pep->state = UDP_EP_STATE_READ;

    if (udp_endpoint_read_ready_p (udp, endpoint))
    {
        udp_endpoint_fifo_read (udp, endpoint);

        if (!pep->buffered)
        {
            /* The FIFO buffer is empty so tell controller.  */
            udp_rx_flag_clear (udp, endpoint);
        }

        if (pep->remaining == 0)
        {
            udp_endpoint_complete (udp, endpoint, UDP_STATUS_SUCCESS);
            return UDP_STATUS_SUCCESS;
        }
    }

    /* Enable interrupt on endpoint since waiting for more data.  */
    udp_endpoint_interrupt_enable (udp, endpoint);
    
    return UDP_STATUS_SUCCESS;
}


static void
udp_setup_read (udp_t udp, udp_ep_t endpoint)
{
    udp_setup_t *setup = &udp->setup;

    // Read setup request packet and squirrel away for later.
    setup->type = UDP->UDP_FDR[0];
    setup->request = UDP->UDP_FDR[0];
    setup->value = UDP->UDP_FDR[0] & 0xFF;
    setup->value |= (UDP->UDP_FDR[0] << 8);
    setup->index = UDP->UDP_FDR[0] & 0xFF;
    setup->index |= (UDP->UDP_FDR[0] << 8);
    setup->length = UDP->UDP_FDR[0] & 0xFF;
    setup->length |= (UDP->UDP_FDR[0] << 8);
    
    // Set the DIR bit before clearing RXSETUP in Control IN sequence.
    if (setup->type & 0x80)
        UDP_CSR_SET (UDP->UDP_CSR[endpoint], UDP_CSR_DIR);
    
    // Acknowledge that setup data packet has been read from FIFO.
    UDP_CSR_CLR (UDP->UDP_CSR[endpoint], UDP_CSR_RXSETUP);
}


void
udp_endpoint_write_handler (udp_t udp, udp_ep_t endpoint)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];
    unsigned int status = UDP->UDP_CSR[endpoint];

    if (pep->state == UDP_EP_STATE_WRITE)
    {
        /* The endpoint is in the write state.
           The scenarios are:
           1: no more data to send (buffered <= packet_size
                                          && remaining == 0)
           1: just buffered data to send (buffered > packet_size
                                          && remaining == 0)
           2: buffered data and more to send (remaining > 0 and ping-pong)
           3: no buffered data and more to send (remaining > 0 and non-ping-pong)
        */

        if ((pep->buffered <= pep->fifo_size && pep->remaining == 0)
            || (((status & UDP_CSR_EPTYPE_Msk) == UDP_CSR_EPTYPE_CTRL)
                && (pep->remaining == 0)
                && (pep->buffered == pep->fifo_size)))
        {
            /* Case 1: Transfer completed.  */

            pep->transferred += pep->buffered;
            pep->buffered = 0;
            
            UDP_CSR_CLR (UDP->UDP_CSR[endpoint], UDP_CSR_TXCOMP);

            /* Terminate transfer and call callback.  */
            udp_endpoint_complete (udp, endpoint, UDP_STATUS_SUCCESS);
        }
        else
        {
            pep->transferred += pep->fifo_size;
            pep->buffered -= pep->fifo_size;
            
            /* Transfer next block of data.  */
            if (pep->num_fifo == 1)
            {
                /* No double buffering, so load FIFO and say it is
                   ready.  */
                udp_endpoint_fifo_write (udp, endpoint);

                UDP_CSR_SET (UDP->UDP_CSR[endpoint], UDP_CSR_TXPKTRDY);

                /* Acknowledge TXCOMP interrupt.  The datasheet says
                 * this must be cleared adfter TXPKTRDY is set. */
                UDP_CSR_CLR (UDP->UDP_CSR[endpoint], UDP_CSR_TXCOMP);
            }
            else
            {
                /* Say another packet ready and acknowledge TXCOMP
                   interrupt.  The data sheet shows this should be
                   cleared after TXPKTRDY set and before the FIFO is
                   written.  The tutorial says the other way around!
                   Let's try both at the same time.  */

                status = (status & ~UDP_CSR_TXCOMP) | UDP_CSR_TXPKTRDY;
                while (UDP->UDP_CSR[endpoint] != status)
                    UDP->UDP_CSR[endpoint] = status;

                /* Load other FIFO with data.  */
                if (pep->remaining)
                    udp_endpoint_fifo_write (udp, endpoint);
                else if (pep->buffered <= pep->fifo_size)
                {
                    /* The transfer will terminate soon since
                       there is only pep->buffered samples to send.
                       So just poll for TXCOMP going high.  */
                    while (UDP_REG_ISCLR (UDP->UDP_CSR[endpoint],
                                          UDP_CSR_TXCOMP))
                        continue;

                    pep->transferred += pep->buffered;
                    pep->buffered = 0;
            
                    UDP_CSR_CLR (UDP->UDP_CSR[endpoint], UDP_CSR_TXCOMP);

                    /* Terminate transfer and call callback.  */
                    udp_endpoint_complete (udp, endpoint, UDP_STATUS_SUCCESS);
                }
            }
        }
    }
    else
    {
        /* Hmmm, how did we get here?  */
        UDP_CSR_CLR (UDP->UDP_CSR[endpoint], UDP_CSR_TXCOMP);
        pep->spurious++;
    }
}


void
udp_endpoint_read_handler (udp_t udp, udp_ep_t endpoint)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];
    unsigned int status = UDP->UDP_CSR[endpoint];

    if (pep->state == UDP_EP_STATE_READ)
    {
        if (pep->buffered)
        {
            /* Ouch.  Raise the white flag.  We've got another interrupt
               before we've read the previous buffer.  */
            udp_endpoint_error (udp, endpoint, UDP_ERROR_FISHY);
        }

        udp_endpoint_fifo_read (udp, endpoint);
        
        if (!pep->buffered)
        {
            /* The FIFO buffer is empty so tell controller.  */
            udp_rx_flag_clear (udp, endpoint);
        }

        if (pep->remaining == 0)
        {
            udp_endpoint_complete (udp, endpoint, UDP_STATUS_SUCCESS);
            udp_endpoint_interrupt_disable (udp, endpoint);
        }
    }
    else
    {
        /* Endpoint is not in read state so check the endpoint
           type to see if it is a control endpoint.  */
        if (((status & UDP_CSR_EPTYPE_Msk) == UDP_CSR_EPTYPE_CTRL)
            && UDP_REG_ISCLR (status, 0xFFFF0000))
        {
            /* Control endpoint, 0 bytes received.  This is the
               host sending an Ack in response to us sending a
               ZLP.  Acknowledge the data and finish the
               current transfer.  */
            TRACE_INFO (UDP, "UDP:Ack%d\n", endpoint);
            udp_rx_flag_clear (udp, endpoint);
            
            udp_endpoint_complete (udp, endpoint, UDP_STATUS_SUCCESS);
        }
        else if (UDP_REG_ISSET (status, UDP_CSR_FORCESTALL))
        {
            /* Non-control endpoint so discard stalled data.  */
            TRACE_INFO (UDP, "UDP:Disc%d\n", endpoint);
            pep->buffered = 0;
            udp_rx_flag_clear (udp, endpoint);
        }
        else
        {
            pep->spurious++;
        }
    }
}


void
udp_endpoint_setup_handler (udp_t udp, udp_ep_t endpoint)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];

    /* This interrupt occurs as a result of receiving a setup packet.  */
    TRACE_DEBUG (UDP, "UDP:Setup\n");
    
    /* Handle the case where during the status phase of a control
       write transfer, the host receives the device ZLP and acks
       it, but the ack is not received by the device.  Hmmm?  */
    if ((pep->state == UDP_EP_STATE_WRITE)
        || (pep->state == UDP_EP_STATE_READ))
    {
        udp_endpoint_complete (udp, endpoint, UDP_STATUS_SUCCESS);
    }

    /* Read then process setup packet.  */
    udp_setup_read (udp, endpoint);
    
    /* Pass setup packet to request handler.  */
    if (!udp->request_handler
        || !udp->request_handler (udp->request_handler_arg, &udp->setup))
    {
        /* Send protocol stall if not handled.  */
        udp_stall (udp, UDP_EP_CONTROL);
    }
}


void
udp_endpoint_stall_handler (udp_t udp, udp_ep_t endpoint)
{
    udp_ep_info_t *pep = &udp->eps[endpoint];

    /* This interrupt occurs as a result of setting FORCESTALL.  */
    
    TRACE_INFO (UDP, "UDP:Stallsent%d\n", endpoint);
    
    /* Acknowledge interrupt.  */
    UDP_CSR_CLR (UDP->UDP_CSR[endpoint], UDP_CSR_STALLSENT);
    
    /* If the endpoint is not halted, clear the stall condition.  */
    if (pep->state != UDP_EP_STATE_HALTED)
        UDP_CSR_CLR (UDP->UDP_CSR[endpoint], UDP_CSR_FORCESTALL);
}


    
/**
 * UDP endpoint handler
 * \param   endpoint    Endpoint to use
 * 
 * Called from UDP interrupt handler in response to endpoint interrupt.
 */
void 
udp_endpoint_handler (udp_t udp, udp_ep_t endpoint)
{
    unsigned int status = UDP->UDP_CSR[endpoint];
    
    /* IN packet sent.  */
    if (UDP_REG_ISSET (status, UDP_CSR_TXCOMP))
    {
        udp_endpoint_write_handler (udp, endpoint);
    }

    /* OUT packet received.  */
    if (UDP_REG_ISSET (status, UDP_CSR_RX_DATA_BK0) 
        || UDP_REG_ISSET (status, UDP_CSR_RX_DATA_BK1))
    {
        udp_endpoint_read_handler (udp, endpoint);
    }

    /* Setup packet received.  */
    if (UDP_REG_ISSET (status, UDP_CSR_RXSETUP))
    {
        udp_endpoint_setup_handler (udp, endpoint);
    }

    /* Stall sent.  */
    if (UDP_REG_ISSET (status, UDP_CSR_STALLSENT))
    {
        udp_endpoint_stall_handler (udp, endpoint);
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
    udp_ep_info_t *pep = &udp->eps[endpoint];

    if (pep->state != UDP_EP_STATE_IDLE) 
        return;

    TRACE_INFO (UDP, "UDP:Stall%d\n", endpoint);

    UDP_CSR_SET (UDP->UDP_CSR[endpoint], UDP_CSR_FORCESTALL);
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
        UDP_CSR_CLR (UDP->UDP_CSR[endpoint], UDP_CSR_FORCESTALL);

        // Reset endpoint FIFOs, beware this is a 2 step operation
        UDP_REG_SET (UDP->UDP_RST_EP, 1 << endpoint);
        UDP_REG_CLR (UDP->UDP_RST_EP, 1 << endpoint);
    }
    // Set the halt feature on the endpoint if it is not already enabled
    // and the endpoint is not disabled
    else if ((pep->state != UDP_EP_STATE_HALTED)
             && (pep->state != UDP_EP_STATE_DISABLED))
    {
        TRACE_INFO (UDP, "UDP:Halt%d\n", endpoint);

        // Abort the current transfer if necessary
        udp_endpoint_complete (udp, endpoint, UDP_STATUS_ABORTED);

        // Put endpoint into halt state
        UDP_CSR_SET (UDP->UDP_CSR[endpoint], UDP_CSR_FORCESTALL);
        pep->state = UDP_EP_STATE_HALTED;

        // Enable the endpoint interrupt
        udp_endpoint_interrupt_enable (udp, endpoint);
    }
    
    // Return the endpoint halt status
    return pep->state == UDP_EP_STATE_HALTED;
}


bool
udp_idle_p (udp_t udp, udp_ep_t endpoint)
{
    return udp->eps[endpoint].state == UDP_EP_STATE_IDLE;
}


bool
udp_halt_p (udp_t udp, udp_ep_t endpoint)
{
    return udp->eps[endpoint].state == UDP_EP_STATE_HALTED;
}


/**
 * Sets or unsets the device address
 *
 */ 
void
udp_address_set (void *arg, udp_transfer_t *ptransfer __unused__)
{
    udp_t udp = arg;
    uint16_t address;

    address = udp->setup.value;

    // Save address for debugging
    udp->address = address;

    TRACE_DEBUG (UDP, "UDP:SetAddr 0x%2x\n", address);

    // Set address
    UDP_REG_SET (UDP->UDP_FADDR, UDP_FADDR_FEN | address);

    if (address == 0) 
    {
        // Inform UDP that out of address state
        UDP_REG_SET (UDP->UDP_GLB_STAT, 0);

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
        UDP_REG_SET (UDP->UDP_GLB_STAT, UDP_GLB_STAT_FADDEN);

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
    unsigned int config = udp->setup.value;
    unsigned int ep;

    TRACE_DEBUG (UDP, "UDP:SetCfg %d\n", config);

    // Check the request
    if (config != 0)
    {
        // TODO, if have multiple configurations, use selected one

        // Enter configured state
        udp->state = UDP_STATE_CONFIGURED;
        UDP_REG_SET (UDP->UDP_GLB_STAT, UDP_GLB_STAT_CONFG);

        // Configure endpoints
        for (ep = 1; ep < UDP_EP_NUM; ep++)
            udp_endpoint_configure (udp, ep);
    }
    else
    {
        // Go back to Address state
        udp->state = UDP_STATE_ADDRESS;
        UDP_REG_SET (UDP->UDP_GLB_STAT, UDP_GLB_STAT_FADDEN);

        // For each endpoint, if it is enabled, disable it
        for (ep = 1; ep < UDP_EP_NUM; ep++)
        {
            udp_endpoint_complete (udp, ep, UDP_STATUS_RESET);
            udp->eps[ep].state = UDP_EP_STATE_DISABLED;
        }
    }
}


void
udp_control_gobble (udp_t udp)
{
    /* I'm not too sure what the point of this is!.  */
    while (! (UDP->UDP_CSR[UDP_EP_CONTROL] & UDP_CSR_RX_DATA_BK0));

    UDP->UDP_CSR[UDP_EP_CONTROL] &= ~UDP_CSR_RX_DATA_BK0;
}


bool
udp_read_ready_p (udp_t udp)
{
    return udp_endpoint_read_ready_p (udp, UDP_EP_OUT);
}


udp_size_t
udp_endpoint_read (udp_t udp, udp_ep_t endpoint, void *buffer, udp_size_t len)
{
    unsigned int timeout;
    udp_ep_info_t *pep = &udp->eps[endpoint];

    if (udp_read_async (udp, endpoint, buffer, len, 0, 0)
        != UDP_STATUS_SUCCESS)
        return 0;

    /* We may have a race condition if someone else grabs the endpoint
       as soon as it goes idle.  I'm not sure if this can happen.  */

    timeout = UDP_TIMEOUT_US;
    while (pep->state != UDP_EP_STATE_IDLE)
    {
        timeout--;
        if (!timeout || ! udp_configured_p (udp))
        {
            udp_endpoint_error (udp, endpoint, UDP_ERROR_READ_TIMEOUT);
            udp_endpoint_complete (udp, endpoint, UDP_STATUS_TIMEOUT);
            break;
        }
        DELAY_US (1);
    }
    return pep->transferred;
}


udp_size_t
udp_endpoint_write (udp_t udp, udp_ep_t endpoint,
                    const void *buffer, udp_size_t len)
{
    unsigned int timeout;
    udp_ep_info_t *pep = &udp->eps[endpoint];

    /* This is slower than the old method of writing directly to the
       FIFOs, especially when sending single characters, but should be
       more robust since it uses the standard endpoint handling
       code.  */

    if (udp_write_async (udp, endpoint, buffer, len, 0, 0)
        != UDP_STATUS_SUCCESS)
        return 0;

    /* We may have a race condition if someone else grabs the endpoint
       as soon as it goes idle.  I'm not sure if this can happen.  */

    timeout = UDP_TIMEOUT_US;
    while (pep->state != UDP_EP_STATE_IDLE)
    {
        timeout--;
        if (!timeout || ! udp_configured_p (udp))
        {
            udp_endpoint_error (udp, endpoint, UDP_ERROR_WRITE_TIMEOUT);
            udp_endpoint_complete (udp, endpoint, UDP_STATUS_TIMEOUT);
            break;
        }
        DELAY_US (1);
    }
    return pep->transferred;
}


udp_size_t
udp_read (udp_t udp, void *buffer, udp_size_t len)
{
    return udp_endpoint_read (udp, UDP_EP_OUT, buffer, len);
}


udp_size_t
udp_write (udp_t udp, const void *buffer, udp_size_t len)
{
    return udp_endpoint_write(udp, UDP_EP_IN, buffer, len);
}


/* Signal the host by pulling D+ high (drive a K state).  */
void
udp_signal (udp_t udp __unused__)
{
#ifdef __SAM7__
#ifdef UDP_PULLUP_PIO
    /* Enable UDP pull up by driving gate of p-channel MOSFET low.  */
    pio_config_set (UDP_PULLUP_PIO, PIO_OUTPUT_LOW);
#endif
#else
    UDP->UDP_TXVC |= UDP_TXVC_PUON;
#endif
}


void
udp_unsignal (udp_t udp __unused__)
{
#ifdef __SAM7__
#ifdef UDP_PULLUP_PIO
    /* Configure UDP pullup pio and drive high to disable pullup.  */
    pio_config_set (UDP_PULLUP_PIO, PIO_OUTPUT_HIGH);
#endif
#else
    UDP->UDP_TXVC &= ~UDP_TXVC_PUON;
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

#ifdef __SAM7__
    // Set the PLL USB divider
    PMC->CKGR_PLLR |= AT91C_CKGR_USBDIV_1;
#else
    /* Note, this clears USBS to select PLLA.  */
    PMC->PMC_USB = PMC_USB_USBDIV ((int)(F_PLL / 48e6 + 0.5) - 1);
#endif

    // Enable the 48MHz USB clock UDPCK and System Peripheral USB clock
    PMC->PMC_SCER |= PMC_SCER_UDP;
    mcu_pmc_enable (ID_UDP);

    // Init data banks
    for (i = 0; i < UDP_EP_NUM; i++)
        udp->eps[i].bank = UDP_CSR_RX_DATA_BK0;

    TRACE_INFO (UDP, "UDP:Enabled\n");
}


/**
 * Disable UDP device
 */
static void
udp_disable (udp_t udp)
{
    udp_unsignal (udp);

    // Disable the 48MHz USB clock UDPCK and System Peripheral USB Clock
    PMC->PMC_SCDR |= PMC_SCDR_UDP;
    mcu_pmc_disable (ID_UDP);

    // Disable the interrupt on the interrupt controller
    irq_disable (ID_UDP);

    // Disable all UDP interrupts
    UDP->UDP_IDR = ~0;

    // Disable UDP transceiver
    UDP->UDP_TXVC |= UDP_TXVC_TXVDIS;

    TRACE_INFO (UDP, "UDP:Disabled\n");
}


/** Check UDP bus status.  Return non-zero if attached.  */
static bool
udp_bus_status_check (udp_t udp)
{
    bool attached;

    attached = udp_attached_p (udp);

    if (attached)
    {
        /*  If UDP is deactivated enable it.  */
        if (udp->state == UDP_STATE_NOT_POWERED)
        {
            udp->state = UDP_STATE_ATTACHED;

            udp_enable (udp);

            /* Signal the host by pulling D+ high.  This might be
               permanently pulled high with an external 1k5
               resistor.  */
            udp_signal (udp);            

            /* Clear all UDP interrupts.  */
            UDP->UDP_ICR = 0;

            /* Set up interrupt handler.  */
            irq_config (ID_UDP, 7, 
                        udp_interrupt_handler);
            
            irq_enable (ID_UDP);

            /* Enable UDP peripheral interrupts.  */
            UDP->UDP_IER = UDP_ISR_ENDBUSRES | UDP_IER_WAKEUP
                | UDP_IER_RXSUSP;

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
    return attached;
}


/** Return non-zero if configured.  */
bool
udp_poll (udp_t udp)
{
    udp_bus_status_check (udp);
    return udp_configured_p (udp);
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


/** UDP Protocol reset handler.  Called when a USB bus reset is
    received from the host.  */
static void 
udp_bus_reset_handler (udp_t udp)
{
    TRACE_DEBUG (UDP, "UDP:Reset\n");

    // Clear UDP peripheral interrupt
    UDP->UDP_ICR |= UDP_ISR_ENDBUSRES;
    
    // Reset all endpoints
    UDP->UDP_RST_EP = ~0;
    UDP->UDP_RST_EP = 0;
    
    // Enable the UDP
    UDP->UDP_FADDR = UDP_FADDR_FEN;

    // Configure the control endpoint
    udp_endpoint_configure (udp, 0);

    // Configure UDP peripheral interrupts, here only EP0 and bus reset
    UDP->UDP_IER = UDP_IER_EP0INT | UDP_ISR_ENDBUSRES | UDP_IER_RXSUSP;
    
    // Flush and enable the suspend interrupt
    UDP->UDP_ICR |= UDP_ICR_WAKEUP | UDP_ICR_RXRSM | UDP_ICR_RXSUSP;
    
    // Enable UDP transceiver 
    UDP->UDP_TXVC &= ~UDP_TXVC_TXVDIS;
}


udp_t udp_init (udp_request_handler_t request_handler, void *arg)
{
    udp_t udp = &udp_dev;

    udp->request_handler = request_handler;
    udp->request_handler_arg = arg;
    udp->setup.request = 0;

    udp->configuration = 0;
    udp->connection = 0;
    udp->prev_state = UDP_STATE_NOT_POWERED;
    udp->state = UDP_STATE_NOT_POWERED;
    udp->rx_bank = UDP_CSR_RX_DATA_BK0;
    
    udp_unsignal (udp);

    return udp;
}

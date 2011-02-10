#ifndef UDP_H
#define UDP_H

#include "config.h"

enum {UDP_EP_CONTROL_SIZE = 8, UDP_EP_OUT_SIZE = 64, UDP_EP_IN_SIZE = 64};


/* IN and OUT are referred to the host so we transmit on the IN endpoint
   and receive on the OUT endpoint.  */
typedef enum 
{
    UDP_EP_CONTROL = 0,
    UDP_EP_OUT = 1,
    UDP_EP_IN = 2
} udp_ep_t;

#define UDP_EP_NUM 3

enum {UDP_EP_DIR_OUT = 0, UDP_EP_DIR_IN = 0x80};


typedef enum
{
//! Completed successfully
    UDP_STATUS_SUCCESS = 0,
//! Aborted because the recipient (device, endpoint, ...) was busy
    UDP_STATUS_BUSY = 1,
//! Aborted because of abnormal status
    UDP_STATUS_ABORTED = 2,
//! Aborted because the endpoint or the device has been reset
    UDP_STATUS_RESET = 3,
//! Waiting completion of transfer
    UDP_STATUS_PENDING = 4,
} udp_status_t;


typedef struct udp_transfer_struct
{
    udp_status_t status;
    volatile unsigned int remaining;
    volatile unsigned int buffered;
    volatile unsigned int transferred;
} udp_transfer_t;


typedef struct usb_setup_struct
{
    uint8_t request;
    uint8_t type;
    uint16_t value;
    uint16_t index;
    uint16_t length;
} udp_setup_t;


typedef struct udp_dev_struct udp_dev_t;

typedef udp_dev_t *udp_t;

typedef bool (*udp_request_handler_t) (void *arg, udp_setup_t *setup);

typedef void (*udp_callback_t) (void *arg, udp_transfer_t *udp_transfer);

typedef uint16_t udp_size_t;

void udp_control_gobble (udp_t udp);

bool udp_halt (udp_t udp, udp_ep_t endpoint, bool halt);

void udp_stall (udp_t udp, udp_ep_t endpoint);

bool udp_idle_p (udp_t udp, udp_ep_t endpoint);

bool udp_halt_p (udp_t udp, udp_ep_t endpoint);

bool udp_read_ready_p (udp_t udp);

udp_size_t udp_read (udp_t udp, void *buffer, udp_size_t length);

udp_size_t udp_write (udp_t udp, const void *buffer, udp_size_t length);

udp_status_t udp_write_async (udp_t udp, 
                              udp_ep_t endpoint,
                              const void *buffer, 
                              unsigned int length, 
                              udp_callback_t callback, 
                              void *arg);

udp_status_t udp_read_async (udp_t udp, 
                             udp_ep_t endpoint,
                             void *buffer, 
                             unsigned int length, 
                             udp_callback_t callback, 
                             void *arg);

bool udp_configured_p (udp_t udp);

bool udp_awake_p (udp_t udp);

void udp_configuration_set (void *arg, udp_transfer_t *ptransfer);

void udp_address_set (void *arg, udp_transfer_t *ptransfer);

void udp_shutdown (void);

void udp_poll (udp_t udp);

udp_t udp_init (udp_request_handler_t handler, void *arg);

#endif

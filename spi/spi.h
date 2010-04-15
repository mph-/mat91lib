/** @file   spi.h
    @author Michael Hayes
    @date   30 July 2007
    @brief  Routines for interfacing to the SPI bus.

    To access a device with this driver spi_init must be first called
    passing a configuration structure, specifying the channel to use,
    clock divisor, and chip select pin to use.  spi_init returns a
    handle that needs to be passed to the other spi functions to
    specify the device to communicate with.

    The AT91SAM7S family has a single SPI controller while the
    AT91SAM7X family has two SPI controllers.  Each SPI controller
    controls an SPI bus on which multiple devices can be connected.
    To specify which device on a bus to communicate with a chip select
    (CS) signal is required, driven from a GPIO port.

    Devices on a SPI bus can be configured differently, in terms of
    clock speed, mode, number of bits, and chip select framing.  To
    avoid having to reconfigure an SPI controller for each device,
    each SPI controller supports 4 channels.  For fast access to a
    device, the SPI controller can automatically drive the chip select
    lines (NPCSx).  Each channel has its own dedicated chip select
    line, for example, channel 3 uses the NPCS3 line.  This signal is
    available on several different GPIO pins and thus needs to
    specified in the configuration structure passed to spi_init.
    
    To support more than four devices on each SPI controller, this
    driver allows a channel to control more than one device.  The
    disadvantage is that this slows SPI transfers for this channel
    since the channel needs to be reconfigured for each transfer and
    automatic chip select driving cannot be used.  

    This driver does not use interrupts.  For the fastest SPI
    transfers, DMA can be used (see spi_dma.h).  This requires the use
    of automatic chip select driving and thus only a single device can
    be interfaced to a channel.

    To support additional SPI controllers, this driver uses logical
    channels, where each channel gets mapped to a physical channel on
    a controller. The first four logical channels are mapped to the
    physical channels on SPI controller 0 while the second four
    logical channels are mapped to the physical channels on SPI
    controller 1.

*/

#ifndef SPI_H
#define SPI_H

#include "config.h"
#include "pio.h"

typedef uint16_t spi_size_t;

typedef int16_t spi_ret_t;

typedef uint8_t spi_channel_t;

typedef uint16_t spi_clock_divisor_t;


/* The AT91SAM7S has a single SPI controller with 4 channels;
   the AT91SAM7S has two SPI controllers each with 4 channels.
   SPI_CHANNELX is a logical channel number.  */

enum {SPI_CHANNEL0 = 0, SPI_CHANNEL1, SPI_CHANNEL2, SPI_CHANNEL3,
      SPI_CHANNEL4, SPI_CHANNEL5, SPI_CHANNEL6, SPI_CHANNEL7};


/* SPI mode settings.  */
typedef enum 
{
    /* Clock normally low, read on rising edge.  */
    SPI_MODE_0 = 0,
    /* Clock normally low, read on falling edge.  */
    SPI_MODE_1 = 1,
    /* Clock normally high, read on falling edge.  */
    SPI_MODE_2 = 2,
    /* Clock normally high, read on rising edge.  */
    SPI_MODE_3 = 3
} spi_mode_t;


/* Data order settings (unused).  */
typedef enum
{
    SPI_DATA_MSB_FIRST = 0,
    SPI_DATA_LSB_FIRST = 1
} spi_data_order_t;


/* SPI chip select framing settings.  */
typedef enum
{
    /* The chip select is only active during a SPI transfer.  */
    SPI_CS_MODE_TOGGLE,
    /* The chip select is active for multiple SPI transfers.  */
    SPI_CS_MODE_FRAME
} spi_cs_mode_t;

#include "spi_private.h"

typedef spi_dev_t *spi_t;


/** Configuration structure.  */
typedef struct
{
    /* Logical channel number.  */
    uint8_t channel;
    /* Clock divisor to use for SPI clock rate.  */
    uint16_t clock_divisor;
    /* GPIO port to use for chip select.  */
    pio_t cs;
    /* SPI mode.  */
    spi_mode_t mode;
    /* Bits per frame.  */
    uint8_t bits;
} spi_cfg_t;


/* Function Prototypes.  */


/** Create new SPI device instance.  */
extern spi_t
spi_init (const spi_cfg_t *cfg);


/** Set number of bits in transfer.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
extern void
spi_bits_set (spi_t spi, uint8_t bits);


/** Set SPI mode.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
extern void
spi_mode_set (spi_t spi, spi_mode_t spi_mode);


/** Set chip select framing mode.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
extern void
spi_cs_mode_set (spi_t spi, spi_cs_mode_t mode);


/** Set the delay (in clocks) after CS asserted before the clock
    starts.  The default is zero.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
extern void
spi_cs_assert_delay_set (spi_t spi, uint16_t delay);


/** Set the delay (in clocks) at end of transmission before CS is
    negated.  The default is 0.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
extern void
spi_cs_negate_delay_set (spi_t spi, uint16_t delay);


/** Set the clock divisor.  This does not take affect until spi_config
    or one of the I/O routines is called.  */
extern void
spi_divisor_set (spi_t spi, spi_clock_divisor_t clock_divisor);


/** Configure SPI with previously specified parameters.  */
extern void
spi_config (spi_t spi);


/** Write a sequence of bytes to SPI.
    @param spi SPI channel to use.
    @param buffer Data buffer to write from.
    @param len Number of bytes to transfer.
    @param terminate True to negate CS when last byte transferred.  */
extern spi_ret_t
spi_write (spi_t spi, const void *buffer, spi_size_t len, bool terminate);


/** Read a sequence of bytes from SPI.
    @param spi SPI channel to use.
    @param buffer Data buffer to read to.
    @param len Number of bytes to transfer.
    @param terminate True to negate CS when last byte transferred.  */
extern spi_ret_t
spi_read (spi_t spi, void *buffer, spi_size_t len, bool terminate);


/** Transfer a sequence of bytes to/from SPI.
    @param spi SPI channel to use.
    @param txbuffer Data buffer to write from.
    @param rxbuffer Data buffer to read into.
    @param len Number of bytes to transfer.
    @param terminate True to negate CS when last byte transferred.  */
extern spi_ret_t
spi_transfer (spi_t spi, const void *txbuffer, void *rxbuffer, 
              spi_size_t len, bool terminate);


/** Return non-zero if there is a character ready to be read.  */
extern bool
spi_read_ready_p (spi_t spi); 


/** Return non-zero if a character can be written without blocking.  */
extern bool
spi_write_ready_p (spi_t spi); 


/** Read character from SPI.  This blocks if nothing
    is available to read.  It sends a dummy word 0.  */
extern uint8_t
spi_getc (spi_t spi); 


/** Write character to SPI.  Ignore received character.  */
extern void 
spi_putc (spi_t spi, char ch);


/** Write character to SPI.  This returns the character just read.  */
extern uint8_t
spi_xferc (spi_t spi, char ch);


extern bool
spi_cs_enable (spi_t spi);


extern bool
spi_cs_disable (spi_t spi);


extern void
spi_shutdown (spi_t spi);


extern void
spi_wakeup (spi_t spi);

#endif

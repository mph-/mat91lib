/** @file   spi.h
    @author Michael Hayes
    @date   30 July 2007
    @brief  Routines for interfacing to the SPI bus.

    SPI can be simple but gets complicated when performance is
    required, especially with automatic driving of the chip select
    signals.

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

#ifdef __cplusplus
extern "C" {
#endif

typedef size_t spi_size_t;

typedef ssize_t spi_ret_t;

typedef uint8_t spi_channel_t;

typedef uint16_t spi_clock_divisor_t;

typedef uint32_t spi_clock_speed_t;


/* The AT91SAM7S has a single SPI controller with 4 channels;
   the AT91SAM7X has two SPI controllers each with 4 channels.
   SPI_CHANNELX is a logical channel number.  */

enum {SPI_CHANNEL0 = 0, SPI_CHANNEL1, SPI_CHANNEL2, SPI_CHANNEL3,
      SPI_CHANNEL4, SPI_CHANNEL5, SPI_CHANNEL6, SPI_CHANNEL7};


/** Number of channels per controller.  */
#define SPI_CHANNELS_NUM 4

/** Bitmask to select desired channel.  */
#define SPI_CHANNEL_MASK(channel) (0x0f ^ BIT (channel & (SPI_CHANNELS_NUM - 1)))


/* SPI mode settings.  */
typedef enum
{
    /* Clock normally low, read on rising edge.  Mode 0, 0.  */
    SPI_MODE_0 = 0,
    /* Clock normally low, read on falling edge.  */
    SPI_MODE_1 = 1,
    /* Clock normally high, read on falling edge.  */
    SPI_MODE_2 = 2,
    /* Clock normally high, read on rising edge.  Mode 1, 1.  */
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
    SPI_CS_MODE_FRAME,
    /* The chip select stays high during SPI transfer.  */
    SPI_CS_MODE_HIGH,
    /* The chip select is driven externally (say by a timer/counter).  */
    SPI_CS_MODE_EXTERNAL,
} spi_cs_mode_t;


/** SPI configuration structure.  */
typedef struct
{
    /* Logical channel number.  For automatic chip-select (CS) driving
       this must correspond to the specified CS.  This field could be
       determined from the CS field except when the specified CS
       cannot be automatically driven.  */
    uint8_t channel;
    /* Clock speed in kHz (maximum).  */
    spi_clock_speed_t clock_speed_kHz;
    /* PIO port to use for chip select.  */
    pio_t cs;
    /* SPI mode.  */
    spi_mode_t mode;
    /* SPI chip select mode (toggle default).  */
    spi_cs_mode_t cs_mode;
    /* Bits per frame.  */
    uint8_t bits;
} spi_cfg_t;


/** Include definition of data structures required for the driver
    implementation.  Do not use.  */
#include "spi_private.h"


/** Define datatype for handle to SPI functions.  */
typedef spi_dev_t *spi_t;


typedef struct
{
    const void *txbuffer;
    void *rxbuffer;
    spi_size_t size;
} spi_transfer_t;


/* Function prototypes.  */


/** Create new SPI device instance.  */
spi_t
spi_init (const spi_cfg_t *cfg);


/** Write a sequence of bytes to SPI.  This is just a wrapper for spi_transfer.
    @param spi SPI channel to use.
    @param buffer Data buffer to write from.
    @param len Number of bytes to transfer.
    @param terminate True to negate CS when last byte transferred.  */
spi_ret_t
spi_write (spi_t spi, const void *buffer, spi_size_t len, bool terminate);


/** Read a sequence of bytes from SPI.  This is just a wrapper for spi_transfer.
    @param spi SPI channel to use.
    @param buffer Data buffer to read to.
    @param len Number of bytes to transfer.
    @param terminate True to negate CS when last byte transferred.  */
spi_ret_t
spi_read (spi_t spi, void *buffer, spi_size_t len, bool terminate);


/** Transfer a sequence of bytes to/from SPI.
    @param spi SPI channel to use.
    @param txbuffer Data buffer to write from (or NULL just for reading).
    @param rxbuffer Data buffer to read into (or NULL just for writing).
    @param len Number of bytes to transfer.
    @param terminate True to negate CS when last byte transferred.
    @return Number of bytes transferred.
*/
spi_ret_t
spi_transfer (spi_t spi, const void *txbuffer, void *rxbuffer,
              spi_size_t len, bool terminate);


/** Transfer a sequence of bytes to/from SPI using multiple buffers.
    @param spi SPI channel to use.
    @param transfer Array of transmit/receive buffers and sizes.
    @param size Size of transfer array.
    @return Number of bytes transferred.
 */
spi_ret_t
spi_transact (spi_t spi, spi_transfer_t *transfer, uint8_t size);


/** Return non-zero if there is a character ready to be read.  */
bool
spi_read_ready_p (spi_t spi);


/** Return non-zero if a character can be written without blocking.  */
bool
spi_write_ready_p (spi_t spi);


/** Read character from SPI.  This blocks if nothing
    is available to read.  It sends a dummy word 0.  */
uint8_t
spi_getc (spi_t spi);


/** Write character to SPI.  Ignore received character.  */
void
spi_putc (spi_t spi, char ch);


/** Write character to SPI and return the character read in response.  */
uint8_t
spi_xferc (spi_t spi, char ch);


/** Shutdown SPI peripheral to save power.  */
void
spi_shutdown (spi_t spi);


/* The following functions are for advanced use only.  */


/** Change number of bits in transfer.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
void
spi_bits_set (spi_t spi, uint8_t bits);


/** Change SPI mode.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
void
spi_mode_set (spi_t spi, spi_mode_t spi_mode);


/** Change chip select framing mode.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
void
spi_cs_mode_set (spi_t spi, spi_cs_mode_t mode);


/** Change the clock divisor.  This does not take affect until spi_config
    or one of the I/O routines is called.  */
void
spi_clock_divisor_set (spi_t spi, spi_clock_divisor_t clock_divisor);


/** Change the clock speed.  This does not take affect until spi_config
    or one of the I/O routines is called. The desired speed is given in kHz.
    N.B., the clock divisor calculated by this routine is rounded up, i.e., the
    clock will be slower than or equal to the given speed. It is also clipped
    to the allowable range of divisors. The actual clock speed (also in kHz) is
    returned.  */
spi_clock_speed_t
spi_clock_speed_kHz_set (spi_t spi, spi_clock_speed_t clock_speed);


/** Set the delay (in clocks) after CS asserted before the clock
    starts.  The default is zero.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
void
spi_cs_setup_set (spi_t spi, uint16_t delay);


/** Set the delay (in clocks) at end of transmission before CS is
    negated.  The default is 0.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
void
spi_cs_hold_set (spi_t spi, uint16_t delay);


/** Enable auto chip select.  Return zero if not possible with selected pio.  */
bool
spi_cs_auto_enable (spi_t spi);


/** Disable auto chip select.  */
void
spi_cs_auto_disable (spi_t spi);


/** Configure SPI with previously specified parameters.  */
void
spi_config (spi_t spi);


/** Force assertion of chip select (set low).   This is useful
    when sleeping to avoid inadvertely powering the SPI device.
    This should only be done when all SPI transfers have finished.  */
void
spi_cs_assert (spi_t spi);


/** Force negation of chip select (set high).  */
void
spi_cs_negate (spi_t spi);

#ifdef __cplusplus
}
#endif


#endif

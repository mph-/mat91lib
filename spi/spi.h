/** @file   spi.h
    @author Michael Hayes
    @date   30 July 2007
    @brief  Routines for interfacing to the SPI bus.
*/

#ifndef SPI_H
#define SPI_H

#include "config.h"
#include "port.h"

/* Macros for behind the scenes manipulation of the SPI peripheral.  These
   should not be used for normal use.  */
#ifdef HOSTED
#define SPI_READY_P() (HOSTED || (AT91C_BASE_SPI->SPI_SR & AT91C_SPI_RDRF))
#else
#define SPI_READY_P() (AT91C_BASE_SPI->SPI_SR & AT91C_SPI_RDRF)
#endif

#define SPI_TXEMPTY_P() (AT91C_BASE_SPI->SPI_SR & AT91C_SPI_TXEMPTY)

#define SPI_CHANNEL_MASK(channel) (0x0f ^ BIT (channel))



typedef uint16_t spi_size_t;

typedef int16_t spi_ret_t;

typedef uint8_t spi_channel_t;

typedef uint16_t spi_clock_divisor_t;


enum {SPI_CHANNEL0 = 0, SPI_CHANNEL1, SPI_CHANNEL2, SPI_CHANNEL3};


/* SPI Mode Settings.  */
typedef enum {SPI_MODE_0 = 0,
              SPI_MODE_1 = 1,
              SPI_MODE_2 = 2,
              SPI_MODE_3 = 3
} spi_mode_t;


/* Data order settings.  */
typedef enum {SPI_DATA_MSB_FIRST = 0,
              SPI_DATA_LSB_FIRST = 1
} spi_data_order_t;


typedef enum
{
    SPI_CS_MODE_TOGGLE,
    SPI_CS_MODE_FRAME
} spi_cs_mode_t;


/* The fields in this structure should be considered private.  */
typedef struct
{
    uint8_t channel;
    uint8_t bits;
    uint16_t clock_divisor;
    uint16_t cs_assert_delay;
    uint16_t cs_negate_delay;
    spi_mode_t mode;
    port_cfg_t cs;
    spi_cs_mode_t cs_mode;
    bool cs_active;
    bool cs_auto;
} spi_dev_t;

typedef spi_dev_t *spi_t;


/** Configuration structure.  */
typedef struct
{
    uint8_t channel;
    uint16_t clock_divisor;
    port_cfg_t cs;
} spi_cfg_t;


/** Macro to initialise configuration structure.  */
#define SPI_CFG(CHANNEL, DIVISOR, CS_PORT, CS_PORTBIT) \
{(CHANNEL), (DIVISOR), PORT_CFG (CS_PORT, CS_PORTBIT)}



/* Function Prototypes.  */


/** Create new SPI device instance.  */
extern spi_t
spi_init (const spi_cfg_t *cfg);


extern void
spi_shutdown (spi_t spi);


extern void
spi_wakeup (spi_t spi);


extern bool
spi_cs_enable (spi_t spi);


extern bool
spi_cs_disable (spi_t spi);


/** Set number of bits in transfer.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
extern void
spi_bits_set (spi_t spi, uint8_t bits);


/** Set SPI mode.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
extern void
spi_mode_set (spi_t spi, spi_mode_t spi_mode);


/** Set chip select mode.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
extern void
spi_cs_mode_set (spi_t spi, spi_cs_mode_t mode);


/** Set the delay (in clocks) after CS asserted before the clock
    starts.  The default is zero.  This does not take affect until
    spi_config or one of the I/O routines is called.  */
extern void
spi_cs_assert_delay_set (spi_t spi, uint16_t delay);


/** Set the delay (in clocks) at end of transmission before CS negated.
    The default is 0.  This does not take affect
    until spi_config or one of the I/O routines is called.  */
extern void
spi_cs_negate_delay_set (spi_t spi, uint16_t delay);


/** Set the clock divisor.  This does not take affect until spi_config
    or one of the I/O routines is called.  */
extern void
spi_divisor_set (spi_t spi, spi_clock_divisor_t clock_divisor);


/** Configure SPI with previously specified parameters.  */
extern void
spi_config (spi_t spi);


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


extern spi_ret_t
spi_write (spi_t spi, const void *buffer, spi_size_t len, bool terminate);


extern spi_ret_t
spi_read (spi_t spi, void *buffer, spi_size_t len, bool terminate);


extern spi_ret_t
spi_transfer (spi_t spi, const void *txbuffer, void *rxbuffer, 
              spi_size_t len, bool terminate);

#endif

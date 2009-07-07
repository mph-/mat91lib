/** @file   spi_private.h
    @author Michael Hayes
    @date   30 July 2007
    @brief  Private data structure for SPI routines. 
*/

#ifndef SPI_PRIVATE_H
#define SPI_PRIVATE_H

/* The fields in this structure are private.  Do not use.  */
typedef struct spi_dev_struct
{
    uint8_t channel;
    uint8_t bits;
    uint16_t clock_divisor;
    /* Delay from CS asserted (set low) until SPI transfer starts.  */
    uint16_t cs_assert_delay;
    /* Delay from CS negated (set high) after SPI transfer stops.  */
    uint16_t cs_negate_delay;
    spi_mode_t mode;
    /* The GPIO port that drives the CS.  */
    port_cfg_t cs;
    spi_cs_mode_t cs_mode;
    bool cs_active;
    bool cs_auto;
} spi_dev_t;


#endif

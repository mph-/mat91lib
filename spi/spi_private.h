/** @file   spi_private.h
    @author Michael Hayes
    @date   30 July 2007
    @brief  Private data structure for SPI routines. 
*/

#ifndef SPI_PRIVATE_H
#define SPI_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"

/* The fields in this structure are private.  Do not use.  */
typedef struct spi_dev_struct
{
    Spi *base;
    uint8_t channel;
    uint8_t bits;
    uint16_t clock_divisor;
    /* Delay from CS asserted (set low) until SPI transfer starts.  */
    uint16_t cs_setup;
    /* Delay from CS negated (set high) after SPI transfer stops.  */
    uint16_t cs_hold;
    spi_mode_t mode;
    /* The PIO port that drives the CS.  */
    pio_t cs;
    pio_config_t cs_config;
    spi_cs_mode_t cs_mode;
    bool cs_active;
} spi_dev_t;



#ifdef __cplusplus
}
#endif    
#endif


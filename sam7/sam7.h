#ifndef SAM7_H
#define SAM7_H

/* This is fixed.  */
#define PIT_CLOCK_DIVISOR 16


#if defined (__AT91SAM7S512__)
#include "AT91SAM7S512.h"
#elif defined (__AT91SAM7S256__)
#include "AT91SAM7S256.h"
#elif defined (__AT91SAM7X256__)
#include "AT91SAM7X256.h"
#elif defined (__AT91SAM7S128__)
#include "AT91SAM7S128.h"
#elif defined (__AT91SAM7S64__)
#include "AT91SAM7S64.h"
#elif defined (__AT91SAM7S32__)
#include "AT91SAM7S32.h"
#else
#error "MCU type not defined"
#endif



/* Provide a compatibility layer with Atmel's ASF definitions.  */

#define PMC AT91C_BASE_PMC
#define SPI AT91C_BASE_SPI
#define PWMC AT91C_BASE_PWMC
#define USART0 AT91C_BASE_US0
#define USART1 AT91C_BASE_US1
#define UDP AT91C_BASE_UDP
#define TC0 AT91C_BASE_TC0
#define TC1 AT91C_BASE_TC1
#define TC2 AT91C_BASE_TC2
#define PIOA AT91C_BASE_PIOA
#ifdef AT91C_BASE_PIOB
#define PIOB AT91C_BASE_PIOB
#endif
#define ADC AT91C_BASE_ADC
#define PDC_SPI AT91C_BASE_PDC_SPI
#define CKGR AT91C_BASE_CKGR
#define AIC AT91C_BASE_AIC
#define RSTC AT91C_BASE_RSTC
#define VREG AT91C_BASE_VREG
#define WDTC AT91C_BASE_WDTC
#define MC AT91C_BASE_MC

#define Pwm AT91S_PWMC_CH


#define PWMC_CH0 AT91C_BASE_PWMC_CH0
#define PWMC_CH1 AT91C_BASE_PWMC_CH1
#define PWMC_CH2 AT91C_BASE_PWMC_CH2
#define PWMC_CH3 AT91C_BASE_PWMC_CH3

#define ADC_CR_SWRST AT91C_ADC_SWRST
#define ADC_CR_START AT91C_ADC_START
#define ADC_MR_SLEEP AT91C_ADC_SLEEP
#define ADC_MR_LOWRES AT91C_ADC_LOWRES

#define US_TXEMPTY AT91C_US_TXEMPTY
#define US_RXRDY AT91C_US_RXRDY
#define US_TXRDY AT91C_US_TXRDY

#endif

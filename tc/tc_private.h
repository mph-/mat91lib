/** @file   tc_private.h
    @author Michael Hayes
    @date   30 March 2011
    @brief  Private data structure for TC routines. 
*/

#ifndef TC_PRIVATE_H
#define TC_PRIVATE_H

#include "config.h"

typedef struct
{
    TcChannel *base;
    volatile tc_counter_t captureA;
    volatile tc_counter_t captureB;
    volatile tc_counter_t overflows;
} tc_dev_t;

#endif

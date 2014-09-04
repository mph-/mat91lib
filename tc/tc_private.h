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
    tc_counter_t captureA;
    tc_counter_t captureB;
    tc_counter_t overflows;
} tc_dev_t;

#endif

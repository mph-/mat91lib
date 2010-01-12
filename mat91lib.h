/** @file   mat91lib.h
    @author M. P. Hayes, UCECE
    @date   23 March 2008
    @brief  Datatypes and macros required for mat91lib.
*/
#ifndef MAT91LIB_H
#define MAT91LIB_H

#include <stdint.h>
enum {false = 0, true = 1};
typedef uint8_t bool;


#ifndef BIT
#define BIT(X) (1U << (X))
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(ARRAY) (sizeof(ARRAY) / sizeof (ARRAY[0]))
#endif

#ifndef __UNUSED__
#define  __UNUSED__ __attribute__ ((unused))
#endif

#ifndef __unused__
#define  __unused__ __attribute__ ((unused))
#endif


#ifndef _DOXYGEN_
#define __packed__ __attribute__((packed))
#else
#define __packed__
#endif

#define __inline static inline


#if defined (AT91SAM7S512)
#include "AT91SAM7S512.h"
#elif defined (AT91SAM7S256)
#include "AT91SAM7S256.h"
#elif defined (AT91SAM7X256)
#include "AT91SAM7X256.h"
#elif defined (AT91SAM7S128)
#include "AT91SAM7S128.h"
#elif defined (AT91SAM7S64)
#include "AT91SAM7S64.h"
#elif defined (AT91SAM7S32)
#include "AT91SAM7S32.h"
#else
#error "device type not defined"
#endif


#endif

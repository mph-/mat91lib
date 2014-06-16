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


#ifndef __always_inline__
#define  __always_inline__ __attribute__ ((always_inline))
#endif


#ifndef _DOXYGEN_
#define __packed__ __attribute__((packed))
#else
#define __packed__
#endif

#define __inline static inline

#if defined (__SAM7__)
#include "sam7.h"
#elif defined (__SAM4S__)
#include "sam4s.h"
#else
#error "MCU family not defined"
#endif

#endif

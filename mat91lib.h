/** @file   mat91lib.h
    @author M. P. Hayes, UCECE
    @date   23 March 2008
    @brief  Datatypes and macros required for mat91lib.
*/
#ifndef MAT91LIB_H
#define MAT91LIB_H

#include <stdint.h>
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

#ifndef HOSTED
#define HOSTED 0
#endif

#endif

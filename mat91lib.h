/** @file   mat91lib.h
    @author M. P. Hayes, UCECE
    @date   23 March 2008
    @brief  Datatypes and macros required for mat91lib.
*/
#ifndef MAT91LIB_H
#define MAT91LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

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
#define  __always_inline__ inline __attribute__ ((always_inline))
#endif


#ifndef _DOXYGEN_
#define __packed__ __attribute__((packed))
#else
#define __packed__
#endif

#if defined (__SAM7__)
#include "sam7.h"
#elif defined (__SAM4S__)
#include "sam4s.h"
#else
#error "MCU family not defined"
#endif

#define F_CPU_UL ((unsigned long)F_CPU)


#define LOG2_1(n) (((n) >= 2) ? 1 : 0)
#define LOG2_2(n) (((n) >= 1 << 2) ? (2 + LOG2_1((n) >> 2)) : LOG2_1(n))
#define LOG2_4(n) (((n) >= 1 << 4) ? (4 + LOG2_2((n) >> 4)) : LOG2_2(n))
#define LOG2_8(n) (((n) >= 1 << 8) ? (8 + LOG2_4((n) >> 8)) : LOG2_4(n))
#define LOG2(n)  (((n) >= 1 << 16) ? (16 + LOG2_8((n) >> 16)) : LOG2_8(n))



#ifdef __cplusplus
}
#endif
#endif

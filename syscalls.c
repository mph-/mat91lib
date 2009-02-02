/** @file   syscalls.c
    @author Michael Hayes
    @date   18 February 2008
    @brief  This file contains a collection of stub functions to replace those
    used by the C library newlib.  Primarily, it provides functionality
    to redirect standard I/O.  */

#include "config.h"
#include <errno.h>
#include <unistd.h>


static int
stdio_write_block (void *stream, const char *buffer, int size);

static int
stdio_read_block (void *stream, char *buffer, int size);


static void (*stdio_putc) (void *stream, int ch) = 0;
static int (*stdio_getc) (void *stream) = 0;
static int (*stdio_write) (void *stream, const char *buffer, int size) = stdio_write_block;
static int (*stdio_read) (void *stream, char *buffer, int size) = stdio_read_block;
static void *stdio_stream = 0;

extern char _heap_start__;    /** Start of heap.  */


void
stdio_redirect (void (*putc1) (void *stream, int ch),
                int (*getc1) (void *stream), void *stream)
{
    stdio_putc = putc1;
    stdio_getc = getc1;
    stdio_stream = stream;
}


void
stdio_redirect_block (int (*write1) (void *stream, const char *buffer, int size),
                      int (*read1) (void *stream, char *buffer, int size),
                      void *stream)
{
    stdio_write = write1;
    stdio_read = read1;
    stdio_stream = stream;
}


/* Helper write routine if no block write exists.  */
static int
stdio_write_block (void *stream, const char *buffer, int size)
{
    int i;
    
    if (!stdio_putc)
    {
        errno = ENODEV;
        return -1;
    }

    for (i = 0; i < size; i++)
        stdio_putc (stream, *buffer++);

    return size;
}


/* Helper read routine if no block read exists.  */
static int
stdio_read_block (void *stream, char *buffer, int size)
{
    int i;
    
    if (!stdio_getc)
    {
        errno = ENODEV;
        return -1;
    }

    for (i = 0; i < size; i++)
        *buffer++ = stdio_getc (stream);

    return size;
}


int
_read (int file __UNUSED__, char *buffer, int size)
{
    if (!stdio_read)
    {
        errno = ENODEV;
        return -1;
    }
    return stdio_read (stdio_stream, buffer, size);
}


int
_lseek (int file __UNUSED__, int ptr __UNUSED__, int dir __UNUSED__)
{
    return -1;
}


int
_write (int file __UNUSED__, char *buffer, int size)
{
    if (!stdio_write)
    {
        errno = ENODEV;
        return -1;
    }
    return stdio_write (stdio_stream, buffer, size);
}


int
_open (const char *path __UNUSED__, int flags __UNUSED__, ...)
{
    return 0;
}


int
_close (int file __UNUSED__)
{
    return 0;
}


int
_kill (int pid __UNUSED__, int sig __UNUSED__)
{
    return -1;
}


void
_exit (int status __UNUSED__)
{
    while (1)
        continue;
}


int
_getpid (int n __UNUSED__)
{
    return 1;
}


caddr_t
_sbrk (int incr)
{
    /* Defined by the linker.  */
    extern char end __asm ("end");
   /* Register name faking - works in collusion with the linker.  */
    register char *stack_ptr __asm ("sp");
    static char *heap_end;
    char *prev_heap_end;
    
    if (heap_end == NULL)
        heap_end = & end;
    
    prev_heap_end = heap_end;
  
    if (heap_end + incr > stack_ptr)
    {
        errno = ENOMEM;
        return (caddr_t) -1;
    }
    
    heap_end += incr;

    return (caddr_t) prev_heap_end;
}


int
_fstat (int file __UNUSED__, void *st __UNUSED__)
{
    return -1;
}


int _stat (const char *fname __UNUSED__, void *st __UNUSED__)
{
    return -1;
}


int
_link (void)
{
    return -1;
}


int
_unlink (const char *path __UNUSED__)
{
    return -1;
}


void
_raise (void)
{
}


int
_gettimeofday (void *tp __UNUSED__, void *tzp __UNUSED__)
{
    return -1;
}


/* Return a clock that ticks at 100Hz.  */
clock_t 
_times (void *tp __UNUSED__)
{
    return -1;
};


int
_isatty (int fd)
{
    return (fd <= 2) ? 1 : 0;  /* one of stdin, stdout, stderr */
}


int
_system (const char *s __UNUSED__)
{
    return -1;
}


int
_rename (const char *oldpath __UNUSED__, const char *newpath __UNUSED__)
{
    errno = ENOSYS;
    return -1;
}

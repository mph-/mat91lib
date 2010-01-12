/** @file   syscalls.c
    @author Michael Hayes
    @date   18 February 2008
    @brief  This file contains a collection of stub functions to replace those
    used by the C library newlib.  Primarily, it provides functionality
    to redirect standard I/O and to interface to a file system.  */

#include "config.h"
#include <errno.h>
#include <unistd.h>

#include "sys.h"


#ifndef SYS_FILE_NUM
#define SYS_FILE_NUM 4
#endif

#define SYS_FD_NUM (SYS_FILE_NUM + 3)


typedef struct sys_file_struct
{
    sys_dev_t *dev;
    void *file;
} sys_file_t;

static sys_dev_t stdin_dev;
static sys_dev_t stdout_dev;
static sys_dev_t stderr_dev;

static sys_file_t sys_files[SYS_FD_NUM] =
{
    {.dev = &stdin_dev, .file = 0},
    {.dev = &stdout_dev, .file = 0},
    {.dev = &stderr_dev, .file = 0}
};

static sys_fs_t *sys_fs;
static void *sys_fs_arg;
static void (*stdio_putc) (void *stream, int ch) = 0;
static int (*stdio_getc) (void *stream) = 0;



extern char _heap_start__;    /** Start of heap.  */


/* Helper write routine if no block write exists.  */
static ssize_t
stdio_write_block (void *stream, const char *buffer, size_t size)
{
    size_t i;
    
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
static ssize_t
stdio_read_block (void *stream, char *buffer, size_t size)
{
    size_t i;
    
    if (!stdio_getc)
    {
        errno = ENODEV;
        return -1;
    }

    for (i = 0; i < size; i++)
        *buffer++ = stdio_getc (stream);

    return size;
}


/* This is an old compatibility function for character based I/O.  */
void
stdio_redirect (void (*putc1) (void *stream, int ch),
                int (*getc1) (void *stream), void *stream)
{
    stdio_putc = putc1;
    stdio_getc = getc1;
    sys_redirect_stdin (stdio_read_block, stream);
    sys_redirect_stdout (stdio_write_block, stream);
    sys_redirect_stderr (stdio_write_block, stream);
}


void
sys_redirect_stdin (sys_read_t read1, void *file)
{
    sys_files[0].dev->read = read1;
    sys_files[0].file = file;
}


void
sys_redirect_stdout (sys_write_t write1, void *file)
{
    sys_files[1].dev->write = write1;
    sys_files[1].file = file;
}


void
sys_redirect_stderr (sys_write_t write1, void *file)
{
    sys_files[2].dev->write = write1;
    sys_files[2].file = file;
}


ssize_t
_read (int fd, char *buffer, size_t size)
{
    if ((fd >= SYS_FD_NUM) || !sys_files[fd].dev->read)
    {
        errno = ENODEV;
        return -1;
    }

    return sys_files[fd].dev->read (sys_files[fd].file, buffer, size);
}


off_t
_lseek (int fd, off_t offset, int dir)
{
    if ((fd >= SYS_FD_NUM) || !sys_files[fd].dev->lseek)
    {
        errno = ENODEV;
        return -1;
    }

    return sys_files[fd].dev->lseek (sys_files[fd].file, offset, dir);
}


ssize_t
_write (int fd, char *buffer, size_t size)
{
    if ((fd >= SYS_FD_NUM) || !sys_files[fd].dev->write)
    {
        errno = ENODEV;
        return -1;
    }

    return sys_files[fd].dev->write (sys_files[fd].file, buffer, size);
}


int
_open (const char *path, int flags, ...)
{
    void *arg;
    int fd;

    /* This is not called for stdin, stdout, stderr.  */

    for (fd = 3; fd < SYS_FD_NUM; fd++)
    {
        if (!sys_files[fd].dev)
            break;
    }
    if (fd == SYS_FD_NUM)
    {
        errno = ENFILE;
        return -1;
    }

    if (!sys_fs || !sys_fs->dev || !sys_fs->dev->open)
    {
        errno = EACCES;
        return -1;
    }

    arg = sys_fs->dev->open (sys_fs_arg, path, flags);
    if (!arg)
    {
        errno = EACCES;
        return -1;
    }
    sys_files[fd].dev = sys_fs->dev;
    sys_files[fd].file = arg;

    return fd;
}


int
_close (int fd __UNUSED__)
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
   /* Register name faking using collusion with the linker.  */
    register char *stack_ptr __asm ("sp");
    static char *heap_end;
    char *prev_heap_end;
    
    if (heap_end == NULL)
        heap_end = &end;
    
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
_fstat (int fd __UNUSED__, void *st __UNUSED__)
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
_unlink (const char *path)
{
    if (!sys_fs || !sys_fs->dev || !sys_fs->unlink)
    {
        errno = EACCES;
        return -1;
    }
    return sys_fs->unlink (path);
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


/* Register a file system for file I/O.  Currently only a single
   file system is supported.  */
void
sys_fs_register (sys_fs_t *fs, void *arg)
{
    sys_fs = fs;
    sys_fs_arg = arg;
}

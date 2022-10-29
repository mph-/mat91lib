/** @file   syscalls.c
    @author Michael Hayes
    @date   18 February 2008
    @brief  This file contains a collection of stub functions to replace those
    used by the C library newlib.  Primarily, it provides functionality
    to redirect standard I/O and to interface to a file system.
    It assumes that all devices are non-blocking.
 */

#include "config.h"
#include "delay.h"
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "sys.h"


/* Maximum number of open files.  */
#ifndef SYS_FILE_NUM
#define SYS_FILE_NUM 4
#endif


/* Maximum number of file descriptors, including stdin, stdout, stderr.  */
#define SYS_FD_NUM (SYS_FILE_NUM + 3)


/* Maximum number of file systems.  */
#ifndef SYS_FS_NUM
#define SYS_FS_NUM 1
#endif


typedef struct sys_file_struct
{
    sys_file_ops_t *file_ops;
    void *file;
} sys_file_t;


typedef struct sys_device_struct
{
    const char *name;
    sys_file_ops_t *file_ops;
    void *arg;
    struct sys_device_struct *next;
} sys_device_t;


static sys_file_ops_t stdin_file_ops;
static sys_file_ops_t stdout_file_ops;
static sys_file_ops_t stderr_file_ops;

/* Array of file descriptors.  */
static sys_file_t sys_files[SYS_FD_NUM] =
{
    {.file_ops = &stdin_file_ops, .file = 0},
    {.file_ops = &stdout_file_ops, .file = 0},
    {.file_ops = &stderr_file_ops, .file = 0}
};

/* Linked list of devices.  */
static sys_device_t *sys_devices = 0;

/* Array of file systems.  */
static sys_fs_t *sys_fs[SYS_FS_NUM];

static void (*stdio_putc) (void *stream, int ch) = 0;
static int (*stdio_getc) (void *stream) = 0;

/** Start of heap.  */
extern char _heap_start__;


/* Helper write routine for old style stdio redirection.  */
static ssize_t
stdio_write_block (void *stream, const void *buffer, size_t size)
{
    size_t i;
    const char *src = buffer;

    if (!stdio_putc)
    {
        errno = ENODEV;
        return -1;
    }

    for (i = 0; i < size; i++)
        stdio_putc (stream, *src++);

    return size;
}


/* Helper read routine for old style stdio redirection.  */
static ssize_t
stdio_read_block (void *stream, void *buffer, size_t size)
{
    size_t i;
    char *dst = buffer;

    if (!stdio_getc)
    {
        errno = ENODEV;
        return -1;
    }

    for (i = 0; i < size; i++)
        *dst++ = stdio_getc (stream);

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
sys_redirect (unsigned int fd, sys_read_t read1, sys_write_t write1, void *arg)
{
    if (read1)
        sys_files[fd].file_ops->read = read1;
    if (write1)
        sys_files[fd].file_ops->write = write1;
    sys_files[fd].file = arg;
}


void
sys_redirect_stdin (sys_read_t read1, void *arg)
{
    sys_redirect (0, read1, 0, arg);
}


void
sys_redirect_stdout (sys_write_t write1, void *arg)
{
    sys_redirect (1, 0, write1, arg);
}


void
sys_redirect_stderr (sys_write_t write1, void *arg)
{
    sys_redirect (2, 0, write1, arg);
}


static const char *
sys_fs_find (const char *pathname, sys_fs_t **pfs)
{
    /* TODO, map pathname to a fs.  For now assume that first file
       system is mounted as /.  */

    *pfs = sys_fs[0];

    if (*pathname == '/')
        return pathname + 1;

    return pathname;
}


ssize_t
_read (int fd, char *buffer, size_t size)
{
    if ((fd >= SYS_FD_NUM) || !sys_files[fd].file_ops->read)
    {
        errno = ENODEV;
        return -1;
    }

    return sys_files[fd].file_ops->read (sys_files[fd].file, buffer, size);
}


off_t
_lseek (int fd, off_t offset, int whence)
{
    if ((fd >= SYS_FD_NUM) || !sys_files[fd].file_ops->lseek)
    {
        errno = ENODEV;
        return -1;
    }

    return sys_files[fd].file_ops->lseek (sys_files[fd].file, offset, whence);
}


ssize_t
_write (int fd, char *buffer, size_t size)
{
    if ((fd >= SYS_FD_NUM) || !sys_files[fd].file_ops->write)
    {
        errno = ENODEV;
        return -1;
    }

    return sys_files[fd].file_ops->write (sys_files[fd].file, buffer, size);
}


static int
sys_next_fd (void)
{
    int fd;

    for (fd = 3; fd < SYS_FD_NUM; fd++)
    {
        if (!sys_files[fd].file_ops)
            break;
    }
    if (fd == SYS_FD_NUM)
    {
        errno = ENFILE;
        return -1;
    }

    return fd;
}


static int
sys_file_open (const char *pathname, int flags)
{
    sys_fs_t *fs;
    void *arg;
    int fd;

    fd = sys_next_fd ();
    if (fd < 0)
        return fd;

    pathname = sys_fs_find (pathname, &fs);

    if (!fs || !fs->file_ops || !fs->file_ops->open)
    {
        errno = EACCES;
        return -1;
    }

    arg = fs->file_ops->open (fs->handle, pathname, flags);
    if (!arg)
    {
        errno = EACCES;
        return -1;
    }
    sys_files[fd].file_ops = (sys_file_ops_t *)fs->file_ops;
    sys_files[fd].file = arg;

    return fd;
}


int
sys_attach (sys_file_ops_t *file_ops, void *arg)
{
    int fd;

    fd = sys_next_fd ();
    if (fd < 0)
        return fd;

    sys_files[fd].file_ops = file_ops;
    sys_files[fd].file = arg;

    return fd;
}


static int
sys_device_open (const char *pathname, int flags)
{
    sys_device_t *device;

    /* Traverse list of devices.  */
    for (device = sys_devices; device; device = device->next)
    {
        if (strcmp (device->name, pathname) == 0)
            return sys_attach (device->file_ops, device->arg);
    }
    /* No device found.  */
    errno = ENXIO;
    return -1;
}


int
_open (const char *pathname, int flags, ...)
{
    /* This is not called for stdin, stdout, stderr.  */

    if (strncmp (pathname, "/dev/", 5) == 0)
        return sys_device_open (pathname, flags);

    return sys_file_open (pathname, flags);
}


int
_close (int fd)
{
    int ret;

    if ((fd >= SYS_FD_NUM) || !sys_files[fd].file_ops->close)
    {
        errno = ENODEV;
        return -1;
    }

    ret = sys_files[fd].file_ops->close (sys_files[fd].file);

    sys_files[fd].file_ops = 0;
    sys_files[fd].file = 0;
    return ret;
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

    if (heap_end == 0)
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
_unlink (const char *pathname)
{
    sys_fs_t *fs;

    pathname = sys_fs_find (pathname, &fs);

    /* TODO: select fs based on pathname.  */
    if (!fs || !fs->fs_ops || !fs->fs_ops->unlink)
    {
        errno = EACCES;
        return -1;
    }
    return fs->fs_ops->unlink (fs->handle, pathname);
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


/* Return a clock that ticks at 100 Hz.  */
clock_t
_times (void *tp __UNUSED__)
{
    return -1;
};


int
_isatty (int fd)
{
    return (fd <= 2) ? 1 : 0;  /* One of stdin, stdout, stderr.  */
}


int
_system (const char *s __UNUSED__)
{
    return -1;
}


int
_rename (const char *oldpath __UNUSED__, const char *newpath __UNUSED__)
{
    /* TODO: Vector to fs op.  Note, both pathnames must be on the
       same file system otherwise need to return EXDEV.  */

    errno = ENOSYS;
    return -1;
}


/* Mount a file system for file I/O.  */
bool
sys_mount (sys_fs_t *fs, int flags)
{
    int i;

    for (i = 0; i < SYS_FS_NUM; i++)
    {
        if (!sys_fs[i])
            break;
    }

    if (i == SYS_FS_NUM)
        return 0;

    sys_fs[i] = fs;
    sys_fs[i]->flags = flags;
    return 1;
}


/** Register a device with a devicename, say /dev/usart0,
    and record the file_ops and arg (say device handle).
    The device can then be opened using open.  */
int
sys_device_register (const char *devicename, const sys_file_ops_t *file_ops,
                     void *arg)
{
    sys_device_t *device;
    sys_device_t *new;

    new = malloc (sizeof (*new));
    new->name = devicename;
    new->file_ops = (void *)file_ops;
    new->arg = arg;
    new->next = 0;

    if (! sys_devices)
    {
        sys_devices = new;
    }
    else
    {
        /* Traverse list of devices.  */
        for (device = sys_devices; device->next; device = device->next)
        {
            /* Perhaps check if device already registered?  */
            continue;
        }
        device->next = new;
    }

    return 0;
}


/* Helper read function for device drivers.  The timeout is reset for
   every read without fail.  */
ssize_t
sys_read_timeout (void *dev, void *data, size_t size,
                  uint32_t timeout_us, sys_read_t dev_read)
{
    uint8_t *buffer = data;
    size_t left;
    size_t count;
    uint32_t timeout = timeout_us;

    count = 0;
    left = size;
    while (left)
    {
        int ret;

        errno = 0;
        ret = dev_read (dev, buffer, left);
        if (ret < 0)
        {
            if (errno != EAGAIN)
                return ret;
            if (timeout_us == 0)
                return ret;
            timeout_us--;
            DELAY_US (1);
            continue;
        }

        count += ret;
        left -= ret;
        buffer += ret;
        timeout = timeout_us;
    }
    return count;
}


/* Helper write function for device drivers.  The timeout is reset for
   every write without fail.  */
ssize_t
sys_write_timeout (void *dev, const void *data, size_t size,
                   uint32_t timeout_us, sys_write_t dev_write)
{
    const uint8_t *buffer = data;
    size_t left;
    size_t count;
    uint32_t timeout = timeout_us;

    count = 0;
    left = size;
    while (left)
    {
        int ret;

        errno = 0;
        ret = dev_write (dev, buffer, left);
        if (ret < 0)
        {
            if (errno != EAGAIN)
                return ret;
            if (timeout_us == 0)
                return ret;
            timeout_us--;
            DELAY_US (1);
            continue;
        }

        count += ret;
        left -= ret;
        buffer += ret;
        timeout = timeout_us;
    }
    return count;
}

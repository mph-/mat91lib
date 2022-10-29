#ifndef SYS_H
#define SYS_H

#ifdef __cplusplus
extern "C" {
#endif


#include "config.h"
#include "errno.h"
#include <unistd.h>

typedef ssize_t (*sys_write_t) (void *file, const void *buffer, size_t size);

typedef ssize_t (*sys_read_t) (void *file, void *buffer, size_t size);

typedef void *(*sys_open_t) (void *fs, const char *pathname, int flags);

typedef int (*sys_close_t) (void *file);

typedef off_t (*sys_lseek_t) (void *file, off_t offset, int whence);

typedef int (*sys_unlink_t) (void *fs, const char *pathname);

typedef int (*sys_rename_t) (void *fs, const char *oldpathname,
                             const char *newpathname);

/* Device operations.  */
typedef struct sys_file_ops_struct
{
    sys_write_t write;
    sys_read_t read;
    sys_open_t open;
    sys_close_t close;
    sys_lseek_t lseek;
} sys_file_ops_t;


typedef struct sys_fs_ops_struct
{
    /* Function to unlink (delete) a file.  */
    sys_unlink_t unlink;
    /* Function to rename a file.  */
    sys_rename_t rename;
    /* readdir ?  */
} sys_fs_ops_t;


/* Filesystem operations.  */
typedef struct sys_fs_struct
{
    /* Device operations.  */
    const sys_file_ops_t *file_ops;
    /* File system operations.  */
    const sys_fs_ops_t *fs_ops;
    /* The flags could be used for read-only.  */
    int flags;
    /* This is a handle for a file system implementation.  */
    void *handle;
} sys_fs_t;



void sys_redirect (unsigned int fd, sys_read_t read, sys_write_t write,
                   void *arg);

void sys_redirect_stdin (sys_read_t read, void *arg);

void sys_redirect_stdout (sys_write_t write, void *arg);

void sys_redirect_stderr (sys_write_t write, void *arg);

bool sys_mount (sys_fs_t *fs, int flags);

int sys_attach (sys_file_ops_t *file_ops, void *arg);


/** Register a device with a devicename, say /dev/usart0,
    and record the file_ops and arg (say device handle).
    The device can then be opened using open.  */
int
sys_device_register (const char *devicename, const sys_file_ops_t *file_ops,
                     void *arg);


/** Helper blocking read function for device drivers.  */
ssize_t
sys_read_timeout (void *dev, void *data, size_t size,
                  uint32_t timeout_us, sys_read_t dev_read);


/** Helper blocking write function for device drivers.  */
ssize_t
sys_write_timeout (void *dev, const void *data, size_t size,
                   uint32_t timeout_us, sys_write_t dev_write);


#ifdef __cplusplus
}
#endif
#endif

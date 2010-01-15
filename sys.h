#ifndef SYS_H
#define SYS_H

#include <unistd.h>

typedef ssize_t (*sys_write_t) (void *file, const char *buffer, size_t size);

typedef ssize_t (*sys_read_t) (void *file, char *buffer, size_t size);

typedef void *(*sys_open_t) (void *fs, const char *pathname, int flags);

typedef int (*sys_close_t) (void *file);

typedef off_t (*sys_lseek_t) (void *file, off_t offset, int whence);

typedef int (*sys_unlink_t) (void *fs, const char *pathname);

/* Device operations.  */
typedef struct sys_dev_struct
{
    sys_write_t write;
    sys_read_t read;
    sys_open_t open;
    sys_close_t close;
    sys_lseek_t lseek;
} sys_dev_t;


/* Filesystem operations.  */
typedef struct sys_fs_struct
{
    sys_dev_t *dev;
    const char *name;
    sys_unlink_t unlink;
    /* readdir ?  */
} sys_fs_t;


void sys_redirect_stdin (sys_read_t read1, void *file);

void sys_redirect_stdout (sys_write_t write, void *file);

void sys_redirect_stderr (sys_write_t write, void *file);

void sys_fs_register (sys_fs_t *fs, void *arg);

#endif

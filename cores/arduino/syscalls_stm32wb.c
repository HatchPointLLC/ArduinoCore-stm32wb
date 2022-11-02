/*
 * Copyright (c) 2016-2022 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

#include "armv7m.h"
#include "rtos_api.h"

extern int wiring_stdin_read(char *data, int nbytes);
extern int wiring_stdout_write(const char *data, int nbytes);
extern int wiring_stderr_write(const char *data, int nbytes);

#undef errno
extern int errno;

void * _sbrk (int nbytes)
{
    void *p = k_heap_allocate(nbytes);

    if (!p)
    {
        errno = ENOMEM;

        return  (void *) -1;
    }

    return p;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;

    errno = EINVAL;

    return -1;
}

int _close(int file) {
    (void)file;

    return -1;
}

int _isatty(int file) 
{
    (void)file;

    switch (file) {
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
        return 1;

    default:
        errno = EBADF;
        return 0;
    }
}

int _fstat(int file, struct stat *st)
{
    (void)file;

    st->st_mode = S_IFCHR;

    return 0;
}

int _lseek(int file, int offset, int whence)
{
    (void)file;
    (void)offset;
    (void)whence;

    return 0;
}

int _read(int file, char *buf, int nbytes)
{
    switch (file) {
    case STDIN_FILENO:
        return wiring_stdin_read(buf, nbytes);

    default:
        errno = EBADF;
        return -1;
    }
}

int _write(int file, const char *buf, int nbytes)
{
    switch (file) {
    case STDOUT_FILENO:
        return wiring_stdout_write(buf, nbytes);

    case STDERR_FILENO:
        return wiring_stderr_write(buf, nbytes);

    default:
        errno = EBADF;
        return -1;
    }
}

void _exit(int status) 
{
    (void)status;

    while (1) { };
}

static k_mutex_t __malloc_mutex = K_MUTEX_INIT(K_PRIORITY_MIN, K_MUTEX_PRIORITY_INHERIT | K_MUTEX_RECURSIVE);

void __malloc_lock(struct _reent *ptr __attribute__((unused)))
{
    k_mutex_lock(&__malloc_mutex, K_TIMEOUT_FOREVER);
}

void __malloc_unlock(struct _reent *ptr __attribute__((unused)))
{
    k_mutex_unlock(&__malloc_mutex);
}


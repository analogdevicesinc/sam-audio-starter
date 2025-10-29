/**
 * Copyright (c) 2025 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

/* CCES includes. */
#include <stdlib.h>
#include <sys/reent.h>
#include <libio/device.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/***********************************************************************
 * Device I/O (DevIO) workarounds
 **********************************************************************/
extern DevEntry_t DevDrvTable[MAXDEV];
extern sDevTab DeviceIOtable[MAXFD];
extern int __devtabs_initialized;

extern void __init_devtabs(void);

static SemaphoreHandle_t _DEVTABS_LOCK = NULL;

static __inline void
_INIT_DEVTABS(void)
{
    if (!__devtabs_initialized)
        __init_devtabs();
}

static __inline void
_LOCK_DEVTABS(void)
{
    if (_DEVTABS_LOCK == NULL) {
        _DEVTABS_LOCK = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(_DEVTABS_LOCK, portMAX_DELAY);
}

static __inline void
_UNLOCK_DEVTABS(void)
{
    xSemaphoreGive(_DEVTABS_LOCK);
}

#define _LOCK_DEVDRV(drv)
#define _UNLOCK_DEVDRV(drv)
#define DEVFD(fd)   DeviceIOtable[fd].DevFileDes
#define DEVDRV(fd)  DeviceIOtable[fd].DeviceID

/*
 * The ADI _dev_open() function in the ARM standard library is not thread
 * safe.  It does not lock the global Device I/O device driver table
 * (DevDrvTable[]) when searching for a free file descriptor.
 *
 * This function can replace the ADI library function _dev_open() to
 * protect the search for an empty file descriptor by implenting
 * _LOCK_DEVTABS() / _UNLOCK_DEVTABS().
 *
 * The underlying driver's open() function is thread-safe in the
 * fs_devio module so no need to implent _LOCK_DEVDRV() / _UNLOCK_DEVDRV().
 *
 * Use the -Wl,--wrap=_dev_open linker command to enable this function.
 *
 */
int
__wrap__dev_open(int devid, const char *name, int mode)
{
     _INIT_DEVTABS();

    // Look up the device driver by ID.
    DevEntry *dep;
    int drv = 0;
    for (;;) {
        if (!(dep = DevDrvTable[drv]))
            return -1;  // Reached end marker.
        if (devid == dep->DeviceID)
            break;  // Found the driver.
        if (++drv == MAXDEV)  // Reached end of the table.
            return -1;
    }

    // Find a free file descriptor.
    _LOCK_DEVTABS();
    int fd = 0;
    while (DEVDRV(fd) >= 0) {
        if (++fd == MAXFD) {
            // None available.
            _UNLOCK_DEVTABS();
            return -1;
        }
    }
    DEVDRV(fd) = drv;   // Found one: grab it.
    _UNLOCK_DEVTABS();

    // Invoke the driver's open function.
    _LOCK_DEVDRV(drv);
    int devfd = dep->open(name, mode);
    _UNLOCK_DEVDRV(drv);

    if (devfd == -1) {
        // Driver failed to open the file: release the FD and return failure.
        DEVDRV(fd) = -1;
        return -1;
    }

    DEVFD(fd) = devfd;
    return fd;
}

/*
 * The ADI _dev_close() function in the ARM standard library does not
 * track thread specific standard I/O streams so any thread with initialized
 * stdin, stdout, and stderr streams (like telnet) will call this function
 * during _reclaim_reent() cleanup.
 *
 * This function overrides _dev_close() to never close the standard I/O
 * streams.
 *
 * Use the -Wl,--wrap=_dev_close linker command to enable this function.
 *
 */

#define FDOK(fd)    ((fd) >= 0 && (fd) < MAXFD)

int
__wrap__dev_close(int fd)
{
    if (!FDOK(fd))
        return -1;

    if (fd < 3) {
        return 0;
    }

    _INIT_DEVTABS();
    int drv = DEVDRV(fd);
    if (drv < 0)
        return -1;

    int devfd = DEVFD(fd);
    DEVDRV(fd) = -1;

    _LOCK_DEVDRV(drv);
    int ret = DevDrvTable[drv]->close(devfd);
    _UNLOCK_DEVDRV(drv);
    return ret;
}

/***********************************************************************
 * standard stream workarounds
 **********************************************************************/
/*
 * This function ensures a task has an independent set of standard I/O
 * streams (stdin, stdout, and stderr).
 *
 * This code works around an ADI specific modification in __sinit() that
 * assigns a new thread's standard I/O streams to the _GLOBAL_REENT
 * standard I/O streams whenever _REENT != _GLOBAL_REENT.
 *
 * In the ADI thread model, this condition is always true because threads
 * are allocated a thread specific _reent structure in adi_rtl_get_tls_ptr()
 * and the _REENT macro resolves to adi_rtl_get_tls_ptr().
 *
 * A wide variety of bad things happen when threads that perform
 * standard I/O on those streams share the _GLOBAL_REENT streams.
 * For instance, a new thread's standard streams are unconditionally
 * initialized in __sinit() when the thread performs its first file I/O
 * operation.  This results in memory leaks and data loss.
 * Additionally, sharing a stream means the stream's I/O buffer is
 * shared so the output of any thread emitting to stdout will be
 * mixed with the output from other threads.  Similarly, buffered input
 * from stdin could be delivered to the wrong thread.
 *
 * This function should be called by any thread using standard I/O streams
 * immediately after thread startup.
 *
 */
void THREAD_INIT_STDIO(void)
{
    struct _reent **GR_PTR = (struct _reent ** )&_GLOBAL_REENT;
    struct _reent *GR = (struct _reent * )_GLOBAL_REENT;
    taskENTER_CRITICAL();
    *GR_PTR = _REENT;
    _REENT_SMALL_CHECK_INIT(_REENT);
    *GR_PTR = GR;
    taskEXIT_CRITICAL();
}

/*
 * This function frees thread specific resources.  This function should
 * be called by all terminating threads immediately before calling
 * vTaskDelete().
 *
 */
void THREAD_FREE_STDIO(void)
{
    _reclaim_reent(_REENT);
    free(adi_rtl_get_tls_ptr());
}

/***********************************************************************
 * CRT malloc / calloc / realloc / free tracking (not umm_malloc)
 **********************************************************************/
/*
 * These functions can be used to track CRT heap memory allocations
 *
 * Use the -Wl,--wrap=malloc linker command to enable this function.
 * Use the -Wl,--wrap=calloc linker command to enable this function.
 * Use the -Wl,--wrap=free linker command to enable this function.
 *
 */
#include <stdlib.h>

void * __real__malloc_r(struct _reent *r, size_t size);
void * __real__calloc_r(struct _reent *r, size_t nmemb, size_t size);
void * __real__realloc_r(struct _reent *r, void *ptr, size_t size);
void __real__free_r(struct _reent *r, void *ptr);

void *__wrap__malloc_r(struct _reent *r, size_t size)
{
    void *ptr;
    ptr = __real__malloc_r(r, size);
    return(ptr);
}

void *__wrap__calloc_r(struct _reent *r, size_t nmemb, size_t size)
{
    void *ptr;
    ptr = __real__calloc_r(r, nmemb, size);
    return(ptr);
}

void *__wrap__realloc_r(struct _reent *r, void *ptr, size_t size)
{
    ptr = __real__realloc_r(r, ptr, size);
    return(ptr);
}

void __wrap__free_r(struct _reent *r, void *ptr)
{
    __real__free_r(r, ptr);
}

/***********************************************************************
 * Cortex A55 hacks
 **********************************************************************/
#if defined(__ADSPCORTEXA55__)
/*
 * When the debugger is not attached, _fstat() can be called which calls
 * _swistat() that calls do_AngelSVC(). This may cause FreeRTOS to crash.
 *
 * Implement _fstat() here and always return an error.
 */
#include <sys/stat.h>
int
_fstat (int fd, struct stat * st)
{
    return -1;
}
#endif

/**
 * Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#include <stdio.h>
#include <stdint.h>

#if defined(__ADSPARM__)
#include <libio/device.h>
#include <libio/device_int.h>
#else
#include <device.h>
#endif

/* Kernel includes. */
#ifdef FREE_RTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

#include "uart_stdio.h"
#include "uart_stdio_io.h"

/* Select proper driver API for stdio operations */
#ifdef USB_CDC_STDIO
#define uart_write uart_cdc_write
#define uart_read uart_cdc_read
#define uart_setTimeouts uart_cdc_setTimeouts
#else
#define uart_write uart_write
#define uart_read uart_read
#define uart_setTimeouts uart_setTimeouts
#endif

#define UART_STDIO_DEV         100
#define UART_STDIO_STDIN_FD      0
#define UART_STDIO_STDOUT_FD     1
#define UART_STDIO_STDERR_FD     2

static sUART *stdioUartHandle;
static int stdioUartMode;

static int uart_stdio_dev_init(struct DevEntry *deventry)
{
    return(_DEV_IS_THREADSAFE);
}

static int uart_stdio_dev_open(const char *path, int flags)
{
    return(0);
}

static int uart_stdio_dev_close(int fh)
{
    return(0);
}

static int uart_stdio_dev_write(int fh, unsigned char *ptr, int len)
{
    UART_SIMPLE_RESULT uartResult;
    unsigned char buf[32];
    uint16_t b;
    int i;
    UART_STDIO_IO *io = NULL;

#ifdef FREE_RTOS
    io = pvTaskGetThreadLocalStoragePointer(NULL, configSTDIO_APP_TLS_POINTER);
#endif
    if (io && io->out) {
        io->out(ptr, len, io->usr);
    } else {
        if (!stdioUartHandle) {
            return(-1);
        }
        b = 0;
        for (i = 0; i < len; i++) {
            if ((ptr[i] == '\n') && (stdioUartMode == UART_STDIO_MODE_COOKED)) {
                buf[b++] = '\r';
            }
            buf[b++] = *(ptr+i);
            if (b >= (sizeof(buf)-1)) {
                uartResult = uart_write(stdioUartHandle, buf, &b);
                b = 0;
            }
        }
        if (b) {
            uartResult = uart_write(stdioUartHandle, buf, &b);
        }
    }

#if defined(__ADSPARM__)
    len = 0;
#endif

    return(len);
}

static int uart_stdio_dev_read(int fh, unsigned char *buffer, int len)
{
    uint16_t readLen = len;
    UART_SIMPLE_RESULT uartResult;
    UART_STDIO_IO *io = NULL;

#ifdef FREE_RTOS
    io = pvTaskGetThreadLocalStoragePointer(NULL, configSTDIO_APP_TLS_POINTER);
#endif
    if (io && io->in) {
        readLen = io->in(buffer, len, io->usr);
    } else {
        if (!stdioUartHandle) {
            return(-1);
        }
        readLen = (len > 255) ? 255 : len;
        uartResult = uart_read(stdioUartHandle, buffer, &readLen);
    }

    if (readLen == 0) {
        len = -1;
    } else {
#if defined(__ADSPARM__)
        len = len - readLen;
#else
        len = readLen;
#endif
    }

    return(len);
}

static long uart_stdio_seek(int fh, long pos, int dir)
{
    return(-1);
}

static int uart_stdio_unlink(const char *path)
{
    return(-1);
}

static int uart_stdio_rename(const char *oldpath, const char *newpath)
{
    return(-1);
}

/***********************************************************************
 * ARM specific devio functions and structs
 **********************************************************************/
#if defined(__ADSPARM__)

static int uart_stdio_isatty(int fh)
{
    return(0);
}

static int uart_stdio_system(const char *cmd)
{
    return(0);
}

static clock_t uart_stdio_times(void)
{
    return(0);
}

static void uart_stdio_gettimeofday(struct timeval *tp, void *tzvp)
{
}

static int uart_stdio_kill(int processID, int signal)
{
    return(0);
}

static int uart_stdio_get_errno(void)
{
    return(0);
}

DevEntry uart_stdio_deventry = {
    UART_STDIO_DEV,            /* int DeviceID */
    NULL,                      /* void *data */
    uart_stdio_dev_init,       /* int device _init(struct DevEntry *d) */
    uart_stdio_dev_open,       /* int device _open(const char *path, int flags) */
    uart_stdio_dev_close,      /* int device _close(int fh) */
    uart_stdio_dev_write,      /* int device _write(int fh, char *ptr, int len) */
    uart_stdio_dev_read,       /* int device _read(int fh, char *ptr, int len) */
    uart_stdio_seek,           /* int device _seek(int fh, int pos, int dir) */
    UART_STDIO_STDIN_FD,       /* int stdinfd */
    UART_STDIO_STDOUT_FD,      /* int stdoutfd */
    UART_STDIO_STDERR_FD,      /* int stderrfd */
    uart_stdio_unlink,         /* int device _unlink(const char *path) */
    uart_stdio_rename,         /* int device _rename(const char *oldpath, const char *newpath) */
    uart_stdio_system,         /* int device _system(const char *cmd) */
    uart_stdio_isatty,         /* int device _isatty(int fh) */
    uart_stdio_times,          /* clock_t device _times(void) */
    uart_stdio_gettimeofday,   /* void device _gettimeofday(struct timeval *tp, void *tzvp) */
    uart_stdio_kill,           /* int device _kill(int processID, int signal) */
    uart_stdio_get_errno       /* int device _get_errno(void) */
};

#else

/***********************************************************************
 * SHARC+ specific devio functions and structs
 **********************************************************************/
static int uart_stdio_ioctl(int fildes, int request, va_list varg_list)
{
    int result = -1;

    if (request == UART_STDIO_MODE_SET) {
        if (varg_list != NULL) {
            stdioUartMode = *(int *) varg_list;
            result = 0;
        }
    }

    return(result);
}

struct DevEntry_Extension uart_stdio_extension = {
    DEVFLAGS_BYTEADDRESSED,
    NULL,
    NULL
};

DevEntry uart_stdio_deventry = {
              UART_STDIO_DEV,
              0,
              uart_stdio_dev_init,
              uart_stdio_dev_open,
              uart_stdio_dev_close,
              uart_stdio_dev_write,
              uart_stdio_dev_read,
              uart_stdio_seek,
              UART_STDIO_STDIN_FD,
              UART_STDIO_STDOUT_FD,
              UART_STDIO_STDERR_FD,
              uart_stdio_unlink,
              uart_stdio_rename,
              uart_stdio_ioctl,
              &uart_stdio_extension
};

#endif

/*
 * Defining the first entry of the DevDrvTable[] disconnects stdio
 * from the debugger's RDI interface.
 */
DevEntry_t DevDrvTable[MAXDEV] = {
  &uart_stdio_deventry,
  0,
};

/***********************************************************************
 * Public functions
 **********************************************************************/
int uart_stdio_read(unsigned char *ptr, int len)
{
    len = uart_stdio_dev_read(UART_STDIO_STDIN_FD, ptr, len);
    return len;
}

int uart_stdio_write(unsigned char *ptr, int len)
{
    len = uart_stdio_dev_write(UART_STDIO_STDOUT_FD, ptr, len);
    return len;
}

void uart_stdio_init(sUART *uart)
{
    stdioUartHandle = uart;
    stdioUartMode = UART_STDIO_MODE_COOKED;
#if defined(__ADSPARM__)
    set_default_io_device(UART_STDIO_DEV);
#endif
}

void uart_stdio_set_read_timeout(int timeout)
{
    UART_STDIO_IO *io = NULL;

#ifdef FREE_RTOS
    io = pvTaskGetThreadLocalStoragePointer(NULL, configSTDIO_APP_TLS_POINTER);
#endif
    if (io && io->set_read_timeout) {
        io->set_read_timeout(timeout, io->usr);
    } else {
        if (!stdioUartHandle) {
            return;
        }
        if (timeout == STDIO_TIMEOUT_NONE) {
            timeout = UART_SIMPLE_TIMEOUT_NONE;
        } else if (timeout == STDIO_TIMEOUT_INF) {
            timeout = UART_SIMPLE_TIMEOUT_INF;
        }
        uart_setTimeouts(stdioUartHandle,
            timeout, UART_SIMPLE_TIMEOUT_NO_CHANGE);
    }
}

int uart_stdio_set_mode(int mode)
{
    UART_STDIO_IO *io = NULL;
    int oldMode;

#ifdef FREE_RTOS
    io = pvTaskGetThreadLocalStoragePointer(NULL, configSTDIO_APP_TLS_POINTER);
#endif
    if (io && io->set_read_timeout) {
        oldMode = io->set_mode(mode, io->usr);
    } else {
        oldMode = stdioUartMode;
        stdioUartMode = mode;
    }

    return(oldMode);
}


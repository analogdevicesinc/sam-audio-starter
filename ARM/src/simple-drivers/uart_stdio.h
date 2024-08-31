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

#ifndef _uart_stdio_h
#define _uart_stdio_h

/* Simple driver includes */
#ifdef USB_CDC_STDIO
#include "uart_simple_cdc.h"
#else
#include "uart_simple.h"
#endif

#define STDIO_TIMEOUT_NONE  (UART_SIMPLE_TIMEOUT_NONE)
#define STDIO_TIMEOUT_INF   (UART_SIMPLE_TIMEOUT_INF)

/* SHARC+ ioctl cmd values */
#define UART_STDIO_MODE_SET     0

#define UART_STDIO_MODE_COOKED ((int)0)
#define UART_STDIO_MODE_RAW    ((int)1)

void uart_stdio_init(sUART *uart);
void uart_stdio_set_read_timeout(int timeout);
void uart_stdio_set_mode(int mode);

#endif

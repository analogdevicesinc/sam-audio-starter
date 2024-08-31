/**
 * Copyright (c) 2021 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#ifndef _fs_devio_h
#define _fs_devio_h

#include <time.h>

typedef int (*FS_DEVIO_GETTIMEOFDAY)(struct timeval *tp, void *tzvp);

typedef struct FS_DEVIO_INIT {
    FS_DEVIO_GETTIMEOFDAY getTimeOfDay;
} FS_DEVIO_INIT;

void fs_devio_init(FS_DEVIO_INIT *devioInit);

#endif

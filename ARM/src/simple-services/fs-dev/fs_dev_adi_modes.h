/**
 * Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#ifndef _fs_dev_adi_modes_h
#define _fs_dev_adi_modes_h

#if defined(__ADSPARM__)
#include <fcntl.h>
#define ADI_READ     (O_RDONLY + 1)
#define ADI_WRITE    (O_WRONLY + 1)
#define ADI_RW       (O_RDWR + 1)
#define ADI_APPEND   O_APPEND
#define ADI_CREAT    O_CREAT
#define ADI_TRUNC    O_TRUNC
#else
#define ADI_READ     0x0001
#define ADI_WRITE    0x0002
#define ADI_APPEND   0x0004
#define ADI_TRUNC    0x0008
#define ADI_CREAT    0x0010
#define ADI_RW       (ADI_READ | ADI_WRITE)
#define ADI_BINARY   0x0020
#endif

#endif

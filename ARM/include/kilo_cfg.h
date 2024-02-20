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
#ifndef _kilo_cfg_h
#define _kilo_cfg_h

ssize_t shell_edit_write(void *usr, const void *buf, size_t len);
ssize_t shell_edit_read(void *usr, const void *buf, size_t len);
ssize_t getline(char **buf, size_t *bufsiz, FILE *fp);

#include "umm_malloc.h"
#define KILO_MALLOC  umm_malloc
#define KILO_FREE    umm_free
#define KILO_REALLOC umm_realloc

#include "util.h"
#define KILO_TIME       util_time

/* These must be defined */
#define KILO_TERM_WRITE shell_edit_write
#define KILO_TERM_READ  shell_edit_read

#endif

/**
 * Copyright (c) 2022 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

/*!
 * @brief  Example configuration file enabling the umm_malloc heap
 *         allocation module.
 *
 * @file      xmodem_cfg.h
 * @version   1.0.0
 *
*/

#ifndef _XMODEM_CFG_H
#define _XMODEM_CFG_H

#include "umm_malloc.h"

#define XMODEM_MALLOC  umm_malloc
#define XMODEM_FREE    umm_free

#endif

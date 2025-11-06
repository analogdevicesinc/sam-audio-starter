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

#ifndef _tdm_init_h
#define _tdm_init_h

#include "context.h"

void tdm_init(APP_CONTEXT *context);
void tdm_enable(APP_CONTEXT *context);
void tdm_disable(APP_CONTEXT *context);

#endif
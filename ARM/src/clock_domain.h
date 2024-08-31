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
#ifndef _clock_domain_h
#define _clock_domain_h

#include <stdbool.h>

#include "context.h"
#include "clock_domain_defs.h"

void clock_domain_init(APP_CONTEXT *context);
void clock_domain_set(APP_CONTEXT *context, CLOCK_DOMAIN domain, unsigned mask);
CLOCK_DOMAIN clock_domain_get(APP_CONTEXT *context, unsigned mask);
void clock_domain_set_active(APP_CONTEXT *context, CLOCK_DOMAIN domain, unsigned mask);
void clock_domain_clr_active(APP_CONTEXT *context, CLOCK_DOMAIN domain, unsigned mask);
bool clock_domain_get_active(APP_CONTEXT *context, CLOCK_DOMAIN domain, unsigned mask);
bool clock_domain_ready(APP_CONTEXT *context, CLOCK_DOMAIN domain);
char *clock_domain_str(CLOCK_DOMAIN domain);

#endif

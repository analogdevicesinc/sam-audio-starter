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
#include "context.h"
#include "clock_domain_defs.h"
#include "clock_domain.h"

char *clock_domain_str(CLOCK_DOMAIN domain)
{
    char *str = "CLOCK_DOMAIN_UNKNOWN";

    switch (domain) {
        case CLOCK_DOMAIN_A2B:
            str = "CLOCK_DOMAIN_A2B";
            break;
        case CLOCK_DOMAIN_SYSTEM:
            str = "CLOCK_DOMAIN_SYSTEM";
            break;
        default:
            break;
    }

    return(str);
}

void clock_domain_set(APP_CONTEXT *context, CLOCK_DOMAIN domain, unsigned mask)
{
    int i;
    for (i = 0; i < CLOCK_DOMAIN_MAX; i++) {
        if (i == domain) {
            context->clockDomainMask[i] |= mask;
        } else {
            context->clockDomainMask[i] &= ~mask;
            context->clockDomainActive[i] &= ~mask;
        }
    }
}

CLOCK_DOMAIN clock_domain_get(APP_CONTEXT *context, unsigned mask)
{
    int i;
    for (i = 0; i < CLOCK_DOMAIN_MAX; i++) {
        if (context->clockDomainMask[i] & mask) {
            break;
        }
    }
    return(i);
}

void clock_domain_set_active(APP_CONTEXT *context, CLOCK_DOMAIN domain, unsigned mask)
{
    context->clockDomainActive[domain] |= mask;
}

bool clock_domain_ready(APP_CONTEXT *context, CLOCK_DOMAIN domain)
{
    bool ready =
        (context->clockDomainMask[domain] &
         context->clockDomainActive[domain]) == context->clockDomainMask[domain];
    if (ready) {
        context->clockDomainActive[domain] = 0x00000000;
    }
    return(ready);
}

void clock_domain_init(APP_CONTEXT *context)
{
    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_CODEC_IN);
    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_CODEC_OUT);
    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_SPDIF_IN);
    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_SPDIF_OUT);
    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_USB_RX);
    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_USB_TX);
    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_A2B_IN);
    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_A2B_OUT);
    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_WAV_SRC);
    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_WAV_SINK);
}

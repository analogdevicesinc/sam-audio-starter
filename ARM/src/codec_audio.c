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

#include <stdint.h>
#include <stdbool.h>

#include "context.h"
#include "codec_audio.h"
#include "cpu_load.h"
#include "clock_domain.h"
#include "sharc_audio.h"

#include "sae.h"

void codecAudioOut(void *buffer, uint32_t maxSize, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;
    SAE_MSG_BUFFER *msg = NULL;
    uint32_t inCycles, outCycles;

    /* Track ISR cycle count for CPU load */
    inCycles = cpuLoadGetTimeStamp();

    /* Get the IPC message associated with the data pointer */
    if (buffer == context->codecAudioOut[0]) {
        msg = context->codecMsgOut[0];
    } else if (buffer == context->codecAudioOut[1]) {
        msg = context->codecMsgOut[1];
    }

    /* This is the CLOCK_DOMAIN_SYSTEM "out" clock source */
    sharcAudio(context, CLOCK_DOMAIN_BITM_CODEC_OUT, msg, true, false);

    /* Track ISR cycle count for CPU load */
    outCycles = cpuLoadGetTimeStamp();
    cpuLoadIsrCycles(outCycles - inCycles);
}

void codecAudioIn(void *buffer, uint32_t size, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;
    SAE_MSG_BUFFER *msg = NULL;
    uint32_t inCycles, outCycles;

    /* Track ISR cycle count for CPU load */
    inCycles = cpuLoadGetTimeStamp();

    /* Get the IPC message associated with the data pointer */
    if (buffer == context->codecAudioIn[0]) {
        msg = context->codecMsgIn[0];
    } else if (buffer == context->codecAudioIn[1]) {
        msg = context->codecMsgIn[1];
    }

    /* This is the CLOCK_DOMAIN_SYSTEM "in" clock source */
    sharcAudio(context, CLOCK_DOMAIN_BITM_CODEC_IN, msg, true, true);

    /* Track ISR cycle count for CPU load */
    outCycles = cpuLoadGetTimeStamp();
    cpuLoadIsrCycles(outCycles - inCycles);
}

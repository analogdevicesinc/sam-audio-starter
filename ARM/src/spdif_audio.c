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
#include "spdif_audio.h"
#include "cpu_load.h"
#include "clock_domain.h"
#include "sharc_audio.h"

#include "sae.h"

void spdifAudioOut(void *buffer, uint32_t maxSize, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;
    SAE_MSG_BUFFER *msg = NULL;
    uint32_t inCycles, outCycles;

    /* Track ISR cycle count for CPU load */
    inCycles = cpuLoadGetTimeStamp();

    /* Get the IPC message associated with the data pointer */
    if (buffer == context->spdifAudioOut[0]) {
        msg = context->spdifMsgOut[0];
    } else if (buffer == context->spdifAudioOut[1]) {
        msg = context->spdifMsgOut[1];
    }

    /* Indicate SPDIF "out" audio ready */
    sharcAudio(context, CLOCK_DOMAIN_BITM_SPDIF_OUT, msg, false, false);

    /* Track ISR cycle count for CPU load */
    outCycles = cpuLoadGetTimeStamp();
    cpuLoadIsrCycles(outCycles - inCycles);
}

void spdifAudioIn(void *buffer, uint32_t size, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;
    SAE_MSG_BUFFER *msg = NULL;
    uint32_t inCycles, outCycles;

    /* Track ISR cycle count for CPU load */
    inCycles = cpuLoadGetTimeStamp();

    /* Get the IPC message associated with the data pointer */
    if (buffer == context->spdifAudioIn[0]) {
        msg = context->spdifMsgIn[0];
    } else if (buffer == context->spdifAudioIn[1]) {
        msg = context->spdifMsgIn[1];
    }

    /* Indicate SPDIF "in" audio ready */
    sharcAudio(context, CLOCK_DOMAIN_BITM_SPDIF_IN, msg, false, true);

    /* Track ISR cycle count for CPU load */
    outCycles = cpuLoadGetTimeStamp();
    cpuLoadIsrCycles(outCycles - inCycles);
}

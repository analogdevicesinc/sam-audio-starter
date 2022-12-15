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
#include "a2b_audio.h"
#include "cpu_load.h"
#include "sharc_audio.h"

#include "sae.h"

void a2bAudioOut(void *buffer, uint32_t size, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;
    SAE_MSG_BUFFER *msg = NULL;
    uint32_t inCycles, outCycles;
    bool clockSource;

    /* Track ISR cycle count for CPU load */
    inCycles = cpuLoadGetTimeStamp();

    /* Get the IPC message associated with the data pointer */
    if (buffer == context->a2bAudioOut[0]) {
        msg = context->a2bMsgOut[0];
    } else if (buffer == context->a2bAudioOut[1]) {
        msg = context->a2bMsgOut[1];
    }

    /* This is the CLOCK_DOMAIN_A2B "out" clock source in slave mode */
    clockSource = (context->a2bmode == A2B_BUS_MODE_SLAVE);
    sharcAudio(context, CLOCK_DOMAIN_BITM_A2B_OUT, msg, clockSource, false);

    /* Track ISR cycle count for CPU load */
    outCycles = cpuLoadGetTimeStamp();
    cpuLoadIsrCycles(outCycles - inCycles);
}

void a2bAudioIn(void *buffer, uint32_t size, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;
    SAE_MSG_BUFFER *msg = NULL;
    uint32_t inCycles, outCycles;
    bool clockSource;

    /* Track ISR cycle count for CPU load */
    inCycles = cpuLoadGetTimeStamp();

    /* Get the IPC message associated with the data pointer */
    if (buffer == context->a2bAudioIn[0]) {
        msg = context->a2bMsgIn[0];
    } else if (buffer == context->a2bAudioIn[1]) {
        msg = context->a2bMsgIn[1];
    }

    /* This is the CLOCK_DOMAIN_A2B "in" clock source in slave mode */
    clockSource = (context->a2bmode == A2B_BUS_MODE_SLAVE);
    sharcAudio(context, CLOCK_DOMAIN_BITM_A2B_IN, msg, clockSource, true);

    /* Track ISR cycle count for CPU load */
    outCycles = cpuLoadGetTimeStamp();
    cpuLoadIsrCycles(outCycles - inCycles);
}

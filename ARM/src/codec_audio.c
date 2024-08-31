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

#include <string.h>

#include "context.h"
#include "codec_audio.h"
#include "cpu_load.h"
#include "process_audio.h"
#include "clock_domain.h"
#include "route.h"

void codecAudioOut(void *buffer, uint32_t maxSize, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;

    /* Track ISR CPU load */
    cpuLoadISREnter();

    /* Clear the contents */
    memset(buffer, 0, maxSize);

    /* This is the CLOCK_DOMAIN_SYSTEM "out" clock source */
    processAudio(context, CLOCK_DOMAIN_BITM_CODEC_OUT, STREAM_ID_CODEC_OUT,
        CODEC_DMA_CHANNELS, SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
        buffer, true,
        true, false);

    /* Track ISR CPU load */
    cpuLoadISRExit();
}

void codecAudioIn(void *buffer, uint32_t size, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;

    /* Track ISR CPU load */
    cpuLoadISREnter();

    /* This is the CLOCK_DOMAIN_SYSTEM "in" clock source */
    processAudio(context, CLOCK_DOMAIN_BITM_CODEC_IN, STREAM_ID_CODEC_IN,
        CODEC_DMA_CHANNELS, SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
        buffer, false,
        true, true);

    /* Track ISR CPU load */
    cpuLoadISRExit();
}

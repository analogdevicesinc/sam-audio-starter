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
#include "spdif_audio.h"
#include "cpu_load.h"
#include "process_audio.h"
#include "clock_domain.h"
#include "route.h"

void spdifAudioOut(void *buffer, uint32_t maxSize, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;

    /* Track ISR CPU load */
    cpuLoadISREnter();

    /* Clear the contents */
    memset(buffer, 0, maxSize);

    /* Process audio */
    processAudio(context, CLOCK_DOMAIN_BITM_SPDIF_OUT, STREAM_ID_SPDIF_OUT,
        SPDIF_DMA_CHANNELS, SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
        buffer, true,
        false, false);

    /* Track ISR CPU load */
    cpuLoadISRExit();
}

void spdifAudioIn(void *buffer, uint32_t maxSize, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;

    /* Track ISR CPU load */
    cpuLoadISREnter();

    /* Process audio */
    processAudio(context, CLOCK_DOMAIN_BITM_SPDIF_IN, STREAM_ID_SPDIF_IN,
        SPDIF_DMA_CHANNELS, SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
        buffer, false,
        false, true);

    /* Track ISR CPU load */
    cpuLoadISRExit();
}

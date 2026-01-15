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

#include <string.h>
#include <stdbool.h>
#include <runtime/cache/adi_cache.h>

#include "context.h"
#include "tdm_audio.h"
#include "cpu_load.h"
#include "process_audio.h"
#include "clock_domain.h"
#include "route.h"

/* This is the CLOCK_DOMAIN_TDM "out" clock source */
void tdmAudioOut(void *buffer, uint32_t maxSize, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;

    /* Track ISR CPU load */
    cpuLoadISREnter();

    /* Clear the contents */
    memset(buffer, 0, maxSize);

    /* This is the CLOCK_DOMAIN_TDM "out" clock source */
    processAudio(context, CLOCK_DOMAIN_BITM_TDM_OUT, STREAM_ID_TDM_OUT,
        TDM_OUT_DMA_CHANNELS, SYSTEM_BLOCK_SIZE, sizeof(TDM_AUDIO_TYPE),
        buffer, true,
        true, false);

    /* Track ISR CPU load */
    cpuLoadISRExit();
}

/*
 * Incoming TDM data will be physically presented on the data lines
 * in 16-bit TDM8 format as as follows:
 *
 *   Pin D0: X0, Y0, Z0, 0, 0, 0, 0, 0
 *   Pin D1: X1, Y1, Z1, 0, 0, 0, 0, 0
 *
 * This will map to DMA memory in an interleaved format in blocks of 16
 * channels
 *
 *  X0, X1, Y0, Y1, Z0, Z1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
 *
 * This audio is de-interleaved in tdm_audio.c into this format
 *
 *  X0, Y0, Z0, X1, Y1, Z1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
 *
 * Pin D0 or D1 content can then be routed through the SHARC and back
 * out to D2 in the following format
 *
 *  X, Y, Z, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
 *
 * This is the CLOCK_DOMAIN_TDM "in" clock source
 *
 */
void tdmAudioIn(void *buffer, uint32_t size, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;

    /* Track ISR CPU load */
    cpuLoadISREnter();

    /* De-interleave the TDM audio */
    TDM_AUDIO_TYPE *XYZ_DATA = (TDM_AUDIO_TYPE *)buffer;
    TDM_AUDIO_TYPE XYZ_D1[3];
    for (int i = 0; i < SYSTEM_BLOCK_SIZE; i++) {
        XYZ_D1[0] = XYZ_DATA[1];     // D1_X
        XYZ_D1[1] = XYZ_DATA[3];     // D1_Y
        XYZ_D1[2] = XYZ_DATA[5];     // D1_Z
        XYZ_DATA[0] = XYZ_DATA[0];   // D0_X
        XYZ_DATA[1] = XYZ_DATA[2];   // D0_X
        XYZ_DATA[2] = XYZ_DATA[4];   // D0_X
        XYZ_DATA[3] = XYZ_D1[0];     // D1_X
        XYZ_DATA[4] = XYZ_D1[1];     // D1_Y
        XYZ_DATA[5] = XYZ_D1[2];     // D1_Z
        XYZ_DATA += TDM_IN_DMA_CHANNELS;
    }

    /* Flush modified DMA buffer back to memory */
    flush_data_buffer(buffer,
        (void *)((uintptr_t)buffer + size),
        ADI_FLUSH_DATA_NOINV
    );

    /* This is the CLOCK_DOMAIN_TDM "in" clock source */
    processAudio(context, CLOCK_DOMAIN_BITM_TDM_IN, STREAM_ID_TDM_IN,
        TDM_IN_DMA_CHANNELS, SYSTEM_BLOCK_SIZE, sizeof(TDM_AUDIO_TYPE),
        buffer, false,
        true, true);

    /* Track ISR CPU load */
    cpuLoadISRExit();
}

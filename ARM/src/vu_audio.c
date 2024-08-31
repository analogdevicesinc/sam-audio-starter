/**
 * Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "context.h"
#include "task_cfg.h"
#include "vu_audio.h"
#include "clock_domain.h"
#include "task_cfg.h"

static SemaphoreHandle_t lock = NULL;

__attribute__ ((section(".l3_cached_data")))
static SYSTEM_AUDIO_TYPE vuAudioBuffer[2][VU_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE vuBuffer[VU_MAX_CHANNELS];

__attribute__((optimize("O1")))
static inline int32_t fastAbs32(int32_t val)
{
    int32_t neg;

    neg = -val;
    if (neg < 0) {
        neg = val - (((uint32_t)val) >> 31);
    }

    return neg;
}

/* Subtract 2 ** DECAY_SHIFT every block */
#define DECAY_SHIFT 6

/* This task monitors VU audio */
portTASK_FUNCTION(vuTask, pvParameters)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    SYSTEM_AUDIO_TYPE *audio;
    SYSTEM_AUDIO_TYPE *sample;
    SYSTEM_AUDIO_TYPE peak;
    SYSTEM_AUDIO_TYPE decay;
    uint32_t pingPong;
    unsigned block;
    unsigned idx;

    UNUSED(context);

    while (1) {

        /* Wait for a notification of new audio */
        pingPong = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

        /* Get a pointer to the quiescent buffer */
        audio = vuAudioBuffer[pingPong];

        /* Sample a random block */
        block = rand() % (SYSTEM_BLOCK_SIZE - 1);

        /* Lock the VU buffer */
        xSemaphoreTake(lock, portMAX_DELAY);

        /* Update VU from the block */
        sample = audio + block * VU_MAX_CHANNELS;
        for (idx = 0; idx < VU_MAX_CHANNELS; idx++) {
            decay = vuBuffer[idx] >> DECAY_SHIFT;
            if (vuBuffer[idx]) {
                vuBuffer[idx] -= decay ? decay : 1;
            }
            peak = fastAbs32(sample[idx]);
            if (peak > vuBuffer[idx]) {
                vuBuffer[idx] = peak;
            }
        }

        /* Unlock the VU buffer */
        xSemaphoreGive(lock);

        /* Clear the audio buffer */
        memset(audio, 0, sizeof(vuAudioBuffer[0]));
    }
}

unsigned getVU(APP_CONTEXT *context, SYSTEM_AUDIO_TYPE *vu, unsigned channels)
{
    unsigned idx;
    channels = channels > VU_MAX_CHANNELS ? VU_MAX_CHANNELS : channels;
    if (vu) {
        memset(vu, 0, sizeof(SYSTEM_AUDIO_TYPE) * channels);
        xSemaphoreTake(lock, portMAX_DELAY);
        for (idx = 0; idx < channels; idx++) {
            vu[idx] = vuBuffer[idx];
        }
        xSemaphoreGive(lock);
    }
    return(channels);
}

void vu_audio_init(APP_CONTEXT *context)
{
    lock = xSemaphoreCreateMutex();

    memset(vuBuffer, 0, sizeof(vuBuffer));
    memset(vuAudioBuffer, 0, sizeof(vuAudioBuffer));

    xTaskCreate(vuTask, "VUTask", VU_TASK_STACK_SIZE,
        context, VU_TASK_PRIORITY, &context->vuTaskHandle);
}

int xferVUSinkAudio(APP_CONTEXT *context, void **audio, CLOCK_DOMAIN cd)
{
    CLOCK_DOMAIN myCd;
    BaseType_t wake;

    static unsigned pingPong = 0;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_VU_IN);
    if (myCd != cd) {
        return(0);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_VU_IN);

    xTaskNotifyFromISR(context->vuTaskHandle,
        pingPong, eSetValueWithOverwrite, &wake
    );

    portYIELD_FROM_ISR(wake);

    pingPong = (pingPong + 1) & 1;

    *audio = vuAudioBuffer[pingPong];

    return(1);
}

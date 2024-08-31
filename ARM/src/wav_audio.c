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

#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "context.h"
#include "util.h"
#include "wav_file.h"
#include "wav_audio.h"
#include "umm_malloc.h"
#include "clock_domain.h"
#include "task_cfg.h"

static unsigned wavSrcUnderflow = 0;
static unsigned wavSinkOverflow = 0;

/* Task notification values */
enum {
    WAV_TASK_NO_ACTION,
    WAV_TASK_AUDIO_SRC_MORE_DATA,
    WAV_TASK_AUDIO_SINK_MORE_DATA,
};

static SYSTEM_AUDIO_TYPE srcBuffer[WAV_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE srcBuffer2[WAV_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE sinkBuffer2[WAV_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE sinkBuffer3[WAV_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];

/* This task keeps the wav src ring buffer full */
portTASK_FUNCTION(wavSrcTask, pvParameters)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    WAV_FILE *wavSrc = &context->wavSrc;
    PaUtilRingBuffer *wavSrcRB = context->wavSrcRB;
    uint32_t whatToDo;
    unsigned samplesIn;
    unsigned samplesOut;
    size_t rsize;
    bool ok;

    while (1) {
        xSemaphoreTake((SemaphoreHandle_t)wavSrc->lock, portMAX_DELAY);
        if (wavSrc->enabled) {
            samplesIn = WAV_MAX_CHANNELS * SYSTEM_BLOCK_SIZE;
            samplesOut = PaUtil_GetRingBufferWriteAvailable(wavSrcRB);
            ok = true;
            while (ok && (samplesOut >= samplesIn)) {
                rsize = readWave(wavSrc, srcBuffer, samplesIn);
                ok = (rsize >= 0);
                if (ok) {
                    if (wavSrc->wordSizeBytes == sizeof(SYSTEM_AUDIO_TYPE)) {
                        PaUtil_WriteRingBuffer(wavSrcRB, srcBuffer, rsize);
                    } else {
                        copyAndConvert(
                            srcBuffer, wavSrc->wordSizeBytes, wavSrc->channels,
                            srcBuffer2, sizeof(SYSTEM_AUDIO_TYPE), wavSrc->channels,
                            rsize / wavSrc->channels, true
                        );
                        PaUtil_WriteRingBuffer(wavSrcRB, srcBuffer2, rsize);
                    }
                    samplesOut = PaUtil_GetRingBufferWriteAvailable(wavSrcRB);
                }
            }
            if (!ok) {
                wavSrc->enabled = false;
            }
        } else {
            PaUtil_FlushRingBuffer(wavSrcRB);
        }
        xSemaphoreGive((SemaphoreHandle_t)wavSrc->lock);
        whatToDo = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
}

/* This task keeps the wav sink ring buffer empty */
portTASK_FUNCTION(wavSinkTask, pvParameters)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    WAV_FILE *wavSink = &context->wavSink;
    PaUtilRingBuffer *wavSinkRB = context->wavSinkRB;
    uint32_t whatToDo;
    unsigned samplesIn;
    unsigned samplesOut;
    size_t wsize;
    bool ok;

    while (1) {
        xSemaphoreTake((SemaphoreHandle_t)wavSink->lock, portMAX_DELAY);
        if (wavSink->enabled) {
            samplesIn = PaUtil_GetRingBufferReadAvailable(wavSinkRB);
            samplesOut = wavSink->channels * SYSTEM_BLOCK_SIZE;
            ok = true;
            while (ok && (samplesIn >= samplesOut)) {
                PaUtil_ReadRingBuffer(
                    wavSinkRB, sinkBuffer2, samplesOut
                );
                if (wavSink->wordSizeBytes == sizeof(SYSTEM_AUDIO_TYPE)) {
                    wsize = writeWave(wavSink, sinkBuffer2, samplesOut);
                } else {
                    copyAndConvert(
                        sinkBuffer2, sizeof(SYSTEM_AUDIO_TYPE), wavSink->channels,
                        sinkBuffer3, wavSink->wordSizeBytes, wavSink->channels,
                        samplesOut / wavSink->channels, true
                    );
                    wsize = writeWave(wavSink, sinkBuffer3, samplesOut);
                }
                samplesIn = PaUtil_GetRingBufferReadAvailable(wavSinkRB);
            }
        } else {
            PaUtil_FlushRingBuffer(wavSinkRB);
        }
        xSemaphoreGive((SemaphoreHandle_t)wavSink->lock);
        whatToDo = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
}

void wav_audio_init(APP_CONTEXT *context)
{
    uint32_t dataSize;

    /* Allocate and configure the wave file source ring buffer.
     * The ring buffer unit of measure is in SYSTEM_AUDIO_TYPE sized
     * words
     */
    context->wavSrcRB =
        (PaUtilRingBuffer *)umm_malloc(sizeof(PaUtilRingBuffer));
    assert(context->wavSrcRB);
    dataSize = roundUpPow2(WAV_RING_BUF_SAMPLES);
    context->wavSrcRBData = umm_calloc(dataSize, sizeof(SYSTEM_AUDIO_TYPE));
    assert(context->wavSrcRBData);
    PaUtil_InitializeRingBuffer(context->wavSrcRB,
        sizeof(SYSTEM_AUDIO_TYPE), dataSize, context->wavSrcRBData);

    /* Allocate and configure the wave file sink ring buffer.
     * The ring buffer unit of measure is in SYSTEM_AUDIO_TYPE sized
     * words
     */
    context->wavSinkRB =
        (PaUtilRingBuffer *)umm_malloc(sizeof(PaUtilRingBuffer));
    assert(context->wavSinkRB);
    dataSize = roundUpPow2(WAV_RING_BUF_SAMPLES);
    context->wavSinkRBData = umm_calloc(dataSize, sizeof(SYSTEM_AUDIO_TYPE));
    assert(context->wavSinkRBData);
    PaUtil_InitializeRingBuffer(context->wavSinkRB,
        sizeof(SYSTEM_AUDIO_TYPE), dataSize, context->wavSinkRBData);

    context->wavSrc.lock =  (SemaphoreHandle_t)xSemaphoreCreateMutex();
    context->wavSink.lock =  (SemaphoreHandle_t)xSemaphoreCreateMutex();

    xTaskCreate(wavSrcTask, "WavSrcTask", WAV_TASK_STACK_SIZE,
        context, WAV_TASK_PRIORITY, &context->wavSrcTaskHandle );
    xTaskCreate(wavSinkTask, "WavSinkTask", WAV_TASK_STACK_SIZE,
        context, WAV_TASK_PRIORITY, &context->wavSinkTaskHandle );

}

int xferWavSinkAudio(APP_CONTEXT *context, void *audio, CLOCK_DOMAIN cd,
    unsigned *numChannels)
{
    unsigned samplesIn;
    unsigned samplesOut;
    WAV_FILE *wavSink = &context->wavSink;
    PaUtilRingBuffer *wavSinkRB = context->wavSinkRB;
    CLOCK_DOMAIN myCd;
    BaseType_t wake;

    static bool first = true;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_WAV_SINK);
    if (myCd != cd) {
        return(0);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_WAV_SINK);

    if (!wavSink->enabled || (wavSink->channels == 0)) {
        *numChannels = 0;
        first = true;
        return(1);
    }

    samplesIn = wavSink->channels * SYSTEM_BLOCK_SIZE;
    samplesOut = PaUtil_GetRingBufferWriteAvailable(wavSinkRB);

    if ((samplesIn == 0) || (samplesOut == 0)) {
        *numChannels = 0;
        return(1);
    }

    if (samplesIn <= samplesOut) {
        if (!first) {
            PaUtil_WriteRingBuffer(
                wavSinkRB, audio, samplesIn
            );
        }
    } else {
        wavSinkOverflow++;
    }

    memset(audio, 0, samplesIn * sizeof(SYSTEM_AUDIO_TYPE));
    *numChannels = wavSink->channels;
    first = false;

    if (samplesOut < (WAV_RING_BUF_SAMPLES / 2)) {
        xTaskNotifyFromISR(context->wavSinkTaskHandle,
            WAV_TASK_AUDIO_SINK_MORE_DATA, eSetValueWithoutOverwrite, &wake
        );
        portYIELD_FROM_ISR(wake);
    }

    return(1);
}

int xferWavSrcAudio(APP_CONTEXT *context, void *audio, CLOCK_DOMAIN cd,
    unsigned *numChannels)
{
    unsigned samplesIn;
    unsigned samplesOut;
    WAV_FILE *wavSrc = &context->wavSrc;
    PaUtilRingBuffer *wavSrcRB = context->wavSrcRB;
    CLOCK_DOMAIN myCd;
    BaseType_t wake;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_WAV_SRC);
    if (myCd != cd) {
        return(0);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_WAV_SRC);

    if (!wavSrc->enabled || (wavSrc->channels == 0)) {
        *numChannels = 0;
        return(1);
    }

    samplesIn = PaUtil_GetRingBufferReadAvailable(wavSrcRB);
    samplesOut = wavSrc->channels * SYSTEM_BLOCK_SIZE;

    if ((samplesIn == 0) || (samplesOut == 0)) {
        *numChannels = 0;
        return(1);
    }

    if (samplesIn >= samplesOut) {
        PaUtil_ReadRingBuffer(
            wavSrcRB, audio, samplesOut
        );
        *numChannels = wavSrc->channels;
    } else {
        wavSrcUnderflow++;
        *numChannels = 0;
    }

    if (samplesIn < (WAV_RING_BUF_SAMPLES / 2)) {
        xTaskNotifyFromISR(context->wavSrcTaskHandle,
            WAV_TASK_AUDIO_SRC_MORE_DATA, eSetValueWithoutOverwrite, &wake
        );
        portYIELD_FROM_ISR(wake);
    }

    return(1);
}

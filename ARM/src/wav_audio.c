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

static unsigned wavSrcUnderflow = 0;
static unsigned wavSinkOverflow = 0;

/* Task notification values */
enum {
    WAV_TASK_NO_ACTION,
    WAV_TASK_AUDIO_SRC_MORE_DATA,
    WAV_TASK_AUDIO_SINK_MORE_DATA,
};

static SYSTEM_AUDIO_TYPE srcBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE srcBuffer2[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE sinkBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE sinkBuffer2[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];

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
        xSemaphoreTake(wavSrc->lock, portMAX_DELAY);
        if (wavSrc->enabled) {
            samplesIn = SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE;
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
        xSemaphoreGive(wavSrc->lock);
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
        xSemaphoreTake(wavSink->lock, portMAX_DELAY);
        if (wavSink->enabled) {
            samplesIn = PaUtil_GetRingBufferReadAvailable(wavSinkRB);
            samplesOut = wavSink->channels * SYSTEM_BLOCK_SIZE;
            ok = true;
            while (ok && (samplesIn >= samplesOut)) {
                PaUtil_ReadRingBuffer(
                    wavSinkRB, sinkBuffer2, samplesOut
                );
                wsize = writeWave(wavSink, sinkBuffer2, samplesOut);
                samplesIn = PaUtil_GetRingBufferReadAvailable(wavSinkRB);
            }
        } else {
            PaUtil_FlushRingBuffer(wavSinkRB);
        }
        xSemaphoreGive(wavSink->lock);
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

    context->wavSrc.lock =  xSemaphoreCreateMutex();
    context->wavSink.lock =  xSemaphoreCreateMutex();

    xTaskCreate(wavSrcTask, "WavSrcTask", WAV_TASK_STACK_SIZE,
        context, WAV_TASK_PRIORITY, &context->wavSrcTaskHandle );
    xTaskCreate(wavSinkTask, "WavSinkTask", WAV_TASK_STACK_SIZE,
        context, WAV_TASK_PRIORITY, &context->wavSinkTaskHandle );

}

/* Transfers WAV Sink audio (ISR context) */
SAE_MSG_BUFFER *xferWavSinkAudio(APP_CONTEXT *context, SAE_MSG_BUFFER *msg,
    CLOCK_DOMAIN cd)
{
    unsigned samplesIn;
    unsigned samplesOut;
    IPC_MSG *ipc;
    IPC_MSG_AUDIO *audio;
    WAV_FILE *wavSink = &context->wavSink;
    PaUtilRingBuffer *wavSinkRB = context->wavSinkRB;
    CLOCK_DOMAIN myCd;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_WAV_SINK);
    if (myCd != cd) {
        return(NULL);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_WAV_SINK);

    ipc = sae_getMsgBufferPayload(msg);
    audio = &ipc->audio;
    audio->clockDomain = myCd;

    if (!wavSink->enabled) {
        return(msg);
    }

    samplesIn = audio->numChannels * audio->numFrames;
    samplesOut = PaUtil_GetRingBufferWriteAvailable(wavSinkRB);

    if ((samplesIn == 0) || (samplesOut == 0)) {
        return(msg);
    }

    if (samplesIn <= samplesOut) {
        copyAndConvert(
            audio->data, audio->wordSize, audio->numChannels,
            sinkBuffer, wavSink->wordSizeBytes, wavSink->channels,
            audio->numFrames, true
        );
        PaUtil_WriteRingBuffer(
            wavSinkRB, sinkBuffer, wavSink->channels * audio->numFrames
        );
    } else {
        wavSinkOverflow++;
    }

    if (samplesOut < (WAV_RING_BUF_SAMPLES / 2)) {
        xTaskNotifyFromISR(context->wavSinkTaskHandle,
            WAV_TASK_AUDIO_SINK_MORE_DATA, eSetValueWithoutOverwrite, NULL
        );
    }

    return(msg);
}

/* Transfers WAV Src audio (ISR context) */
SAE_MSG_BUFFER *xferWavSrcAudio(APP_CONTEXT *context, SAE_MSG_BUFFER *msg,
    CLOCK_DOMAIN cd)
{
    unsigned samplesIn;
    unsigned samplesOut;
    IPC_MSG *ipc;
    IPC_MSG_AUDIO *audio;
    WAV_FILE *wavSrc = &context->wavSrc;
    PaUtilRingBuffer *wavSrcRB = context->wavSrcRB;
    CLOCK_DOMAIN myCd;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_WAV_SRC);
    if (myCd != cd) {
        return(NULL);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_WAV_SRC);

    ipc = sae_getMsgBufferPayload(msg);
    audio = &ipc->audio;
    audio->clockDomain = myCd;

    if (!wavSrc->enabled) {
        audio->numChannels = 0;
        return(msg);
    }

    samplesIn = PaUtil_GetRingBufferReadAvailable(wavSrcRB);
    samplesOut = wavSrc->channels * SYSTEM_BLOCK_SIZE;

    if ((samplesIn == 0) || (samplesOut == 0)) {
        audio->numChannels = 0;
        return(msg);
    }

    if (samplesIn >= samplesOut) {
        audio->numChannels = wavSrc->channels;
        audio->numFrames = SYSTEM_BLOCK_SIZE;
        audio->wordSize = sizeof(SYSTEM_AUDIO_TYPE);
        PaUtil_ReadRingBuffer(
            wavSrcRB, audio->data, samplesOut
        );
    } else {
        wavSrcUnderflow++;
    }

    if (samplesIn < (WAV_RING_BUF_SAMPLES / 2)) {
        xTaskNotifyFromISR(context->wavSrcTaskHandle,
            WAV_TASK_AUDIO_SRC_MORE_DATA, eSetValueWithoutOverwrite, NULL
        );
    }

    return(msg);
}

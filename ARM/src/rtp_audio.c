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
#include "rtp_audio.h"
#include "rtp_stream.h"
#include "umm_malloc.h"
#include "clock_domain.h"

static unsigned rtpRxUnderflow = 0;
static unsigned rtpTxOverflow = 0;

/* Task notification values */
enum {
    RTP_TASK_NO_ACTION,
    RTP_TASK_AUDIO_RX_MORE_DATA,
    RTP_TASK_AUDIO_TX_MORE_DATA,
};

static SYSTEM_AUDIO_TYPE rxBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE txBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE txBuffer2[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];

/* This task puts RTP frames into the RTP Rx ring buffer */
portTASK_FUNCTION(rtpRxTask, pvParameters)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    RTP_STREAM *rtpRx = &context->rtpRx;
    PaUtilRingBuffer *rtpRxRB = context->rtpRxRB;
    uint32_t whatToDo;
    unsigned samplesIn;
    unsigned samplesOut;
    unsigned framesIn;
    unsigned framesOut;
    void *data;

    while (1) {
        xSemaphoreTake((SemaphoreHandle_t)rtpRx->lock, portMAX_DELAY);
        if (rtpRx->enabled) {
            samplesIn = rtpReadSamplesAvailable(rtpRx, &data);
            samplesOut = PaUtil_GetRingBufferWriteAvailable(rtpRxRB);
            while (samplesIn && (samplesOut >= samplesIn)) {
                framesIn = samplesIn / rtpRx->channels;
                framesOut = samplesOut / rtpRx->channels;
                if (framesOut > SYSTEM_BLOCK_SIZE) {
                    framesOut = SYSTEM_BLOCK_SIZE;
                }
                if (framesOut > framesIn) {
                    framesOut = framesIn;
                }
                samplesOut = framesOut * rtpRx->channels;
                if (rtpRx->wordSizeBytes == sizeof(SYSTEM_AUDIO_TYPE)) {
                    PaUtil_WriteRingBuffer(rtpRxRB, data, samplesOut);
                } else {
                    copyAndConvert(
                        data, rtpRx->wordSizeBytes, rtpRx->channels,
                        rxBuffer, sizeof(SYSTEM_AUDIO_TYPE), rtpRx->channels,
                        framesOut, false
                    );
                    PaUtil_WriteRingBuffer(rtpRxRB, rxBuffer, samplesOut);
                }
                rtpReadSamples(rtpRx, samplesOut);
                samplesIn = rtpReadSamplesAvailable(rtpRx, &data);
                samplesOut = PaUtil_GetRingBufferWriteAvailable(rtpRxRB);
                if (rtpRx->preRoll && (samplesOut < (RTP_RING_BUF_SAMPLES / 2))) {
                    rtpRx->preRoll = false;
                }
            }
        } else {
            PaUtil_FlushRingBuffer(rtpRxRB);
            rtpRx->preRoll = true;
        }
        xSemaphoreGive((SemaphoreHandle_t)rtpRx->lock);
        whatToDo = ulTaskNotifyTake(pdTRUE, 1);
    }
}

/* This task emits RTP frames from the RTP Tx ring buffer */
portTASK_FUNCTION(rtpTxTask, pvParameters)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    RTP_STREAM *rtpTx = &context->rtpTx;
    PaUtilRingBuffer *rtpTxRB = context->rtpTxRB;
    uint32_t whatToDo;
    unsigned samplesIn;
    unsigned samplesOut;
    unsigned wsize;
    bool ok;
    void *data;

    while (1) {
        xSemaphoreTake((SemaphoreHandle_t)rtpTx->lock, portMAX_DELAY);
        if (rtpTx->enabled) {
            samplesIn = PaUtil_GetRingBufferReadAvailable(rtpTxRB);
            samplesOut = rtpTx->channels * SYSTEM_BLOCK_SIZE;
            ok = true;
            while (ok && (samplesIn > samplesOut)) {
                wsize = rtpWriteSamplesAvailable(rtpTx, &data);
                if (wsize > samplesOut) {
                    wsize = samplesOut;
                }
                if (rtpTx->wordSizeBytes == sizeof(SYSTEM_AUDIO_TYPE)) {
                    PaUtil_ReadRingBuffer(rtpTxRB, data, wsize);
                } else {
                    PaUtil_ReadRingBuffer(rtpTxRB, txBuffer, wsize);
                    copyAndConvert(
                        txBuffer, sizeof(SYSTEM_AUDIO_TYPE), rtpTx->channels,
                        data, rtpTx->wordSizeBytes, rtpTx->channels,
                        wsize / rtpTx->channels, false
                    );
                }
                rtpWriteSamples(rtpTx, wsize);
                samplesIn = PaUtil_GetRingBufferReadAvailable(rtpTxRB);
            }
        } else {
            PaUtil_FlushRingBuffer(rtpTxRB);
        }
        xSemaphoreGive((SemaphoreHandle_t)rtpTx->lock);
        whatToDo = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
}

static void rtp_audio_init_stream(RTP_STREAM *rs)
{
    rs->lock =  (SemaphoreHandle_t)xSemaphoreCreateMutex();
    rs->port = 6970;
    rs->channels = 2;
    rs->wordSizeBytes = sizeof(int16_t);
}

void rtp_audio_init(APP_CONTEXT *context)
{
    uint32_t dataSize;

    /* Allocate and configure the rtp rx ring buffer.
     * The ring buffer unit of measure is in SYSTEM_AUDIO_TYPE sized
     * words
     */
    context->rtpRxRB =
        (PaUtilRingBuffer *)umm_malloc(sizeof(PaUtilRingBuffer));
    assert(context->rtpRxRB);
    dataSize = roundUpPow2(RTP_RING_BUF_SAMPLES);
    context->rtpRxRBData = umm_calloc(dataSize, sizeof(SYSTEM_AUDIO_TYPE));
    assert(context->rtpRxRBData);
    PaUtil_InitializeRingBuffer(context->rtpRxRB,
        sizeof(SYSTEM_AUDIO_TYPE), dataSize, context->rtpRxRBData);

    /* Allocate and configure the rtp tx ring buffer.
     * The ring buffer unit of measure is in SYSTEM_AUDIO_TYPE sized
     * words
     */
    context->rtpTxRB =
        (PaUtilRingBuffer *)umm_malloc(sizeof(PaUtilRingBuffer));
    assert(context->rtpTxRB);
    dataSize = roundUpPow2(RTP_RING_BUF_SAMPLES);
    context->rtpTxRBData = umm_calloc(dataSize, sizeof(SYSTEM_AUDIO_TYPE));
    assert(context->rtpTxRBData);
    PaUtil_InitializeRingBuffer(context->rtpTxRB,
        sizeof(SYSTEM_AUDIO_TYPE), dataSize, context->rtpTxRBData);

    rtp_audio_init_stream(&context->rtpRx);
    rtp_audio_init_stream(&context->rtpTx);

    xTaskCreate(rtpRxTask, "RtpRxTask", RTP_TASK_STACK_SIZE,
        context, RTP_TASK_PRIORITY, &context->rtpRxTaskHandle );
    xTaskCreate(rtpTxTask, "RtpTxTask", RTP_TASK_STACK_SIZE,
        context, RTP_TASK_PRIORITY, &context->rtpTxTaskHandle );

}

/* Transfers RTP Tx audio (ISR context) */
SAE_MSG_BUFFER *xferRtpTxAudio(APP_CONTEXT *context, SAE_MSG_BUFFER *msg,
    CLOCK_DOMAIN cd)
{
    unsigned samplesIn;
    unsigned samplesOut;
    IPC_MSG *ipc;
    IPC_MSG_AUDIO *audio;
    RTP_STREAM *rtpTx = &context->rtpTx;
    PaUtilRingBuffer *rtpTxRB = context->rtpTxRB;
    CLOCK_DOMAIN myCd;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_RTP_TX);
    if (myCd != cd) {
        return(NULL);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_RTP_TX);

    ipc = sae_getMsgBufferPayload(msg);
    audio = &ipc->audio;
    audio->clockDomain = myCd;

    if (!rtpTx->enabled) {
        return(msg);
    }

    samplesIn = audio->numChannels * audio->numFrames;
    samplesOut = PaUtil_GetRingBufferWriteAvailable(rtpTxRB);

    if ((samplesIn == 0) || (samplesOut == 0)) {
        return(msg);
    }

    if (samplesIn <= samplesOut) {
        copyAndConvert(
            audio->data, audio->wordSize, audio->numChannels,
            txBuffer2, sizeof(SYSTEM_AUDIO_TYPE), rtpTx->channels,
            audio->numFrames, true
        );
        PaUtil_WriteRingBuffer(
            rtpTxRB, txBuffer2, rtpTx->channels * audio->numFrames
        );
    } else {
        rtpTxOverflow++;
    }

    if (samplesOut < (RTP_RING_BUF_SAMPLES / 2)) {
        xTaskNotifyFromISR(context->rtpTxTaskHandle,
            RTP_TASK_AUDIO_TX_MORE_DATA, eSetValueWithoutOverwrite, NULL
        );
    }

    return(msg);
}

/* Transfers RTP Rx audio (ISR context) */
SAE_MSG_BUFFER *xferRtpRxAudio(APP_CONTEXT *context, SAE_MSG_BUFFER *msg,
    CLOCK_DOMAIN cd)
{
    unsigned samplesIn;
    unsigned samplesOut;
    IPC_MSG *ipc;
    IPC_MSG_AUDIO *audio;
    RTP_STREAM *rtpRx = &context->rtpRx;
    PaUtilRingBuffer *rtpRxRB = context->rtpRxRB;
    CLOCK_DOMAIN myCd;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_RTP_RX);
    if (myCd != cd) {
        return(NULL);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_RTP_RX);

    ipc = sae_getMsgBufferPayload(msg);
    audio = &ipc->audio;
    audio->clockDomain = myCd;

    if (!rtpRx->enabled || rtpRx->preRoll) {
        audio->numChannels = 0;
        return(msg);
    }

    samplesIn = PaUtil_GetRingBufferReadAvailable(rtpRxRB);
    samplesOut = rtpRx->channels * SYSTEM_BLOCK_SIZE;

    if ((samplesIn == 0) || (samplesOut == 0)) {
        audio->numChannels = 0;
        return(msg);
    }

    if (samplesIn >= samplesOut) {
        audio->numChannels = rtpRx->channels;
        audio->numFrames = SYSTEM_BLOCK_SIZE;
        audio->wordSize = sizeof(SYSTEM_AUDIO_TYPE);
        PaUtil_ReadRingBuffer(
            rtpRxRB, audio->data, samplesOut
        );
    } else {
        audio->numChannels = 0;
        rtpRx->preRoll = true;
        rtpRxUnderflow++;
    }

    return(msg);
}

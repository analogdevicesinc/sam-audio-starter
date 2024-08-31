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
int xferRtpTxAudio(APP_CONTEXT *context, void *audio, CLOCK_DOMAIN cd,
    unsigned *numChannels)
{
    unsigned samplesIn;
    unsigned samplesOut;
    RTP_STREAM *rtpTx = &context->rtpTx;
    PaUtilRingBuffer *rtpTxRB = context->rtpTxRB;
    CLOCK_DOMAIN myCd;
    BaseType_t wake;

    static bool first = true;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_RTP_TX);
    if (myCd != cd) {
        return(0);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_RTP_TX);

    if (!rtpTx->enabled) {
        *numChannels = 0;
        first = true;
        return(1);
    }

    samplesIn = rtpTx->channels * SYSTEM_BLOCK_SIZE;
    samplesOut = PaUtil_GetRingBufferWriteAvailable(rtpTxRB);

    if ((samplesIn == 0) || (samplesOut == 0)) {
         *numChannels = 0;
        return(1);
    }

    if (samplesIn <= samplesOut) {
        if (!first) {
            PaUtil_WriteRingBuffer(
                rtpTxRB, audio, rtpTx->channels * SYSTEM_BLOCK_SIZE
            );
        }
    } else {
        rtpTxOverflow++;
    }

    memset(audio, 0, samplesIn * sizeof(SYSTEM_AUDIO_TYPE));
    *numChannels = rtpTx->channels;
    first = false;

    if (samplesOut < (RTP_RING_BUF_SAMPLES / 2)) {
        xTaskNotifyFromISR(context->rtpTxTaskHandle,
            RTP_TASK_AUDIO_TX_MORE_DATA, eSetValueWithoutOverwrite, &wake
        );
        portYIELD_FROM_ISR(wake);
    }

    return(1);
}

/* Transfers RTP Rx audio (ISR context) */
int xferRtpRxAudio(APP_CONTEXT *context, void *audio, CLOCK_DOMAIN cd,
    unsigned *numChannels)
{
    unsigned samplesIn;
    unsigned samplesOut;
    RTP_STREAM *rtpRx = &context->rtpRx;
    PaUtilRingBuffer *rtpRxRB = context->rtpRxRB;
    CLOCK_DOMAIN myCd;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_RTP_RX);
    if (myCd != cd) {
        return(0);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_RTP_RX);

    if (!rtpRx->enabled || rtpRx->preRoll) {
        *numChannels = 0;
        return(1);
    }

    samplesIn = PaUtil_GetRingBufferReadAvailable(rtpRxRB);
    samplesOut = rtpRx->channels * SYSTEM_BLOCK_SIZE;

    if ((samplesIn == 0) || (samplesOut == 0)) {
        *numChannels = 0;
        return(1);
    }

    if (samplesIn >= samplesOut) {
        PaUtil_ReadRingBuffer(
            rtpRxRB, audio, samplesOut
        );
        *numChannels = rtpRx->channels;
    } else {
        rtpRx->preRoll = true;
        rtpRxUnderflow++;
        *numChannels = 0;
    }

    return(1);
}

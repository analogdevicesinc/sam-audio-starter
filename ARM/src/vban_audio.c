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

#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "context.h"
#include "util.h"
#include "vban_audio.h"
#include "vban_stream.h"
#include "umm_malloc.h"
#include "clock_domain.h"

static unsigned vbanRxUnderflow = 0;
static unsigned vbanTxOverflow = 0;

/* Task notification values */
enum {
    VBAN_TASK_NO_ACTION,
    VBAN_TASK_AUDIO_RX_MORE_DATA,
    VBAN_TASK_AUDIO_TX_MORE_DATA,
};

static SYSTEM_AUDIO_TYPE rxBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE txBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];

/* This task puts VBAN frames into the VBAN Rx ring buffer */
portTASK_FUNCTION(vbanRxTask, pvParameters)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    VBAN_STREAM *vbanRx = &context->vbanRx;
    PaUtilRingBuffer *vbanRxRB = context->vbanRxRB;
    uint32_t whatToDo;
    unsigned samplesIn;
    unsigned samplesOut;
    unsigned framesIn;
    unsigned framesOut;
    void *data;

    while (1) {
        xSemaphoreTake((SemaphoreHandle_t)vbanRx->lock, portMAX_DELAY);
        if (vbanRx->enabled) {
            samplesIn = vbanReadSamplesAvailable(vbanRx, &data);
            samplesOut = PaUtil_GetRingBufferWriteAvailable(vbanRxRB);
            while (samplesIn && (samplesOut >= samplesIn)) {
                framesIn = samplesIn / vbanRx->streamChannels;
                framesOut = samplesOut / vbanRx->channels;
                if (framesOut > SYSTEM_BLOCK_SIZE) {
                    framesOut = SYSTEM_BLOCK_SIZE;
                }
                if (framesOut > framesIn) {
                    framesOut = framesIn;
                }
                copyAndConvert(
                    data, vbanRx->wordSizeBytes, vbanRx->streamChannels,
                    rxBuffer, sizeof(SYSTEM_AUDIO_TYPE), vbanRx->channels,
                    framesOut, false
                );
                PaUtil_WriteRingBuffer(vbanRxRB, rxBuffer, framesOut * vbanRx->channels);
                vbanReadSamples(vbanRx, framesOut * vbanRx->streamChannels);
                samplesOut = PaUtil_GetRingBufferWriteAvailable(vbanRxRB);
                samplesIn = vbanReadSamplesAvailable(vbanRx, &data);
                if (vbanRx->preRoll && (samplesOut < (VBAN_RING_BUF_SAMPLES / 2))) {
                    vbanRx->preRoll = false;
                }
            }
        } else {
            PaUtil_FlushRingBuffer(vbanRxRB);
            vbanRx->preRoll = true;
        }
        xSemaphoreGive((SemaphoreHandle_t)vbanRx->lock);
        whatToDo = ulTaskNotifyTake(pdTRUE, 1);
    }
}

/* This task emits VBAN frames from the VBAN Tx ring buffer */
portTASK_FUNCTION(vbanTxTask, pvParameters)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    VBAN_STREAM *vbanTx = &context->vbanTx;
    PaUtilRingBuffer *vbanTxRB = context->vbanTxRB;
    uint32_t whatToDo;
    unsigned samplesIn;
    unsigned samplesOut;
    unsigned wsize;
    bool ok;
    void *data;

    while (1) {
        xSemaphoreTake((SemaphoreHandle_t)vbanTx->lock, portMAX_DELAY);
        if (vbanTx->enabled) {
            samplesIn = PaUtil_GetRingBufferReadAvailable(vbanTxRB);
            samplesOut = vbanTx->channels * SYSTEM_BLOCK_SIZE;
            ok = true;
            while (ok && (samplesIn > samplesOut)) {
                wsize = vbanWriteSamplesAvailable(vbanTx, &data);
                if (wsize > samplesOut) {
                    wsize = samplesOut;
                }
                if (vbanTx->wordSizeBytes == sizeof(SYSTEM_AUDIO_TYPE)) {
                    PaUtil_ReadRingBuffer(vbanTxRB, data, wsize);
                } else {
                    PaUtil_ReadRingBuffer(vbanTxRB, txBuffer, wsize);
                    copyAndConvert(
                        txBuffer, sizeof(SYSTEM_AUDIO_TYPE), vbanTx->channels,
                        data, vbanTx->wordSizeBytes, vbanTx->channels,
                        wsize / vbanTx->channels, false
                    );
                }
                vbanWriteSamples(vbanTx, wsize);
                samplesIn = PaUtil_GetRingBufferReadAvailable(vbanTxRB);
            }
        } else {
            PaUtil_FlushRingBuffer(vbanTxRB);
        }
        xSemaphoreGive((SemaphoreHandle_t)vbanTx->lock);
        whatToDo = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
}

static void vban_audio_init_stream(VBAN_STREAM *rs)
{
    rs->lock =  (SemaphoreHandle_t)xSemaphoreCreateMutex();
    rs->port = 6980;
    rs->channels = 2;
    rs->wordSizeBytes = sizeof(int16_t);
}

void vban_audio_init(APP_CONTEXT *context)
{
    uint32_t dataSize;

    /* Allocate and configure the vban rx ring buffer.
     * The ring buffer unit of measure is in SYSTEM_AUDIO_TYPE sized
     * words
     */
    context->vbanRxRB =
        (PaUtilRingBuffer *)umm_malloc(sizeof(PaUtilRingBuffer));
    assert(context->vbanRxRB);
    dataSize = roundUpPow2(VBAN_RING_BUF_SAMPLES);
    context->vbanRxRBData = umm_calloc(dataSize, sizeof(SYSTEM_AUDIO_TYPE));
    assert(context->vbanRxRBData);
    PaUtil_InitializeRingBuffer(context->vbanRxRB,
        sizeof(SYSTEM_AUDIO_TYPE), dataSize, context->vbanRxRBData);

    /* Allocate and configure the vban tx ring buffer.
     * The ring buffer unit of measure is in SYSTEM_AUDIO_TYPE sized
     * words
     */
    context->vbanTxRB =
        (PaUtilRingBuffer *)umm_malloc(sizeof(PaUtilRingBuffer));
    assert(context->vbanTxRB);
    dataSize = roundUpPow2(VBAN_RING_BUF_SAMPLES);
    context->vbanTxRBData = umm_calloc(dataSize, sizeof(SYSTEM_AUDIO_TYPE));
    assert(context->vbanTxRBData);
    PaUtil_InitializeRingBuffer(context->vbanTxRB,
        sizeof(SYSTEM_AUDIO_TYPE), dataSize, context->vbanTxRBData);

    vban_audio_init_stream(&context->vbanRx);
    vban_audio_init_stream(&context->vbanTx);

    xTaskCreate(vbanRxTask, "vbanRxTask", VBAN_TASK_STACK_SIZE,
        context, VBAN_TASK_PRIORITY, &context->vbanRxTaskHandle );
    xTaskCreate(vbanTxTask, "vbanTxTask", VBAN_TASK_STACK_SIZE,
        context, VBAN_TASK_PRIORITY, &context->vbanTxTaskHandle );

}

/* Transfers VBAN Tx audio (ISR context) */
int xferVbanTxAudio(APP_CONTEXT *context, void *audio, CLOCK_DOMAIN cd,
    unsigned *numChannels)
{
    unsigned samplesIn;
    unsigned samplesOut;
    VBAN_STREAM *vbanTx = &context->vbanTx;
    PaUtilRingBuffer *vbanTxRB = context->vbanTxRB;
    CLOCK_DOMAIN myCd;
    BaseType_t wake;

    static bool first = true;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_VBAN_TX);
    if (myCd != cd) {
        return(0);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_VBAN_TX);

    if (!vbanTx->enabled) {
        *numChannels = 0;
        first = true;
        return(1);
    }

    samplesIn = vbanTx->channels * SYSTEM_BLOCK_SIZE;
    samplesOut = PaUtil_GetRingBufferWriteAvailable(vbanTxRB);

    if ((samplesIn == 0) || (samplesOut == 0)) {
         *numChannels = 0;
        return(1);
    }

    if (samplesIn <= samplesOut) {
        if (!first) {
            PaUtil_WriteRingBuffer(
                vbanTxRB, audio, vbanTx->channels * SYSTEM_BLOCK_SIZE
            );
        }
    } else {
        vbanTxOverflow++;
    }

    memset(audio, 0, samplesIn * sizeof(SYSTEM_AUDIO_TYPE));
    *numChannels = vbanTx->channels;
    first = false;

    if (samplesOut < (VBAN_RING_BUF_SAMPLES / 2)) {
        xTaskNotifyFromISR(context->vbanTxTaskHandle,
            VBAN_TASK_AUDIO_TX_MORE_DATA, eSetValueWithoutOverwrite, &wake
        );
        portYIELD_FROM_ISR(wake);
    }

    return(1);
}

/* Transfers VBAN Rx audio (ISR context) */
int xferVbanRxAudio(APP_CONTEXT *context, void *audio, CLOCK_DOMAIN cd,
    unsigned *numChannels)
{
    unsigned samplesIn;
    unsigned samplesOut;
    VBAN_STREAM *vbanRx = &context->vbanRx;
    PaUtilRingBuffer *vbanRxRB = context->vbanRxRB;
    CLOCK_DOMAIN myCd;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_VBAN_RX);
    if (myCd != cd) {
        return(0);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_VBAN_RX);

    if (!vbanRx->enabled || vbanRx->preRoll) {
        *numChannels = 0;
        return(1);
    }

    samplesIn = PaUtil_GetRingBufferReadAvailable(vbanRxRB);
    samplesOut = vbanRx->channels * SYSTEM_BLOCK_SIZE;

    if ((samplesIn == 0) || (samplesOut == 0)) {
        *numChannels = 0;
        return(1);
    }

    if (samplesIn >= samplesOut) {
        PaUtil_ReadRingBuffer(
            vbanRxRB, audio, samplesOut
        );
        *numChannels = vbanRx->channels;
    } else {
        vbanRx->preRoll = true;
        vbanRxUnderflow++;
        *numChannels = 0;
    }

    return(1);
}

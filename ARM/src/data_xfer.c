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
#include <services/gpio/adi_gpio.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "context.h"
#include "init.h"
#include "util.h"
#include "data_file.h"
#include "data_xfer.h"
#include "data_file.h"
#include "umm_malloc.h"
#include "clock_domain.h"
#include "syslog.h"

static unsigned fileSrcUnderflow = 0;
static unsigned fileSinkOverflow = 0;
static bool newStateSrc = false;

static unsigned bytesStreamed = 0;

/* Task notification values */
enum
{
    FILE_TASK_NO_ACTION,
    FILE_TASK_AUDIO_SRC_MORE_DATA,
    FILE_TASK_AUDIO_SINK_MORE_DATA,
};

#define FILE_READ_BUF_BYTES (FILE_READ_BUF_SIZE * sizeof(SYSTEM_AUDIO_TYPE))

static SYSTEM_AUDIO_TYPE srcBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
// static SYSTEM_AUDIO_TYPE srcBuffer2[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE sinkBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE sinkBuffer2[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];

/*
 * This task keeps the file src ring buffer full
 *
 * if file transfer is enabled (via shell command), then this task will read
 *   the file contents and fill the ring buffer if 'enabled' flag is true.
 *
 */
static bool streaming = false;

portTASK_FUNCTION(fileSrcTask, pvParameters)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    DATA_FILE *fileSrc = &context->fileSrc;
    PaUtilRingBuffer *fileSrcRB = context->fileSrcRB;
    uint32_t whatToDo;
    unsigned samplesOut;
    size_t rsize;
    bool rbEmpty;

    fileSrc->state = FILE_STREAM_STOP;

    while (1)
    {
        xSemaphoreTake(fileSrc->lock, portMAX_DELAY);

        /* if file is opened, read and store into ring buffer */
        if (fileSrc->enabled)
        {
            if (fileSrc->sync) /* start of file, prepend Start SYNC */
            {
                samplesOut = PaUtil_GetRingBufferWriteAvailable(fileSrcRB);
                if (samplesOut >= (2 * sizeof(SYSTEM_AUDIO_TYPE)))
                {
                    srcBuffer[0] = STREAM_START_SYNC_1;
                    srcBuffer[1] = STREAM_START_SYNC_2;
                    PaUtil_WriteRingBuffer(fileSrcRB, srcBuffer, 2);
                    syslog_printf("Stream Start\n");
                }
                fileSrc->sync = false;
                bytesStreamed = 0;
                streaming = false;
            }
            samplesOut = PaUtil_GetRingBufferWriteAvailable(fileSrcRB);
            if (samplesOut >= FILE_READ_BUF_BYTES)
            {
                /* read bytes from SD-Card */
                rsize = readData(fileSrc, srcBuffer, FILE_READ_BUF_BYTES);
                if ((rsize < FILE_READ_BUF_BYTES))
                {
                    if (fileSrc->eof) {
                        syslog_printf("Data File closed: %s (%d)\n", fileSrc->fname, fileSrc->byteCount);
                        closeData(fileSrc);
                    } else {
                        if (fileSrc->loop) {
                            syslog_print("Looping File...\n");
                        }
                    }
                }
                /* write 'samples' to ring buffer */
                data_stream_write(rsize, fileSrc->eof, fileSrcRB, srcBuffer, FILE_READ_BUF_SIZE);
            }
        }

        switch (fileSrc->state)
        {
        case FILE_STREAM_START:
            //            fileSrcUnderflow = 0;
            /* start sync has not been sent, wait here until ready to send */
            bytesStreamed = 0;
            break;

        case FILE_STREAM_PAUSED:
            if (newStateSrc)
            {
                syslog_printf("Src paused, %d\n", bytesStreamed * 4);
                newStateSrc = false;
            }
        /* continue reading file druing pause */
        case FILE_STREAM_ACTIVE:
            if (newStateSrc)
            {
                syslog_printf("Src streaming %d\n", bytesStreamed * 4);
                newStateSrc = false;
            }
            break;

        case FILE_STREAM_STOP:
            /* TODO:  where to flush buffer */
            if (newStateSrc)
            {
                syslog_printf("Src stream end %d, %d\n", fileSrc->byteCount, bytesStreamed * 4);
                newStateSrc = false;
                bytesStreamed = 0;
            }
            if (rbEmpty)
            {
                PaUtil_FlushRingBuffer(fileSrcRB);
                rbEmpty = false;
            }
            break;

        default:
            break;
        }

#define SYSLOG_FILE_SRC_REPORT_STATE
#if defined(SYSLOG_FILE_SRC_REPORT_STATE)

#endif
        xSemaphoreGive(fileSrc->lock);
        whatToDo = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
}

/* This task keeps the data file sink ring buffer empty */
portTASK_FUNCTION(fileSinkTask, pvParameters)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    DATA_FILE *fileSink = &context->fileSink;
    PaUtilRingBuffer *fileSinkRB = context->fileSinkRB;
    uint32_t whatToDo;
    unsigned samplesIn;
    unsigned samplesOut;
    bool ok;
    bool eof = false;
    unsigned samplesRead;

    //    fileSink->state = FILE_STREAM_IDLE;

    while (1)
    {
        xSemaphoreTake(fileSink->lock, portMAX_DELAY);
        switch (fileSink->state)
        {
        case FILE_STREAM_IDLE:
            break;
        case FILE_STREAM_START:
            if (fileSink->enabled)
            {
                samplesIn = PaUtil_GetRingBufferReadAvailable(fileSinkRB);
                //                samplesOut = fileSink->channels * SYSTEM_BLOCK_SIZE;
                samplesOut = 32 * SYSTEM_BLOCK_SIZE;
                ok = true;
                while (ok && (samplesIn >= (samplesOut + (STREAM_SYNC_WORD_COUNT) * sizeof(SYSTEM_AUDIO_TYPE))))
                {
                    samplesRead = PaUtil_ReadRingBuffer(
                        fileSinkRB, sinkBuffer, samplesOut);
                    if (samplesRead != samplesOut)
                    {
                        asm("nop;");
                    }
                    eof = data_file_write(fileSink, sinkBuffer, samplesOut);
                    samplesIn = PaUtil_GetRingBufferReadAvailable(fileSinkRB);
                    if (eof)
                    {
                        fileSink->state = FILE_STREAM_EOF;
                        syslog_printf("Data File closed: %s\n", fileSink->fname);
                        ok = false;
                    }
                }
            }
            else
            {
                PaUtil_FlushRingBuffer(fileSinkRB);
            }
            break;
        case FILE_STREAM_ACTIVE:

            if (eof)
            {
                fileSink->state = FILE_STREAM_EOF;
            }
            break;
        case FILE_STREAM_EOF:
            //            closeData(fileSink);
            fileSink->state = FILE_STREAM_STOP;
            break;
        case FILE_STREAM_STOP:
            fileSink->state = FILE_STREAM_IDLE;
            break;
        default:
            break;
        }
        xSemaphoreGive(fileSink->lock);
        whatToDo = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
}

void data_file_init(APP_CONTEXT *context)
{
    uint32_t dataSize;

    /* Allocate and configure the data file source ring buffer.
     * The ring buffer unit of measure is in SYSTEM_AUDIO_TYPE sized
     * words
     */
    context->fileSrcRB =
        (PaUtilRingBuffer *)umm_malloc(sizeof(PaUtilRingBuffer));
    assert(context->fileSrcRB);
    dataSize = roundUpPow2(FILE_RING_BUF_SAMPLES);
    context->fileSrcRBData = umm_calloc(dataSize, sizeof(SYSTEM_AUDIO_TYPE));
    assert(context->fileSrcRBData);
    PaUtil_InitializeRingBuffer(context->fileSrcRB,
                                sizeof(SYSTEM_AUDIO_TYPE), dataSize, context->fileSrcRBData);

    /* Allocate and configure the data file sink ring buffer.
     * The ring buffer unit of measure is in SYSTEM_AUDIO_TYPE sized
     * words
     */
    context->fileSinkRB =
        (PaUtilRingBuffer *)umm_malloc(sizeof(PaUtilRingBuffer));
    assert(context->fileSinkRB);
    dataSize = roundUpPow2(FILE_RING_BUF_SAMPLES);
    context->fileSinkRBData = umm_calloc(dataSize, sizeof(SYSTEM_AUDIO_TYPE));
    assert(context->fileSinkRBData);
    PaUtil_InitializeRingBuffer(context->fileSinkRB,
                                sizeof(SYSTEM_AUDIO_TYPE), dataSize, context->fileSinkRBData);

    context->fileSrc.lock = xSemaphoreCreateMutex();
    context->fileSink.lock = xSemaphoreCreateMutex();

    xTaskCreate(fileSrcTask, "FileSrcTask", FILE_TASK_STACK_SIZE,
                context, FILE_TASK_PRIORITY, &context->fileSrcTaskHandle);
    xTaskCreate(fileSinkTask, "FileSinkTask", FILE_TASK_STACK_SIZE,
                context, FILE_TASK_PRIORITY, &context->fileSinkTaskHandle);
}

/* write the SD-Card data to the ring buffer adjust file to pad last byte(s) */
static void data_stream_write(size_t rsize, bool eof, PaUtilRingBuffer *rbuf,
                              void *buf, ring_buffer_size_t samplesIn)
{
    uint8_t adjust;
    uint8_t *bytePtr = buf;
    unsigned samplesRead;

    /* rsize is the number of bytes in the buffer */
    if (!eof)
    {
        /* full block of samples */
        PaUtil_WriteRingBuffer(rbuf, buf, samplesIn);
    }
    else
    {
        /* read less than full buffer, should be end of file */
        samplesRead = rsize / sizeof(SYSTEM_AUDIO_TYPE); /* sample count */
        /* calculate how many bytes to pad end of buffer */
        adjust = sizeof(SYSTEM_AUDIO_TYPE) - rsize % sizeof(SYSTEM_AUDIO_TYPE);
        if (adjust < sizeof(SYSTEM_AUDIO_TYPE))
        {
            bytePtr += rsize;
            for (int i = 0; i < adjust; i++)
            {
                *bytePtr++ = 0x00; /* pad buffer to make last sample */
            }
            samplesRead++;
        }
        /* write buffer and sync words */
        PaUtil_WriteRingBuffer(rbuf, buf, samplesRead);
        srcBuffer[0] = STREAM_STOP_SYNC_1;
        srcBuffer[1] = STREAM_STOP_SYNC_2;
        PaUtil_WriteRingBuffer(rbuf, buf, 2);
    }
}

/* write the ring buffer to the SD-Card file if sync found */
static bool data_file_write(DATA_FILE *fileSink, SYSTEM_AUDIO_TYPE *samples,
                            ring_buffer_size_t samplesOut)
{
    unsigned i;
    static bool synced = false;
    static bool sync1 = false;
    bool eof = false;
    unsigned j = 0;

    /* look for sync in the buffer of samples */
    for (i = 0; i < samplesOut; i++)
    {
        if (synced)
        {
            /* check for ending sync, else write sample */
            if (sync1 && samples[i] == STREAM_STOP_SYNC_2)
            {
                /* end of file sync */
                eof = true;
                sync1 = false;
                synced = false;
                syslog_printf("file STOP stream sync\n");
            }
            else if (samples[i] == STREAM_STOP_SYNC_1)
            {
                if (sync1)
                {
                    /* repeated SYNC_1, it was a sample */
                    sinkBuffer2[j++] = samples[i];
                }
                else
                {
                    sync1 = true;
                }
            }
            else
            {
                /* write the sample */
                sinkBuffer2[j++] = samples[i];
                sync1 = false;
            }
        }
        else if (!eof)
        { /* not sync'd, check for starting sync */
            if (samples[i] == STREAM_START_SYNC_1)
            {
                sync1 = true;
                //                adi_gpio_Set(ADI_GPIO_PORT_D, ADI_GPIO_PIN_3);
            }
            else if (sync1 && (samples[i] == STREAM_START_SYNC_2))
            {
                /* sync occurred, now active */
                synced = true;
                sync1 = false;
                syslog_printf("file START stream synced\n");
                //                adi_gpio_Clear(ADI_GPIO_PORT_D, ADI_GPIO_PIN_3);
            }
            else
            {
                sync1 = false;
            }
            eof = false;
        }
    }
    if (j > 0)
    {
        writeData(fileSink, sinkBuffer2, j);
        if (eof)
        {
            closeData(fileSink);
            syslog_printf("Sink overflows: %d, Src underflows: %d\n", fileSinkOverflow, fileSrcUnderflow);
        }
    }
    return eof;
}

/* Transfers data file Sink (ISR context) */
SAE_MSG_BUFFER *xferFileSinkData(APP_CONTEXT *context, SAE_MSG_BUFFER *msg,
                                 CLOCK_DOMAIN cd)
{
    unsigned samplesIn;
    unsigned samplesOut;
    IPC_MSG *ipc;
    IPC_MSG_AUDIO *audio;
    DATA_FILE *fileSink = &context->fileSink;
    PaUtilRingBuffer *fileSinkRB = context->fileSinkRB;
    CLOCK_DOMAIN myCd;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_FILE_SINK);
    if (myCd != cd)
    {
        return (NULL);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_FILE_SINK);

    ipc = sae_getMsgBufferPayload(msg);
    audio = &ipc->audio;
    audio->clockDomain = myCd;

    if (!fileSink->enabled)
    {
        return (msg);
    }

    samplesIn = audio->numChannels * audio->numFrames;
    samplesOut = PaUtil_GetRingBufferWriteAvailable(fileSinkRB);

    if (samplesOut == 0)
    {
        asm("nop;");
    }

    if ((samplesIn == 0) || (samplesOut == 0))
    {
        return (msg);
    }

    if (samplesIn <= samplesOut)
    {
        copyAndConvert(
            audio->data, audio->wordSize, audio->numChannels,
            sinkBuffer, fileSink->wordSizeBytes, fileSink->channels,
            audio->numFrames, true);
        PaUtil_WriteRingBuffer(
            fileSinkRB, sinkBuffer, fileSink->channels * audio->numFrames);
    }
    else
    {
        fileSinkOverflow++;
    }

    if (samplesOut < (FILE_RING_BUF_SAMPLES / 2))
    {
        xTaskNotifyFromISR(context->fileSinkTaskHandle,
                           FILE_TASK_AUDIO_SINK_MORE_DATA, eSetValueWithoutOverwrite, NULL);
    }

    return (msg);
}

/* Transfers file Src data (ISR context) */
SAE_MSG_BUFFER *xferFileSrcData(APP_CONTEXT *context, SAE_MSG_BUFFER *msg,
                                CLOCK_DOMAIN cd)
{
    unsigned samplesIn;
    unsigned samplesOut;
    IPC_MSG *ipc;
    IPC_MSG_AUDIO *audio;
    DATA_FILE *fileSrc = &context->fileSrc;
    PaUtilRingBuffer *fileSrcRB = context->fileSrcRB;
    CLOCK_DOMAIN myCd;


    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_FILE_SRC);
    if (myCd != cd)
    {
        return (NULL);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_FILE_SRC);

    ipc = sae_getMsgBufferPayload(msg);
    audio = &ipc->audio;
    audio->clockDomain = myCd;

#if 0
    if (!fileSrc->enabled)
    {
        audio->numChannels = 0;
        return (msg);
    }
#endif

    samplesOut = fileSrc->channels * SYSTEM_BLOCK_SIZE;

    /* check for receiver ready for data (A2B GPIO over distance)*/
    if (okToStream())
    {
        /* OK to stream */
        if (fileSrc->state == FILE_STREAM_PAUSED)
        {
            /* send re-start sync words */
            memset(audio->data,
                   0,
                   audio->numChannels * audio->numFrames * audio->wordSize);
            audio->data[samplesOut - 2] = STREAM_RESTART_SYNC_1;
            audio->data[samplesOut - 1] = STREAM_RESTART_SYNC_2;
            fileSrc->state = FILE_STREAM_ACTIVE;
            newStateSrc = true;
        }
        else if (fileSrc->state == FILE_STREAM_ACTIVE)
        {
            /* start sync already sent, continue sending */
            /* OK to send stream data, receiver is ready */
            samplesIn = PaUtil_GetRingBufferReadAvailable(fileSrcRB);

            audio->numChannels = fileSrc->channels;
            audio->numFrames = SYSTEM_BLOCK_SIZE;
            audio->wordSize = sizeof(SYSTEM_AUDIO_TYPE);
            if (samplesIn >= samplesOut)
            {
                /* read file source ring buffer and store in audio buffer */

                PaUtil_ReadRingBuffer(
                    fileSrcRB, audio->data, samplesOut);
                bytesStreamed += samplesOut;
            }
            else if (fileSrc->eof)
            {
                PaUtil_ReadRingBuffer(
                    fileSrcRB, audio->data, samplesIn);
                bytesStreamed += samplesIn;
                newStateSrc = true;
                fileSrc->state = FILE_STREAM_STOP;
                streaming = false;
                fileSrc->enabled = false;
            }
            else
            {
                fileSrcUnderflow++;
            }
        }
        else if (fileSrc->state == FILE_STREAM_START)
        {
            if (!streaming)
            {
                streaming = true;
                /* need to send start sync */
                memset(audio->data,
                       0,
                       audio->numChannels * audio->numFrames * audio->wordSize);
                //                audio->data[samplesOut - 2] = STREAM_START_SYNC_1;
                //                audio->data[samplesOut - 1] = STREAM_START_SYNC_2;
                fileSrc->state = FILE_STREAM_ACTIVE;
                newStateSrc = true;
            }
        }
        else if (fileSrc->state == FILE_STREAM_STOP)
        {
            memset(audio->data,
                   0,
                   audio->numChannels * audio->numFrames * audio->wordSize);
        }
    }
    else
    {
        /* NOT OK to stream */
        memset(audio->data,
               0,
               audio->numChannels * audio->numFrames * audio->wordSize);
        if (fileSrc->state == FILE_STREAM_ACTIVE)
        {
            audio->data[0] = STREAM_PAUSE_SYNC_1;
            audio->data[1] = STREAM_PAUSE_SYNC_2;
            fileSrc->state = FILE_STREAM_PAUSED;
            newStateSrc = true;
        }
        else
        {
            asm("nop;");
        }
    }
#if 0
    if (samplesIn < (FILE_RING_BUF_SAMPLES / 2))
    {
        xTaskNotifyFromISR(context->fileSrcTaskHandle,
                           FILE_TASK_AUDIO_SRC_MORE_DATA, eSetValueWithoutOverwrite, NULL);
    }
#endif
    return (msg);
}

bool okToStream(void)
{
    uint32_t a2bGpio = 0;
    /* light LED 3 if A2B IO7 is asserted */
    adi_gpio_GetData(A2B_IO7_PORT, &a2bGpio);
    if (a2bGpio & A2B_IO7_PIN)
    {
        /* OK to Send */
        adi_gpio_Set(LED_PORT, LED3);
        return true;
    }
    else
    {
        /* NOT OK to Send */
        adi_gpio_Clear(LED_PORT, LED3);
        return false;
    }
}

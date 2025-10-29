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

/* Standard includes. */
#include <stdint.h>

/* CCES includes */
#include <services/int/adi_sec.h>
#define DO_CYCLE_COUNTS
#include <cycle_count.h>

/* Simple service includes */
#include "sae.h"

/* IPC includes */
#include "ipc.h"

SAE_CONTEXT *saeContext = NULL;
IPC_MSG_AUDIO *streamInfo[IPC_STREAM_ID_MAX];
SAE_MSG_BUFFER *cyclesMsg = NULL;

/***********************************************************************
 * Audio functions
 **********************************************************************/

/*
 * In these functions, IN and OUT are relative to the SHARC.  This
 * code uses src and sink to help minimize confusion.  In all cases,
 * src buffers are copied to sink buffers.
 */
#pragma optimize_for_speed
static void processAudio(IPC_MSG_PROCESS_AUDIO *process)
{
    uint8_t clockDomain = process->clockDomain;
    IPC_MSG_AUDIO *src, *sink, *stream;
    unsigned i, channel, frame, channels;
    int32_t *in, *out;
    cycle_t startCycles;
    cycle_t finalCycles;

    START_CYCLE_COUNT(startCycles);

    src = streamInfo[IPC_STREAMID_SHARC0_IN];
    sink = streamInfo[IPC_STREAMID_SHARC0_OUT];

    if ((src == NULL) || (sink == NULL)) {
        return;
    }
    if (src->clockDomain != clockDomain) {
        return;
    }
    if (sink->clockDomain != clockDomain) {
        return;
    }

#if 1
    if (src->numFrames != sink->numFrames) {
        return;
    }
    if (src->wordSize != sink->wordSize) {
        return;
    }
    if (src->wordSize != sizeof(int32_t)) {
        return;
    }
#endif

    channels = (src->numChannels < sink->numChannels) ?
        src->numChannels : sink->numChannels;
    in = src->data;
    out = sink->data;

    for (frame = 0; frame < src->numFrames; frame++) {
        for (channel = 0; channel < channels; channel++) {
            *(out + channel) = *(in + channel);
        }
        in += src->numChannels;
        out += sink->numChannels;
    }

    /* Clear the contents of src/in buffer */
    memset(src->data, 0,
                src->numChannels * src->numFrames * src->wordSize);

    /* Invalidate all streams associated with this clock domain */
    for (i = 0; i < IPC_STREAM_ID_MAX; i++) {
        stream = streamInfo[i];
        if (stream && (stream->clockDomain == clockDomain)) {
            streamInfo[i] = NULL;
        }
    }

    STOP_CYCLE_COUNT(finalCycles, startCycles);

    if (cyclesMsg &&(clockDomain < IPC_CYCLE_DOMAIN_MAX)) {
        IPC_MSG *msg = sae_getMsgBufferPayload(cyclesMsg);
        msg->cycles.cycles[clockDomain] = finalCycles;
    }
}

static void newAudio(IPC_MSG_AUDIO *audio)
{
    bool clear = false;
    bool unknown = false;

    switch (audio->streamID) {
        case IPC_STREAMID_SHARC0_IN:
            break;
        case IPC_STREAMID_SHARC0_OUT:
            clear = true;
            break;
        default:
            unknown = true;
            break;
    }

    if (!unknown) {
        streamInfo[audio->streamID] = audio;
        if (clear) {
            memset(audio->data, 0,
                audio->numChannels * audio->numFrames * audio->wordSize);
        }
    }
}

/***********************************************************************
 * Application IPC functions
 **********************************************************************/
SAE_RESULT ipcToCore(SAE_CONTEXT *saeContext, SAE_MSG_BUFFER *ipcBuffer,
    SAE_CORE_IDX core)
{
    SAE_RESULT result;

    result = sae_sendMsgBuffer(saeContext, ipcBuffer, core, true);
    if (result != SAE_RESULT_OK) {
        sae_unRefMsgBuffer(saeContext, ipcBuffer);
    }

    return(result);
}

SAE_RESULT quickIpcToCore(SAE_CONTEXT *saeContext, enum IPC_TYPE type,
    SAE_CORE_IDX core)
{
    SAE_MSG_BUFFER *ipcBuffer;
    SAE_RESULT result;
    IPC_MSG *msg;

    ipcBuffer = sae_createMsgBuffer(saeContext, sizeof(*msg), (void **)&msg);
    if (ipcBuffer) {
        msg->type = type;
        result = ipcToCore(saeContext, ipcBuffer, core);
    } else {
        result = SAE_RESULT_ERROR;
    }

    return(result);
}

static void ipcMsgRx(SAE_CONTEXT *saeContext, SAE_MSG_BUFFER *buffer,
    void *payload, void *usrPtr)
{
    SAE_MSG_BUFFER *ipcBuffer;
    SAE_RESULT result;
    IPC_MSG *msg = (IPC_MSG *)payload;
    IPC_MSG *replyMsg;

    /* Process the message */
    switch (msg->type) {
        case IPC_TYPE_PING:
            ipcBuffer = sae_createMsgBuffer(saeContext, sizeof(*replyMsg), (void **)&replyMsg);
            replyMsg->type = IPC_TYPE_PING;
            result = sae_sendMsgBuffer(saeContext, ipcBuffer, IPC_CORE_ARM, true);
            if (result != SAE_RESULT_OK) {
                sae_unRefMsgBuffer(saeContext, ipcBuffer);
            }
            break;
        case IPC_TYPE_PROCESS_AUDIO:
            processAudio((IPC_MSG_PROCESS_AUDIO *)&msg->process);
            break;
        case IPC_TYPE_AUDIO:
            newAudio((IPC_MSG_AUDIO *)&msg->audio);
            break;
        case IPC_TYPE_CYCLES:
            if (cyclesMsg) {
                sae_refMsgBuffer(saeContext, cyclesMsg);
                result = sae_sendMsgBuffer(saeContext, cyclesMsg, IPC_CORE_ARM, true);
                if (result != SAE_RESULT_OK) {
                    sae_unRefMsgBuffer(saeContext, cyclesMsg);
                }
            }
            break;
        default:
            break;
    }

    /* Done with the message so decrement the ref count */
    result = sae_unRefMsgBuffer(saeContext, buffer);
}

int main(int argc, char **argv)
{
    SAE_RESULT ok = SAE_RESULT_OK;
    IPC_MSG *msg;

    /* Initialize the SEC */
    adi_sec_Init();

    /* Initialize the SHARC Audio Engine */
    sae_initialize(&saeContext, IPC_CORE_SHARC0, false);

    /* Create a persistent message for cycle counts */
    cyclesMsg = sae_createMsgBuffer(saeContext, sizeof(*msg), (void **)&msg);
    if (cyclesMsg) {
        msg->type = IPC_TYPE_CYCLES;
        msg->cycles.core = IPC_CORE_SHARC0;
        msg->cycles.max = IPC_CYCLE_DOMAIN_MAX;
    }

    /* Register an IPC message Rx callback */
    sae_registerMsgReceivedCallback(saeContext, ipcMsgRx, NULL);

    /* Tell the ARM we're ready */
    quickIpcToCore(saeContext, IPC_TYPE_SHARC0_READY, IPC_CORE_ARM);

    while(1) {
        asm("nop;");
    };
}

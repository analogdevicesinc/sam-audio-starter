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

/* Standard includes. */

/* CCES includes */
#include <services/int/adi_sec.h>

/* Simple service includes */
#include "sae.h"

/* IPC includes */
#include "ipc.h"

SAE_CONTEXT *saeContext;

static void processAudio(IPC_MSG_AUDIO *audio)
{
    uint8_t streamID = audio->streamID;
    uint8_t numChannels = audio->numChannels;
    uint8_t numFrames = audio->numFrames;
    uint8_t wordSize = audio->wordSize;
    int32_t *data = audio->data;

    switch (audio->streamID) {
        case IPC_STREAMID_CODEC_IN:
            break;
        case IPC_STREAMID_CODEC_OUT:
            break;
        case IPC_STREAMID_SPDIF_IN:
            break;
        case IPC_STREAMID_SPDIF_OUT:
            break;
        case IPC_STREAMID_A2B_IN:
            break;
        case IPC_STREAMID_A2B_OUT:
            break;
        case IPC_STREAMID_USB_RX:
            break;
        case IPC_STREAMID_USB_TX:
            break;
        case IPC_STREAM_ID_WAV_SRC:
            break;
        case IPC_STREAM_ID_WAV_SINK:
            break;
        default:
            break;
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
    IPC_MSG_AUDIO *audio;
    IPC_MSG *replyMsg;

    /* Process the message */
    switch (msg->type) {
        case IPC_TYPE_PING:
            ipcBuffer = sae_createMsgBuffer(saeContext, sizeof(*replyMsg), (void **)&replyMsg);
            if (ipcBuffer) {
                replyMsg->type = IPC_TYPE_PING;
                result = sae_sendMsgBuffer(saeContext, ipcBuffer, SAE_CORE_IDX_0, true);
                if (result != SAE_RESULT_OK) {
                    sae_unRefMsgBuffer(saeContext, ipcBuffer);
                }
            }
            break;
        case IPC_TYPE_AUDIO:
            audio = (IPC_MSG_AUDIO *)&msg->audio;
            processAudio(audio);
            break;
        default:
            break;
    }

    /* Done with the message so decrement the ref count */
    result = sae_unRefMsgBuffer(saeContext, buffer);
}

int main(int argc, char **argv)
{
    /* Initialize the SEC */
    adi_sec_Init();

    /* Initialize the SHARC Audio Engine */
    sae_initialize(&saeContext, SAE_CORE_IDX_2, false);

    /* Register an IPC message Rx callback */
    sae_registerMsgReceivedCallback(saeContext, ipcMsgRx, NULL);

    /* Tell the ARM we're ready */
    quickIpcToCore(saeContext, IPC_TYPE_SHARC1_READY, IPC_CORE_ARM);

    while (1) {
        asm("nop;");
    }
}

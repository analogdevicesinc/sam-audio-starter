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
#include <stdint.h>
#include <stdbool.h>

#include "context.h"
#include "clock_domain.h"
#include "usb_audio.h"
#include "wav_audio.h"
#include "sharc_audio.h"
#include "sae.h"

/*
 *  Send audio messages by IPC to both SHARCs in parallel.  Add a
 *  a ref to the message for each SHARC to keep the message from being
 *  deallocated after processing.
 */
static void sendMsg(SAE_CONTEXT *saeContext, SAE_MSG_BUFFER *msg)
{
    SAE_RESULT result;

    sae_refMsgBuffer(saeContext, msg);
    result = sae_sendMsgBuffer(saeContext, msg, IPC_CORE_SHARC0, true);
    if (result != SAE_RESULT_OK) {
        sae_unRefMsgBuffer(saeContext, msg);
    }
    sae_refMsgBuffer(saeContext, msg);
    result = sae_sendMsgBuffer(saeContext, msg, IPC_CORE_SHARC1, true);
    if (result != SAE_RESULT_OK) {
        sae_unRefMsgBuffer(saeContext, msg);
    }
}

/*
 * This function processes and sends audio messages that are ready in
 * the various clock domains.  'clockSource' is true for audio sources
 * and sinks that drive a clock domain.  'source' is true for clock domain
 * sources and false for clock domain sinks.
 *
 * Audio sources/sinks that don't have an inherent clock are executed
 * when their associated clock source/sink executes.
 *
 * When all source/sinks associated with a clock domain have executed, a
 * message is sent to the SHARCs to route that clock domain audio.
 *
 */
void sharcAudio(APP_CONTEXT *context, unsigned mask, SAE_MSG_BUFFER *msg,
    bool clockSource, bool source)
{
    SAE_CONTEXT *sae = context->saeContext;
    CLOCK_DOMAIN cd;
    IPC_MSG *ipcMsg;
    bool ready;

    /*
     * Only audio sources/sinks with inherent clocks call this function so
     * always update the clock domain and send the associated message.
     */
    cd = clock_domain_get(context, mask);
    clock_domain_set_active(context, cd, mask);
    ipcMsg = sae_getMsgBufferPayload(msg);
    ipcMsg->audio.clockDomain = cd;
    sendMsg(context->saeContext, msg);

    /*
     * Process clock-less sinks when the source clock domain executes and clock-less
     * sources when sink clock domain executes.
     */
    if (clockSource) {
        if (source) {
            msg = xferUsbTxAudio(context, context->usbMsgTx[0], cd);
            if (msg) {
                sendMsg(sae, msg);
            }
            msg = xferWavSinkAudio(context, context->wavMsgSink[0], cd);
            if (msg) {
                sendMsg(sae, msg);
            }
        } else {
            msg = xferUsbRxAudio(context, context->usbMsgRx[0], cd);
            if (msg) {
                sendMsg(sae, msg);
            }
            msg = xferWavSrcAudio(context, context->wavMsgSrc[0], cd);
            if (msg) {
                sendMsg(sae, msg);
            }
        }
    }

    /*
     * True when all sources/sinks in a clock domain are ready.  Send a
     * new message to the SHARCs.  Must be a new one because readiness of
     * other domains may overlap processing this one.
     */
    ready = clock_domain_ready(context, cd);
    if (ready) {
        msg = sae_createMsgBuffer(sae, sizeof(*ipcMsg), (void **)&ipcMsg);
        ipcMsg->type = IPC_TYPE_PROCESS_AUDIO;
        ipcMsg->process.clockDomain = cd;
        sendMsg(sae, msg);
        sae_unRefMsgBuffer(sae, msg);
    }
}


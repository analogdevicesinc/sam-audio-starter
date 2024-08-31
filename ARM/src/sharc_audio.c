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
#include "sharc_audio.h"
#include "sae.h"

/*
 *  Send audio messages by IPC to the SHARC.  Add a ref to the message
 *  to keep the message from being deallocated on the SHARC after
 *  processing.
 */
void sendMsg(SAE_CONTEXT *saeContext, SAE_MSG_BUFFER *msg, int core)
{
    SAE_RESULT result;

    sae_refMsgBuffer(saeContext, msg);
    result = sae_sendMsgBuffer(saeContext, msg, core, true);
    if (result != SAE_RESULT_OK) {
        sae_unRefMsgBuffer(saeContext, msg);
    }
}

void *xferSharcAudio(APP_CONTEXT *context, CLOCK_DOMAIN cd, uint32_t cdMask,
    int core, SAE_MSG_BUFFER *sharcMsg[], void *sharcAudio[], int *pp)
{
    CLOCK_DOMAIN myCd;
    SAE_MSG_BUFFER *msg;
    IPC_MSG *ipcMsg;
    void *audio;

    myCd = clock_domain_get(context, cdMask);
    if (myCd != cd) {
        return(NULL);
    }
    clock_domain_set_active(context, myCd, cdMask);

    audio = sharcAudio[*pp];

    *pp = *pp ? 0 : 1;

    msg = sharcMsg[*pp];
    ipcMsg = sae_getMsgBufferPayload(msg);
    ipcMsg->audio.clockDomain = myCd;
    sendMsg(context->saeContext, msg, core);

    return(audio);
}

void *xferSharc0InAudio(APP_CONTEXT *context, CLOCK_DOMAIN cd)
{
    static int pp = 0;
    void *audio;

    audio = xferSharcAudio(
        context, cd, CLOCK_DOMAIN_BITM_SHARC0_IN, IPC_CORE_SHARC0,
        context->sharc0MsgIn, context->sharc0AudioIn, &pp
    );

    return(audio);
}

void *xferSharc0OutAudio(APP_CONTEXT *context, CLOCK_DOMAIN cd)
{
    static int pp = 0;
    void *audio;

    audio = xferSharcAudio(
        context, cd, CLOCK_DOMAIN_BITM_SHARC0_OUT, IPC_CORE_SHARC0,
        context->sharc0MsgOut, context->sharc0AudioOut, &pp
    );

    return(audio);
}


void *xferSharc1InAudio(APP_CONTEXT *context, CLOCK_DOMAIN cd)
{
    static int pp = 0;
    void *audio;

    audio = xferSharcAudio(
        context, cd, CLOCK_DOMAIN_BITM_SHARC1_IN, IPC_CORE_SHARC1,
        context->sharc1MsgIn, context->sharc1AudioIn, &pp
    );

    return(audio);
}

void *xferSharc1OutAudio(APP_CONTEXT *context, CLOCK_DOMAIN cd)
{
    static int pp = 0;
    void *audio;

    audio = xferSharcAudio(
        context, cd, CLOCK_DOMAIN_BITM_SHARC1_OUT, IPC_CORE_SHARC1,
        context->sharc1MsgOut, context->sharc1AudioOut, &pp
    );

    return(audio);
}

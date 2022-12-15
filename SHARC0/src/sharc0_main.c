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
#include <assert.h>

#define DO_CYCLE_COUNTS

/* CCES includes */
#include <services/int/adi_sec.h>
#include <services/gpio/adi_gpio.h>
#include <cycle_count.h>

/* Simple service includes */
#include "sae.h"

/* IPC includes */
#include "ipc.h"

SAE_CONTEXT *saeContext = NULL;
SAE_MSG_BUFFER *cyclesMsg = NULL;

IPC_MSG_ROUTING *routeInfo = NULL;
IPC_MSG_AUDIO *streamInfo[IPC_STREAM_ID_MAX];

#pragma optimize_for_speed
static void routeAudio(uint8_t clockDomain)
{
    ROUTE_INFO *route;
    IPC_MSG_AUDIO *src, *sink, *stream;
    uint8_t channels;
    int32_t *in, *out;
    uint8_t inChannel, outChannel;
    unsigned frame;
    unsigned channel;
    int32_t sample;
    unsigned i;
    cycle_t startCycles;
    cycle_t finalCycles;
    uint8_t attenuationShift;

    /* Toggle LED 11 for measurement */
    adi_gpio_Toggle(ADI_GPIO_PORT_D, ADI_GPIO_PIN_2);

    if (routeInfo == NULL) {
        return;
    }

    START_CYCLE_COUNT(startCycles);

    /* Run all routes associated with this clock domain */
    for (i = 0; i < routeInfo->numRoutes; i++) {

        route = &routeInfo->routes[i];

        if (route->srcID == IPC_STREAMID_UNKNOWN) {
            continue;
        }
        if (route->sinkID == IPC_STREAMID_UNKNOWN) {
            continue;
        }

        src = streamInfo[route->srcID];
        sink = streamInfo[route->sinkID];

        if ((src == NULL) || (sink == NULL)) {
            continue;
        }

        if (src->clockDomain != clockDomain) {
            continue;
        }
        if (sink->clockDomain != clockDomain) {
            continue;
        }

#if 1
        if (src->numFrames != sink->numFrames) {
            continue;
        }
        if (src->wordSize != sink->wordSize) {
            continue;
        }
        if (src->wordSize != sizeof(int32_t)) {
            continue;
        }
        if (route->srcOffset >= src->numChannels) {
            continue;
        }
        if (route->sinkOffset >= sink->numChannels) {
            continue;
        }
#endif

        inChannel = route->srcOffset;
        outChannel = route->sinkOffset;

        channels = route->channels;
        in = src->data + inChannel;
        out = sink->data + outChannel;

        attenuationShift = route->attenuation / 6;

        for (frame = 0; frame < src->numFrames; frame++) {
            for (channel = 0; channel < channels; channel++) {
                if ((outChannel + channel) < sink->numChannels) {
                    if ((inChannel + channel) < src->numChannels) {
                        sample = *(in + channel);
                    } else {
                        sample = 0;
                    }
                    *(out + channel) = sample >> attenuationShift;
                }
            }
            in += src->numChannels;
            out += sink->numChannels;
        }

    }

    /* Invalidate all streams associated with this clock domain */
    for (i = 0; i < IPC_STREAM_ID_MAX; i++) {
        stream = streamInfo[i];
        if (stream && (stream->clockDomain == clockDomain)) {
            streamInfo[i] = NULL;
        }
    }

    STOP_CYCLE_COUNT(finalCycles, startCycles);

    if (clockDomain < IPC_CYCLE_DOMAIN_MAX) {
        IPC_MSG *msg = sae_getMsgBufferPayload(cyclesMsg);
        msg->cycles.cycles[clockDomain] = finalCycles;
    }

}

/*
 * All audio SPORT interrupts (CODEC, SPDIF, A2B) have been hardware aligned
 * at startup by gating their respective bit clocks until all
 * SPORTs have been configured then turning on all clocks at once.  The
 * SPORTs count down exactly 1 frame of bit clocks before starting. This
 * is initiated on the ARM side in init.c -> enable_sport_mclk()
 *
 * The USB RX/TX, WAV src/sink, and RTP sink piggy-back off of their
 * associated clock domain to function like time aligned SPORTs.
 *
 */
static void newAudio(IPC_MSG_AUDIO *audio)
{
    bool clear = false;
    bool unknown = false;

    switch (audio->streamID) {
        case IPC_STREAMID_CODEC_IN:
            break;
        case IPC_STREAMID_CODEC_OUT:
            clear = true;
            break;
        case IPC_STREAMID_SPDIF_IN:
            break;
        case IPC_STREAMID_SPDIF_OUT:
            clear = true;
            break;
        case IPC_STREAMID_A2B_IN:
            break;
        case IPC_STREAMID_A2B_OUT:
            clear = true;
            break;
        case IPC_STREAMID_USB_RX:
            break;
        case IPC_STREAMID_USB_TX:
            clear = true;
            break;
        case IPC_STREAM_ID_WAVE_SRC:
            break;
        case IPC_STREAM_ID_WAVE_SINK:
            clear = true;
            break;
        case IPC_STREAM_ID_RTP_IN:
            break;
        case IPC_STREAM_ID_RTP_OUT:
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

static void ipcMsgRx(SAE_CONTEXT *saeContext, SAE_MSG_BUFFER *buffer,
    void *payload, void *usrPtr)
{
    SAE_MSG_BUFFER *ipcBuffer;
    SAE_RESULT result;
    IPC_MSG *msg = (IPC_MSG *)payload;
    IPC_MSG_AUDIO *audio;
    IPC_MSG *replyMsg;
    IPC_MSG_PROCESS_AUDIO *process;

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
        case IPC_TYPE_AUDIO:
            audio = (IPC_MSG_AUDIO *)&msg->audio;
            newAudio(audio);
            break;
        case IPC_TYPE_AUDIO_ROUTING:
            routeInfo = (IPC_MSG_ROUTING *)&msg->routes;
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
        case IPC_TYPE_PROCESS_AUDIO:
            process = (IPC_MSG_PROCESS_AUDIO *)&msg->process;
            routeAudio(process->clockDomain);
            break;
        default:
            break;
    }

    /* Done with the message so decrement the ref count */
    result = sae_unRefMsgBuffer(saeContext, buffer);
}

int main(int argc, char **argv)
{
    static uint8_t gpioMemory[ADI_GPIO_CALLBACK_MEM_SIZE];
    uint32_t numCallbacks;
    IPC_MSG *msg;

    /* Initialize the SEC */
    adi_sec_Init();

    /* Initialize GPIO */
    adi_gpio_Init(gpioMemory, sizeof(gpioMemory), &numCallbacks);

    /* Initialize the SHARC Audio Engine */
    sae_initialize(&saeContext, SAE_CORE_IDX_1, false);

    /* Create a persistent message for cycle counts */
    cyclesMsg = sae_createMsgBuffer(saeContext, sizeof(*msg), (void **)&msg);
    msg->type = IPC_TYPE_CYCLES;
    msg->cycles.core = IPC_CORE_SHARC0;
    msg->cycles.max = IPC_CYCLE_DOMAIN_MAX;

    /* Register an IPC message Rx callback */
    sae_registerMsgReceivedCallback(saeContext, ipcMsgRx, NULL);

    while (1) {
        asm("nop;");
    }
}

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

/***********************************************************************
 * optimized for speed
 **********************************************************************/
#pragma optimize_for_speed

static void routeAudio(uint8_t clockDomain)
{
    ROUTE_INFO *route;
    IPC_MSG_AUDIO *src, *sink, *stream;
    uint8_t channels;
    int32_t *in32, *out32;
    int16_t *in16, *out16;
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
        if ( (src->wordSize != sizeof(int32_t)) &&
             (src->wordSize != sizeof(int16_t)) ) {
            continue;
        }
        if ( (sink->wordSize != sizeof(int32_t)) &&
             (sink->wordSize != sizeof(int16_t)) ) {
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
        in32 = src->data + inChannel; in16 = (int16_t *)src->data + inChannel;
        out32 = sink->data + outChannel; out16 = (int16_t *)sink->data + outChannel;

        attenuationShift = route->attenuation / 6;

        for (frame = 0; frame < src->numFrames; frame++) {
            for (channel = 0; channel < channels; channel++) {
                if ((outChannel + channel) < sink->numChannels) {
                    if ((inChannel + channel) < src->numChannels) {
                        if (src->wordSize == sizeof(int32_t)) {
                            sample = *(in32 + channel);
                        } else {
                            sample = *(in16 + channel) << 16;
                        }
                    } else {
                        sample = 0;
                    }
                    sample >>= attenuationShift;
                    if (route->mix) {
                        if (sink->wordSize == sizeof(int32_t)) {
                            *(out32 + channel) += sample;
                        } else {
                            *(out16 + channel) += sample >> 16;
                        }
                    } else {
                        if (sink->wordSize == sizeof(int32_t)) {
                            *(out32 + channel) = sample;
                        } else {
                            *(out16 + channel) = sample >> 16;
                        }
                    }
                }
            }
            in32 += src->numChannels; in16 += src->numChannels;
            out32 += sink->numChannels; out16 += sink->numChannels;
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

    if (cyclesMsg &&(clockDomain < IPC_CYCLE_DOMAIN_MAX)) {
        IPC_MSG *msg = sae_getMsgBufferPayload(cyclesMsg);
        msg->cycles.cycles[clockDomain] = finalCycles;
    }

}

#pragma optimize_as_cmd_line
/***********************************************************************
 * end optimized for speed
 **********************************************************************/

/*
 * All audio SPORT interrupts (CODEC, SPDIF, A2B) have been hardware aligned
 * at startup by gating their respective bit clocks until all
 * SPORTs have been configured then turning on all clocks at once.  The
 * SPORTs count down exactly 1 frame of bit clocks before starting. This
 * is initiated on the ARM side in init.c -> enable_sport_mclk()
 *
 * The USB RX/TX and WAV src/sinkpiggy-back off of their
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
        case IPC_STREAM_ID_WAV_SRC:
            break;
        case IPC_STREAM_ID_WAV_SINK:
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
    IPC_MSG_AUDIO *audio;
    IPC_MSG *replyMsg;
    IPC_MSG_PROCESS_AUDIO *process;

    /* Process the message */
    switch (msg->type) {
        case IPC_TYPE_PING:
            ipcBuffer = sae_createMsgBuffer(saeContext, sizeof(*replyMsg), (void **)&replyMsg);
            if (ipcBuffer) {
                replyMsg->type = IPC_TYPE_PING;
                result = sae_sendMsgBuffer(saeContext, ipcBuffer, IPC_CORE_ARM, true);
                if (result != SAE_RESULT_OK) {
                    sae_unRefMsgBuffer(saeContext, ipcBuffer);
                }
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
    if (cyclesMsg) {
        msg->type = IPC_TYPE_CYCLES;
        msg->cycles.core = IPC_CORE_SHARC0;
        msg->cycles.max = IPC_CYCLE_DOMAIN_MAX;
    }

    /* Register an IPC message Rx callback */
    sae_registerMsgReceivedCallback(saeContext, ipcMsgRx, NULL);

    /* Tell the ARM we're ready */
    quickIpcToCore(saeContext, IPC_TYPE_SHARC0_READY, IPC_CORE_ARM);

    while (1) {
        asm("nop;");
    }
}

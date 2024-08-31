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

#if defined(__ADSPARM__)
#include <runtime/cache/adi_cache.h>
#else
#include <sys/cache.h>
#endif

#include "context.h"
#include "clock_domain.h"
#include "process_audio.h"
#include "wav_audio.h"
#include "rtp_audio.h"
#include "vban_audio.h"
#include "vu_audio.h"
#include "usb_audio.h"
#include "sharc_audio.h"
#include "route.h"
#include "gpio_pins.h"

static STREAM_INFO STREAMS[STREAM_ID_MAX];

static SYSTEM_AUDIO_TYPE wavSrcBuffer[WAV_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE wavSinkBuffer[WAV_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE rtpRxBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE rtpTxBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE vbanRxBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE vbanTxBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];

/* Routes audio between sources and sinks */
static void routeAudio(CLOCK_DOMAIN clockDomain,
    STREAM_INFO *streamInfo, unsigned numStreams,
    ROUTE_INFO *routeInfo, unsigned numRoutes)
{
    ROUTE_INFO *route;
    STREAM_INFO *src, *sink, *stream;
    unsigned channels;
    int32_t *in32, *out32;
    int16_t *in16, *out16;
    unsigned inChannel, outChannel;
    unsigned frame;
    unsigned channel;
    SYSTEM_AUDIO_TYPE sample;
    unsigned i;
    unsigned attenuationShift;
    unsigned size;

    /* Run all routes associated with this clock domain */
    for (i = 0; i < numRoutes; i++) {

        route = &routeInfo[i];

        if (route->srcID == STREAM_ID_UNKNOWN) {
            continue;
        }
        if (route->sinkID == STREAM_ID_UNKNOWN) {
            continue;
        }

        src = &streamInfo[route->srcID];
        sink = &streamInfo[route->sinkID];

        if ((src->data == NULL) || (sink->data == NULL)) {
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
        in32 = (int32_t *)src->data + inChannel; in16 = (int16_t *)src->data + inChannel;
        out32 = (int32_t *)sink->data + outChannel; out16 = (int16_t *)sink->data + outChannel;

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

    /* Invalidate all active streams associated with this clock domain */
    for (i = 0; i < STREAM_ID_MAX; i++) {
        stream = &streamInfo[i];
        if ( (stream->streamID != STREAM_ID_UNKNOWN) &&
             (stream->clockDomain == clockDomain) ) {
            if (stream->flush) {
                size = stream->numChannels * stream->numFrames * stream->wordSize;
                flush_data_buffer(stream->data, (char *)stream->data + size, 0);
            }
            stream->streamID = STREAM_ID_UNKNOWN;
            stream->data = NULL;
        }
    }
}

static void inline setStreamInfo(STREAM_ID streamID,
    unsigned numChannels, unsigned numFrames, unsigned wordSize, CLOCK_DOMAIN cd,
    void *data, bool flush)
{
    STREAM_INFO *streamInfo;
    streamInfo = STREAMS + streamID;
    streamInfo->streamID = streamID;
    streamInfo->numChannels = numChannels;
    streamInfo->numFrames = numFrames;
    streamInfo->wordSize = wordSize;
    streamInfo->clockDomain = cd;
    streamInfo->data = data;
    streamInfo->flush = flush;
}

/*
 * This function processes audio that is ready in the various clock domains.
 * 'clockSource' is true for audio sources and sinks that drive a clock
 * domain.  'source' is true for clock domain sources and false for
 * clock domain sinks.
 *
 * Audio sources/sinks that don't have an inherent clock are executed
 * when their associated clock source/sink executes.
 *
 * When all source/sinks associated with a clock domain is ready, that
 * clock domain is processed.
 *
 */
void processAudio(APP_CONTEXT *context, unsigned mask, STREAM_ID streamID,
    unsigned numChannels, unsigned numFrames, unsigned wordSize,
    void *data, bool flush,
    bool clockSource, bool source)
{
    CLOCK_DOMAIN cd;
    bool ready;

    /*
     * Only audio sources/sinks with inherent clocks call this function so
     * always update the clock domain.
     */
    cd = clock_domain_get(context, mask);
    clock_domain_set_active(context, cd, mask);

    /* Store audio info */
    setStreamInfo(streamID, numChannels, numFrames, wordSize, cd, data, flush);

    /*
     * Process clock-less sources and sinks.
     */
    if (clockSource) {
        if (source) {
            ready = xferWavSrcAudio(context, wavSrcBuffer, cd, &numChannels);
            if (ready) {
                setStreamInfo(
                    STREAM_ID_WAV_SRC, numChannels,
                    SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
                    cd, wavSrcBuffer, false
                );
            }
            ready = xferRtpRxAudio(context, rtpRxBuffer, cd, &numChannels);
            if (ready) {
                setStreamInfo(
                    STREAM_ID_RTP_RX, numChannels,
                    SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
                    cd, rtpRxBuffer, false
                );
            }
            ready = xferVbanRxAudio(context, vbanRxBuffer, cd, &numChannels);
            if (ready) {
                setStreamInfo(
                    STREAM_ID_VBAN_RX, numChannels,
                    SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
                    cd, vbanRxBuffer, false
                );
            }
            ready = xferUsbRxAudio(context, &data, cd);
            if (ready) {
                setStreamInfo(
                    STREAM_ID_USB_RX, context->cfg.usbOutChannels,
                    SYSTEM_BLOCK_SIZE, context->cfg.usbWordSize,
                    cd, data, false
                );
            }
#ifdef SHARC_AUDIO_ENABLE
            data = xferSharc0OutAudio(context, cd);
            if (data) {
                setStreamInfo(
                    STREAM_ID_SHARC0_OUT, SHARC0_AUDIO_OUT_CHANNELS,
                    SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
                    cd, data, false
                );
            }
            data = xferSharc1OutAudio(context, cd);
            if (data) {
                setStreamInfo(
                    STREAM_ID_SHARC1_OUT, SHARC1_AUDIO_OUT_CHANNELS,
                    SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
                    cd, data, false
                );
            }
#endif
        } else {
            ready = xferWavSinkAudio(context, wavSinkBuffer, cd, &numChannels);
            if (ready) {
                setStreamInfo(
                    STREAM_ID_WAV_SINK, numChannels,
                    SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
                    cd, wavSinkBuffer, false
                );
            }
            ready = xferRtpTxAudio(context, rtpTxBuffer, cd, &numChannels);
            if (ready) {
                setStreamInfo(
                    STREAM_ID_RTP_TX, numChannels,
                    SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
                    cd, rtpTxBuffer, false
                );
            }
            ready = xferVbanTxAudio(context, vbanTxBuffer, cd, &numChannels);
            if (ready) {
                setStreamInfo(
                    STREAM_ID_VBAN_TX, numChannels,
                    SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
                    cd, vbanTxBuffer, false
                );
            }
            ready = xferUsbTxAudio(context, &data, cd);
            if (ready) {
                setStreamInfo(
                    STREAM_ID_USB_TX, context->cfg.usbInChannels,
                    SYSTEM_BLOCK_SIZE, context->cfg.usbWordSize,
                    cd, data, false
                );
            }
            ready = xferVUSinkAudio(context, &data, cd);
            if (ready) {
                setStreamInfo(
                    STREAM_ID_VU_IN, VU_MAX_CHANNELS,
                    SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
                    cd, data, false
                );
            }
#ifdef SHARC_AUDIO_ENABLE
            data = xferSharc0InAudio(context, cd);
            if (data) {
                setStreamInfo(
                    STREAM_ID_SHARC0_IN, SHARC0_AUDIO_IN_CHANNELS,
                    SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
                    cd, data, false
                );
            }
            data = xferSharc1InAudio(context, cd);
            if (data) {
                setStreamInfo(
                    STREAM_ID_SHARC1_IN, SHARC1_AUDIO_IN_CHANNELS,
                    SYSTEM_BLOCK_SIZE, sizeof(SYSTEM_AUDIO_TYPE),
                    cd, data, false
                );
            }
#endif
        }
    }

    /*
     * Process audio when all sources/sinks in a clock domain are ready.
     */
    ready = clock_domain_ready(context, cd);
    if (ready) {
#ifdef SHARC_AUDIO_ENABLE
        SAE_CONTEXT *sae = context->saeContext;
        SAE_MSG_BUFFER *msg;
        IPC_MSG *ipcMsg;
        msg = sae_createMsgBuffer(sae, sizeof(*ipcMsg), (void **)&ipcMsg);
        if (msg) {
           ipcMsg->type = IPC_TYPE_PROCESS_AUDIO;
           ipcMsg->process.clockDomain = cd;
           sendMsg(sae, msg, IPC_CORE_SHARC0);
           sendMsg(sae, msg, IPC_CORE_SHARC1);
           sae_unRefMsgBuffer(sae, msg);
        }
#endif
        routeAudio(cd,
            STREAMS, STREAM_ID_MAX,
            context->routingTable, MAX_AUDIO_ROUTES
        );
    }
}

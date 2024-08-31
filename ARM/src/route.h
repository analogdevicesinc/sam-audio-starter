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

#ifndef _route_h
#define _route_h

#include "clock_domain_defs.h"

/*
 * Audio stream identifiers
 */
typedef enum _STREAM_ID {
    STREAM_ID_UNKNOWN = 0,
    STREAM_ID_CODEC_IN,
    STREAM_ID_CODEC_OUT,
    STREAM_ID_SPDIF_IN,
    STREAM_ID_SPDIF_OUT,
    STREAM_ID_A2B_IN,
    STREAM_ID_A2B_OUT,
    STREAM_ID_USB_RX,
    STREAM_ID_USB_TX,
    STREAM_ID_WAV_SRC,
    STREAM_ID_WAV_SINK,
    STREAM_ID_RTP_RX,
    STREAM_ID_RTP_TX,
    STREAM_ID_VBAN_RX,
    STREAM_ID_VBAN_TX,
    STREAM_ID_SHARC0_IN,
    STREAM_ID_SHARC0_OUT,
    STREAM_ID_SHARC1_IN,
    STREAM_ID_SHARC1_OUT,
    STREAM_ID_VU_IN,
    STREAM_ID_A2B2_IN,
    STREAM_ID_A2B2_OUT,
    STREAM_ID_MAX
} STREAM_ID;

typedef struct _STREAM_INFO {
    STREAM_ID streamID;
    unsigned numChannels;
    unsigned numFrames;
    unsigned  wordSize;
    CLOCK_DOMAIN clockDomain;
    bool flush;
    void *data;
} STREAM_INFO;

typedef struct _ROUTE_INFO {
    STREAM_ID srcID;
    STREAM_ID sinkID;
    unsigned srcOffset;
    unsigned sinkOffset;
    unsigned channels;
    unsigned attenuation;
    unsigned mix;
} ROUTE_INFO;

#endif

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

#ifndef _process_audio_h
#define _process_audio_h

#include <stdint.h>

#include "context.h"
#include "route.h"

void processAudio(APP_CONTEXT *context, unsigned mask, STREAM_ID streamID,
    unsigned numChannels, unsigned numFrames, unsigned wordSize,
    void *data, bool flush,
    bool clockSource, bool source);

#endif

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

#ifndef _wav_audio_h
#define _wav_audio_h

#include "context.h"
#include "wav_file.h"
#include "ipc.h"

void wav_audio_init(APP_CONTEXT *context);

SAE_MSG_BUFFER *xferWavSinkAudio(APP_CONTEXT *context, SAE_MSG_BUFFER *msg,
    CLOCK_DOMAIN cd);

SAE_MSG_BUFFER *xferWavSrcAudio(APP_CONTEXT *context, SAE_MSG_BUFFER *msg,
    CLOCK_DOMAIN cd);

#endif

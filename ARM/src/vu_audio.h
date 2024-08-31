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

#ifndef _vu_audio_h
#define _vu_audio_h

#include "context.h"

void vu_audio_init(APP_CONTEXT *context);

int xferVUSinkAudio(APP_CONTEXT *context, void **audio, CLOCK_DOMAIN cd);
unsigned getVU(APP_CONTEXT *context, SYSTEM_AUDIO_TYPE *vu, unsigned channels);

#endif

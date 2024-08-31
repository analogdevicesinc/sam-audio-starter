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
#ifndef _sharc_audio_h
#define _sharc_audio_h

#include <stdint.h>

#include "context.h"
#include "sae.h"

void sendMsg(SAE_CONTEXT *saeContext, SAE_MSG_BUFFER *msg, int core);

void *xferSharc0InAudio(APP_CONTEXT *context, CLOCK_DOMAIN cd);
void *xferSharc0OutAudio(APP_CONTEXT *context, CLOCK_DOMAIN cd);

void *xferSharc1InAudio(APP_CONTEXT *context, CLOCK_DOMAIN cd);
void *xferSharc1OutAudio(APP_CONTEXT *context, CLOCK_DOMAIN cd);

#endif

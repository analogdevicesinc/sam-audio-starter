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

#ifndef _uac2_audio_h
#define _uac2_audio_h

#include <stdint.h>
#include <stdbool.h>

#include "context.h"
#include "uac2_soundcard.h"

uint16_t uac2Rx(void *data, void **nextData,
    uint16_t rxSize, void *usrPtr);

uint16_t uac2Tx(void *data, void **nextData,
    uint16_t minSize, uint16_t maxSize, void *usrPtr);

uint32_t uac2RateFeedback(void *usrPtr);

void uac2EndpointEnabled(UAC2_DIR dir, bool enable, void *usrPtr);

int xferUsbRxAudio(APP_CONTEXT *context, void **audio, CLOCK_DOMAIN cd);

int xferUsbTxAudio(APP_CONTEXT *context, void **audio, CLOCK_DOMAIN cd);

#endif


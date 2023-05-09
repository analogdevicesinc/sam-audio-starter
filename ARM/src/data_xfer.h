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

#ifndef _data_xfer_h
#define _data_xfer_h

#include "context.h"
#include "data_file.h"
#include "ipc.h"

void data_file_init(APP_CONTEXT *context);
static void data_stream_write(size_t rsize, bool eof, PaUtilRingBuffer *rbuf,
                              void *buf, ring_buffer_size_t samplesIn);

static bool data_file_write(DATA_FILE *fileSink, SYSTEM_AUDIO_TYPE *samples, 
                            ring_buffer_size_t samplesOut);

SAE_MSG_BUFFER *xferFileSinkData(APP_CONTEXT *context, SAE_MSG_BUFFER *msg,
    CLOCK_DOMAIN cd);

SAE_MSG_BUFFER *xferFileSrcData(APP_CONTEXT *context, SAE_MSG_BUFFER *msg,
    CLOCK_DOMAIN cd);

bool okToStream(void);

#endif

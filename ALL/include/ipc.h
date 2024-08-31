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

#ifndef _ipc_h
#define _ipc_h

#include <stdint.h>

#include "sae.h"

/*
 * IPC core identifiers
 */
#define IPC_CORE_ARM         SAE_CORE_IDX_0
#define IPC_CORE_SHARC0      SAE_CORE_IDX_1
#define IPC_CORE_SHARC1      SAE_CORE_IDX_2

/*
 * Max number of clock domains to track cycles on
 */
#define IPC_CYCLE_DOMAIN_MAX 4

/*
 * IPC message types
 */
enum IPC_TYPE {
    IPC_TYPE_UNKNOWN = 0,
    IPC_TYPE_PING,
    IPC_TYPE_AUDIO,
    IPC_TYPE_SHARC0_READY,
    IPC_TYPE_SHARC1_READY,
    IPC_TYPE_PROCESS_AUDIO,
    IPC_TYPE_CYCLES,
};

/*
 * Audio stream identifiers (IPC_TYPE_AUDIO messages).  IN and OUT are
 * as viewed from the ARM core.
 */
enum IPC_STREAMID {
    IPC_STREAMID_UNKNOWN = 0,
    IPC_STREAMID_SHARC0_IN,
    IPC_STREAMID_SHARC0_OUT,
    IPC_STREAMID_SHARC1_IN,
    IPC_STREAMID_SHARC1_OUT,
    IPC_STREAM_ID_MAX
};

/*
 * Streaming audio data message (IPC_TYPE_AUDIO messages)
 */
#pragma pack(1)
typedef struct _IPC_MSG_AUDIO {
    uint8_t streamID;
    uint8_t numChannels;
    uint8_t numFrames;
    uint8_t wordSize;
    uint8_t clockDomain;
    uint8_t reserved[3];
    int32_t data[];
} IPC_MSG_AUDIO;
#pragma pack()

/*
 * CPU cycles (IPC_TYPE_CYCLES messages)
 */
#pragma pack(1)
typedef struct _IPC_MSG_CYCLES {
    uint8_t core;
    uint8_t max;
    uint8_t reserved[2];
    uint32_t cycles[IPC_CYCLE_DOMAIN_MAX];
} IPC_MSG_CYCLES;
#pragma pack()

/*
 * Process (IPC_TYPE_PROCESS_AUDIO messages)
 */
#pragma pack(1)
typedef struct _IPC_MSG_PROCESS_AUDIO {
    uint8_t clockDomain;
    uint8_t reserved[3];
} IPC_MSG_PROCESS_AUDIO;
#pragma pack()

/*
 * Generic message.  Query type to determine which union'd payload to
 * use.
 */
#pragma pack(1)
typedef struct _IPC_MSG {
    uint8_t type;
    uint8_t reserved[3];
    union {
        IPC_MSG_AUDIO audio;
        IPC_MSG_CYCLES cycles;
        IPC_MSG_PROCESS_AUDIO process;
    };
} IPC_MSG;
#pragma pack()

#endif

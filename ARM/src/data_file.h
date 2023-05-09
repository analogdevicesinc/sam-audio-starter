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

#ifndef _data_file_h
#define _data_file_h

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"

typedef enum DATA_FMT {
    DATA_FMT_UNKNOWN = 0,
    DATA_FMT_SIGNED_32BIT_LE,
    DATA_FMT_SIGNED_16BIT_LE
} DATA_FMT;

/*
*  FILE transfer states
*/
typedef enum FILE_STREAM_STATES {
    FILE_STREAM_IDLE = 0,
    FILE_STREAM_START,
    FILE_STREAM_ACTIVE,
    FILE_STREAM_PAUSED,
    FILE_STREAM_EOF,
    FILE_STREAM_STOP
} FILE_STREAM_STATES;

//#define FILE_READ_BUF_SIZE  (SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE)


/* 
*  FILE streaming sync words (sent at beginning and end of file stream)
*/
#define STREAM_SYNC_WORD_COUNT    2
#define STREAM_START_SYNC_1       0x6014fca2
#define STREAM_START_SYNC_2       0xdfc24e50
#define STREAM_RESTART_SYNC_1     0x71250db3
#define STREAM_RESTART_SYNC_2     0xceb13d4f
#define STREAM_PAUSE_SYNC_1       0xc39eb6c3
#define STREAM_PAUSE_SYNC_2       0x53b44f5f
#define STREAM_STOP_SYNC_1        0x3014fca4
#define STREAM_STOP_SYNC_2        0xafc24e50

typedef struct DATA_FILE {
    char *fname;                /* file name */
    FILE *f;                    /* file ptr */
    bool enabled;               /* file stream is enabled */
    bool sync;                  /* sync flag used to signal start of file  */
    bool eof;                   /* file read reached end of file */
    bool loop;                  /* flag to loop the file */
    FILE_STREAM_STATES state;   /* file streaming state */
    SemaphoreHandle_t lock;     /* task semaphore for locking */
    size_t fileSizeBytes;       /* file size in bytes */
    size_t byteCount;           /* current number of bytes in file (bytes read or written */
    unsigned channels;          /* channels (slots) */
    unsigned sampleRate;        /* A2B audio frame rate */
    unsigned frameSizeBytes;    /* bytes per frame = channels*wordSizeBytes */
    unsigned wordSizeBytes;     /* 1 = read bytes */
    bool isSrc;                 /* source or sink indicator */
    void *fileBuf;              /* pointer to file buffer */
    size_t dataOffset;          /* current file byte offset (from start of data) */ 
    void *usrData[2];
    unsigned temp;
} DATA_FILE;

bool openData(DATA_FILE *df);
void closeData(DATA_FILE *df);
size_t readData(DATA_FILE *df, void *buf, size_t samples);
size_t writeData(DATA_FILE *df, void *buf, size_t samples);
void overrideData(DATA_FILE *df, unsigned channels);

#endif

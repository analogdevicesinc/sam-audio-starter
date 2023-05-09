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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "umm_malloc.h"

#include "data_file.h"
#include "context.h"

#ifndef DATA_FILE_BUF_SIZE
#define DATA_FILE_BUF_SIZE (16 * 1024)
#endif

/***********************************************************************
 * Data File helper functions, typedefs and defines
 * 
 * Note: borrowed from wav file routines...
 **********************************************************************/

typedef enum DATA_ENDIAN {
    DATA_ENDIAN_BE = 0,
    DATA_ENDIAN_LE
} DATA_ENDIAN;

static uint16_t fix_uint16(uint16_t val, DATA_ENDIAN endian)
{
    if (endian == DATA_ENDIAN_LE) {
        return(val);
    }
    return (val << 8) | (val >> 8 );
}

static uint32_t fix_uint32(uint32_t val, DATA_ENDIAN endian)
{
    if (endian == DATA_ENDIAN_LE) {
        return(val);
    }
    val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
    return (val << 16) | (val >> 16);
}

static bool writeDataFileHeader(DATA_FILE *df)
{
    FILE *f = df->f;

    fseek(f, 0, SEEK_SET);

    return(true);
}

/* TODO: determine which override features to support */
void overrideData(DATA_FILE *df, unsigned channels)
{
    df->channels = 1;  // channels, force to 1 initially
    df->frameSizeBytes = df->channels * df->wordSizeBytes;
//    df->dataSize -= df->dataSize % df->channels;
}

bool openData(DATA_FILE *df)
{
    bool ok = false;

    df->f = fopen(df->fname, df->isSrc ? "rb" : "wb");
    if (df->f) {
#ifdef DATA_FILE_BUF_SIZE
        df->fileBuf = (char *)umm_calloc(DATA_FILE_BUF_SIZE, 1);
        setvbuf(df->f, df->fileBuf, _IOFBF, DATA_FILE_BUF_SIZE);
#else
        df->fileBuf = NULL;
#endif
/* TODO: determine what parameters to set for file struct */
        if (df->isSrc) {
            /* will be at start of file */
            df->state = FILE_STREAM_START;
            fseek(df->f, 0, SEEK_END);
            df->fileSizeBytes = ftell(df->f);  /* get the size of the file */
            fseek(df->f, 0, SEEK_SET);
            df->dataOffset = 0;
            df->enabled = true;
            df->sync = true;
            df->eof = false;
            df->channels = 1;
            df->wordSizeBytes = 1;        /* read data file as bytes */
            df->sampleRate = SYSTEM_SAMPLE_RATE;
            df->temp = 0;
            df->byteCount = df->fileSizeBytes;
            ok = true;
        } else {
            df->enabled = true;
            df->dataOffset = 0;
            df->fileSizeBytes = 0;
            df->wordSizeBytes = 4;        /* write data file as 32-bit words */
            ok = true;
        }
    }

    return(ok);
}


void closeData(DATA_FILE *df)
{
    if (!df->isSrc) {
    }
    if (df->f) {
        fclose(df->f); df->f = NULL;
    }
    if (df->fileBuf) {
        umm_free(df->fileBuf); df->fileBuf = NULL;
    }
}

size_t readData(DATA_FILE *df, void *buf, size_t numBytes)
{
    size_t sizeBytes;
    size_t rsize;
    size_t bytesRemaining;
    bool ok;
    bool resetData;

    static size_t totalBytes =0;
    
    bytesRemaining = df->fileSizeBytes - df->dataOffset;  /* file size - current location */
    df->temp++;
    sizeBytes = numBytes > bytesRemaining ? bytesRemaining : numBytes;

    resetData = false; ok = true;
    /* read file as bytes */
    rsize = fread(buf, df->wordSizeBytes, numBytes, df->f);
    
    totalBytes += rsize;
    if (rsize != numBytes) {
        if (feof(df->f)) {
            resetData = true;
        } else if (ferror(df->f)) {
            ok = false;
        } else if (rsize <= 0) {
            ok = false;
        }
    } else {
        df->dataOffset += rsize;
        if (df->dataOffset >= df->fileSizeBytes) {
            resetData = true;
        }
    }

/* TODO: the file should be sent, close file */
    if (resetData) {
        if (df->loop) {
            if (resetData) {
                fseek(df->f, 0, SEEK_SET);
                df->dataOffset = 0;
            }
        } else {
            df->eof = true;
            df->enabled = false;
        }
    }

    return(ok ? rsize : -1);
}

size_t writeData(DATA_FILE *df, void *buf, size_t samples)
{
    size_t wsize;
    bool ok;

    ok = true;

    wsize = fwrite(buf, df->wordSizeBytes, samples, df->f);
    if (wsize != samples) {
        ok = false;
    } else {
        df->byteCount += wsize;
        df->fileSizeBytes = df->byteCount;
    }

    return (ok ? wsize : -1);
}

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

#include <stdint.h>
#include <string.h>

#include "util.h"

uint32_t roundUpPow2(uint32_t x)
{
    uint32_t p = 1;
    while (p < x) {
        p *= 2;
    }
    return(p);
}

__attribute__((optimize("O1")))
void copyAndConvert(
    void *src, unsigned srcWordSize, unsigned srcChannels,
    void *dst, unsigned dstWordSize, unsigned dstChannels,
    unsigned frames, bool zero)
{
    uint16_t *s16, *d16;
    uint32_t *s32, *d32;
    unsigned channels;
    unsigned channel;
    unsigned frame;

    channels = srcChannels < dstChannels ? srcChannels : dstChannels;

    if (zero) {
        memset(dst, 0, dstWordSize * dstChannels * frames);
    }

    if ((srcWordSize == sizeof(uint16_t)) && (dstWordSize == sizeof(uint16_t))) {
        s16 = src; d16 = dst;
        for (frame = 0; frame < frames; frame++) {
            for (channel = 0; channel < channels; channel++) {
                *(d16+channel) = *(s16+channel);
            }
            s16 += srcChannels; d16 += dstChannels;
        }
    } else if ((srcWordSize == sizeof(uint32_t)) && (dstWordSize == sizeof(uint32_t))) {
        s32 = src; d32 = dst;
        for (frame = 0; frame < frames; frame++) {
            for (channel = 0; channel < channels; channel++) {
                *(d32+channel) = *(s32+channel);
            }
             s32 += srcChannels; d32 += dstChannels;
        }
    } else if ((srcWordSize == sizeof(uint32_t)) && (dstWordSize == sizeof(uint16_t))) {
        s32 = src; d16 = dst;
        for (frame = 0; frame < frames; frame++) {
            for (channel = 0; channel < channels; channel++) {
                *(d16+channel) = *(s32+channel) >> 16;
            }
            s32 += srcChannels; d16 += dstChannels;
        }
    } else if ((srcWordSize == sizeof(uint16_t)) && (dstWordSize == sizeof(uint32_t))) {
        s16 = src; d32 = dst;
        for (frame = 0; frame < frames; frame++) {
            for (channel = 0; channel < channels; channel++) {
                *(d32+channel) = *(s16+channel) << 16;
            }
            s16 += srcChannels; d32 += dstChannels;
        }
    }
}

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

#ifndef _uac20_descriptors_cfg_h
#define _uac20_descriptors_cfg_h

#include "umm_malloc.h"

/* Allocate descriptors from the cached SDRAM heap and uncached 32-bit aligned memory */
#define UAC20_DESCRIPTORS_HEAP     UMM_SDRAM_HEAP
#define UAC20_L2_DESCRIPTORS_HEAP  UMM_L2_UNCACHED_HEAP

#define UAC20_DESCRIPTORS_MALLOC(x)  \
    umm_malloc_heap(UAC20_DESCRIPTORS_HEAP, x)

#define UAC20_DESCRIPTORS_REALLOC(x, y) \
    umm_realloc_heap(UAC20_DESCRIPTORS_HEAP, x, y)

#define UAC20_DESCRIPTORS_CALLOC(x, y)  \
    umm_calloc_heap(UAC20_DESCRIPTORS_HEAP, x, y)

#define UAC20_DESCRIPTORS_FREE(x)    \
    umm_free_heap(UAC20_DESCRIPTORS_HEAP, x)
    
#define UAC20_L2_DESCRIPTORS_MALLOC_ALIGNED(x)  \
    umm_malloc_heap_aligned(UAC20_L2_DESCRIPTORS_HEAP, x, sizeof(uint32_t))

#define UAC20_L2_DESCRIPTORS_FREE_ALIGNED(x)  \
    umm_free_heap_aligned(UAC20_L2_DESCRIPTORS_HEAP, x)

#endif

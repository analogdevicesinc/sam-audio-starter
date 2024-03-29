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

#ifndef _flash_map_h
#define _flash_map_h

/*
 * WARNING: The upper 4k of the BOOT0 area is reserved for boot
 *          mode information (0x0000F000 - 0x0000FFFF).
 */

/* Physical start of flash */
#define FLASH_ADDR    (0x00000000)

/* Second-stage boot loader (64k reserved) */
#define BOOT0_OFFSET  (FLASH_ADDR)
#define BOOT0_MODE    (0x0000F000)
#define BOOT0_SIZE    (0x00010000)

/* Third stage boot loader (USB re-flasher, 192k reserved) */
#define BOOT1_OFFSET  (BOOT0_OFFSET + BOOT0_SIZE)
#define BOOT1_SIZE    (0x00030000)

/* Application (2M reserved) */
#define APP_OFFSET    (BOOT1_OFFSET + BOOT1_SIZE)
#define APP_SIZE      (0x00200000)

/* SPIFFS Filesystem (8M reserved) */
#define SPIFFS_OFFSET (APP_OFFSET + APP_SIZE)
#define SPIFFS_SIZE   (0x00800000)

/* Erase block size, good for all flash devices */
#define ERASE_BLOCK_SIZE (4*1024)
#define FLASH_PAGE_SIZE  (256)

#endif

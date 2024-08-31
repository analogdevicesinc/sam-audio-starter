/**
 * Copyright (c) 2022 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */
#ifndef _clock_domain_defs_h
#define _clock_domain_defs_h

typedef enum CLOCK_DOMAIN {
    CLOCK_DOMAIN_SYSTEM = 0,
    CLOCK_DOMAIN_A2B,
    CLOCK_DOMAIN_RTP,
    CLOCK_DOMAIN_VBAN,
    CLOCK_DOMAIN_MAX
} CLOCK_DOMAIN;

enum {
    CLOCK_DOMAIN_BITM_CODEC_IN   = 0x00000001u,
    CLOCK_DOMAIN_BITM_CODEC_OUT  = 0x00000002u,
    CLOCK_DOMAIN_BITM_A2B_IN     = 0x00000004u,
    CLOCK_DOMAIN_BITM_A2B_OUT    = 0x00000008u,
    CLOCK_DOMAIN_BITM_USB_RX     = 0x00000010u,
    CLOCK_DOMAIN_BITM_USB_TX     = 0x00000020u,
    CLOCK_DOMAIN_BITM_WAV_SRC    = 0x00000040u,
    CLOCK_DOMAIN_BITM_WAV_SINK   = 0x00000080u,
    CLOCK_DOMAIN_BITM_SPDIF_IN   = 0x00000100u,
    CLOCK_DOMAIN_BITM_SPDIF_OUT  = 0x00000200u,
    CLOCK_DOMAIN_BITM_VU_IN      = 0x00000400u,
    CLOCK_DOMAIN_BITM_SHARC0_IN  = 0x00000800u,
    CLOCK_DOMAIN_BITM_SHARC0_OUT = 0x00001000u,
    CLOCK_DOMAIN_BITM_SHARC1_IN  = 0x00002000u,
    CLOCK_DOMAIN_BITM_SHARC1_OUT = 0x00004000u,
    CLOCK_DOMAIN_BITM_RTP_RX     = 0x00008000u,
    CLOCK_DOMAIN_BITM_RTP_TX     = 0x00010000u,
    CLOCK_DOMAIN_BITM_VBAN_RX    = 0x00020000u,
    CLOCK_DOMAIN_BITM_VBAN_TX    = 0x00040000u,
    CLOCK_DOMAIN_BITM_A2B2_IN    = 0x00080000u,
    CLOCK_DOMAIN_BITM_A2B2_OUT   = 0x00100000u,
};

#endif

/**
 * Copyright (c) 2026 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */
#include <stdint.h>

#include "context.h"
#include "twi_simple.h"

#pragma pack(1)
typedef struct SI5356A_REG_DATA {
   uint8_t addr;
   uint8_t value;
} SI5356A_REG_DATA;
#pragma pack()

SI5356A_REG_DATA SI5356_CLK4_24567Mhz[] = {
    { 230,0x04},
    {  74,0x10},
    {  75,0xC2},
    {  76,0x2A},
    {  77,0x00},
    {  78,0x02},
    {  79,0x00},
    {  80,0x00},
    {  81,0x80},
    {  82,0x01},
    {  83,0x00},
    {  84,0x00},
    { 230,0x00}
};

/* This function sets CLK4/5 of U25 (the main audio @ MCLK DAI0_PIN06) to
 * 24.576MHz instead of it's default speed of 12.288MHz.
 */
void audio_mclk_24576_mhz(APP_CONTEXT *context)
{
    TWI_SIMPLE_RESULT twiResult;
    sTWI *twi;
    uint8_t i;
    uint8_t len;

    /* Fixed clocks on SAM V2 */
    if (context->samVersion >= SAM_VERSION_2) {
        return;
    }

    twi = context->ethClkTwiHandle;

    len = sizeof(SI5356_CLK4_24567Mhz) / sizeof(SI5356A_REG_DATA);

    for (i = 0; i < len; i++) {
        twiResult = twi_write(twi, 0x70,
            (uint8_t *)&SI5356_CLK4_24567Mhz[i], sizeof(SI5356A_REG_DATA));
    }
}

SI5356A_REG_DATA SI5356_ETH_CLK_25Mhz[] = {
    {  0xE6,0x02},
    {  0xF1,0x65},
    {  0x41,0x2A},
    {  0xE6,0x00},
};

/*
 * This function sets the Ethernet clock generator output to 25MHz.
 * HW Version 1.3 and lower defaulted to 50MHz.
 */
void ether_clk_25_mhz(APP_CONTEXT *context)
{
    TWI_SIMPLE_RESULT twiResult;
    sTWI *twi;
    uint8_t i;
    uint8_t len;

    /* Fixed clocks on SAM V2 */
    if (context->samVersion >= SAM_VERSION_2) {
        return;
    }

    twi = context->ethClkTwiHandle;

    len = sizeof(SI5356_ETH_CLK_25Mhz) / sizeof(SI5356A_REG_DATA);

    for (i = 0; i < len; i++) {
        twiResult = twi_write(twi, 0x70,
            (uint8_t *)&SI5356_ETH_CLK_25Mhz[i], sizeof(SI5356A_REG_DATA));
    }
}


/**
 * Copyright (c) 2025 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

/* Standard library includes */
#include <string.h>
#include <limits.h>
#include <stdbool.h>

/* ADI processor includes */
#include <sruSC589.h>

/* Simple driver includes */
#include "sport_simple.h"
#include "pcg_simple.h"

/* Application includes */
#include "context.h"
#include "init.h"
#include "clock_domain.h"
#include "tdm_audio.h"

/* DAI Input Enable Bit definitions (not in any ADI header files) */
#define BITP_PADS0_DAI1_IE_PB03   (3)
#define BITP_PADS0_DAI1_IE_PB04   (4)

#define TDM_ENABLE_MASK \
   ((1 << BITP_PADS0_DAI1_IE_PB03) | (1 << BITP_PADS0_DAI1_IE_PB04))

/***********************************************************************
 * Simple SPORT driver TDM settings
 **********************************************************************/
static SPORT_SIMPLE_CONFIG TDM8x1 = {
    .clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE,
    .fsDir = SPORT_SIMPLE_FS_DIR_SLAVE,
    .dataDir = SPORT_SIMPLE_DATA_DIR_UNKNOWN,  /* Set during init */
    .bitClkOptions = SPORT_SIMPLE_CLK_FALLING,
    .fsOptions = SPORT_SIMPLE_FS_OPTION_EARLY | SPORT_SIMPLE_FS_OPTION_50,
    .tdmSlots = SPORT_SIMPLE_TDM_8,
    .wordSize = SPORT_SIMPLE_WORD_SIZE_16BIT,
    .dataEnable = SPORT_SIMPLE_ENABLE_NONE,    /* Set during init */
    .frames = SYSTEM_BLOCK_SIZE,
    .syncDMA = true
};

/***********************************************************************
 * TDM / SPORT4 / SRU initialization (TDM8 clock slave)
 **********************************************************************/
#if 0
Signal  SAM
TDM8 D0 (RX to Sharc)   DAI1_PIN06 (P5-13)
TDM8 D1 (RX to Sharc)   DAI1_PIN05 (P5-11)
TDM8 D2 (TX from Sharc) DAI1_PIN01 (P5-1)
BCLK    DAI1_PIN03 (P5-5)
SYNC    DAI1_PIN04 (P5-9)
GND P5-7, P5-8
#endif

void tdm_disable(APP_CONTEXT *context)
{
    *pREG_PADS0_DAI1_IE &= ~TDM_ENABLE_MASK;
}

void tdm_enable(APP_CONTEXT *context)
{
    *pREG_PADS0_DAI1_IE |= TDM_ENABLE_MASK;
}

static void sru_config_tdm_cross_clk(void)
{
    /* Configure static PCG C parameters */
    PCG_SIMPLE_CONFIG pcg_c = {
        .pcg = PCG_C,                    // PCG C
        .clk_src = PCG_SRC_DAI_PIN,      // Sourced from DAI
        .clk_in_dai_pin = 3,             // Sourced from DAI pin 3
        .lrclk_clocks_per_frame = 256,   // Not used
        .sync_to_fs = false
    };

    /* Configure a 3.702 MHz I2S BCLK from the 6.144 MHz TDM BCLK */
    pcg_c.bitclk_div = 2;

    /* This sets everything up */
    pcg_open(&pcg_c);
    pcg_enable(PCG_C, true);
}

static void sru_config_tdm_slave(void)
{
    SRU2(LOW,  DAI1_PBEN03_I);        // BCLK (IN)
    SRU2(DAI1_PB03_O, SPT4_ACLK_I);   // BCLK to SPORT4A BCLK
    SRU2(DAI1_PB03_O, SPT4_BCLK_I);   // BCLK to SPORT4B BCLK

    SRU2(LOW,  DAI1_PBEN04_I);        // SYNC (IN)
    SRU2(DAI1_PB04_O, SPT4_AFS_I);    // SYNC to SPORT4A FS
    SRU2(DAI1_PB04_O, SPT4_BFS_I);    // SYNC to SPORT4B FS

    SRU2(LOW,  DAI1_PBEN06_I);        // D0 (IN)
    SRU2(DAI1_PB06_O, SPT4_AD0_I);    // D0 to SPORT4A D0 input

    SRU2(LOW,  DAI1_PBEN05_I);        // D1 (IN)
    SRU2(DAI1_PB05_O, SPT4_AD1_I);    // D1 to SPORT4A D1 input

    SRU2(HIGH, DAI1_PBEN01_I);        // D2 (OUT)
    SRU2(SPT4_BD0_O, DAI1_PB01_I);    // SPORT4B D0 output to D2
}

void tdm_sport_init(APP_CONTEXT *context)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;

    /* SPORT4B: TDM data out */
    sportCfg = TDM8x1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_TX;
    sportCfg.dataEnable = SPORT_SIMPLE_ENABLE_PRIMARY;
    memcpy(sportCfg.dataBuffers, context->tdmAudioOut, sizeof(sportCfg.dataBuffers));
    context->tdmSportOutHandle = single_sport_init(
        SPORT4B, &sportCfg, tdmAudioOut,
        context->tdmAudioOut, &context->tdmAudioOutLen, context, false, NULL
    );
    if (context->tdmSportOutHandle) {
        clock_domain_set(context, CLOCK_DOMAIN_TDM, CLOCK_DOMAIN_BITM_TDM_OUT);
        sportResult = sport_start(context->tdmSportOutHandle, true);
    }


    /* SPORT4A: TDM data in */
    sportCfg = TDM8x1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_RX;
    sportCfg.dataEnable = SPORT_SIMPLE_ENABLE_BOTH;
    memcpy(sportCfg.dataBuffers, context->tdmAudioIn, sizeof(sportCfg.dataBuffers));
    context->tdmSportInHandle = single_sport_init(
        SPORT4A, &sportCfg, tdmAudioIn,
        context->tdmAudioIn, &context->tdmAudioInLen, context, true, NULL
    );
    if (context->tdmSportInHandle) {
        clock_domain_set(context, CLOCK_DOMAIN_TDM, CLOCK_DOMAIN_BITM_TDM_IN);
        sportResult = sport_start(context->tdmSportInHandle, true);
    }
}

void tdm_sport_deinit(APP_CONTEXT *context)
{
    if (context->tdmSportOutHandle) {
        single_sport_deinit(
            &context->tdmSportOutHandle, context->tdmAudioOut, &context->tdmAudioOutLen
        );
        clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_TDM_OUT);
        clock_domain_clr_active(context, CLOCK_DOMAIN_TDM, CLOCK_DOMAIN_BITM_TDM_OUT);
    }
    if (context->tdmSportInHandle) {
        single_sport_deinit(
            &context->tdmSportInHandle, context->tdmAudioIn, &context->tdmAudioInLen
        );
        clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_TDM_IN);
        clock_domain_clr_active(context, CLOCK_DOMAIN_TDM, CLOCK_DOMAIN_BITM_TDM_IN);
    }
}

void tdm_init(APP_CONTEXT *context)
{
    /* Disable BCLK and SYNC clocks */
    tdm_disable(context);

    /* Configure the DAI routing */
    sru_config_tdm_slave();
    sru_config_tdm_cross_clk();

    /* Initialize the TDM SPORTs */
    tdm_sport_init(context);
}

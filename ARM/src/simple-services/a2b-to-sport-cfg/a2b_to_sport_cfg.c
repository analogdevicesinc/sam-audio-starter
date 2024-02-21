/**
 * Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "a2b_to_sport_cfg_cfg.h"
#include "a2b_to_sport_cfg.h"

#include "sport_simple.h"

#ifdef A2B_TO_SPORT_CFG_USE_SYSLOG
#include "syslog.h"
#else
#define syslog_print(x)
#define syslog_printf(...)
#endif

/*****************************************************************************
 * a2b_to_sport_cfg()
 ****************************************************************************/
bool a2b_to_sport_cfg(bool master, bool rx,
    uint8_t I2SGCFG, uint8_t I2SCFG, SPORT_SIMPLE_CONFIG *sportCfg,
    bool verbose, A2B_TO_SPORT_CFG_XCVR xcvrType)
{
    SPORT_SIMPLE_CONFIG backup;
    bool ok = false;
    uint8_t bits;
    char *xcvrStr;

    if (!sportCfg) {
        goto abort;
    }

    switch (xcvrType) {
        case A2B_TO_SPORT_CFG_XCVR_AD241x:
            xcvrStr = "AD241x";
            break;
        case A2B_TO_SPORT_CFG_XCVR_AD242x:
            xcvrStr = "AD242x";
            break;
        case A2B_TO_SPORT_CFG_XCVR_AD243x:
            xcvrStr = "AD243x";
            break;
        default:
            xcvrStr = "Unknown";
            break;
    }

    if (verbose) { syslog_printf("%s A2B SPORT CFG", xcvrStr); }

    /* Save a backup in case of failure */
    memcpy(&backup, sportCfg, sizeof(backup));

    /* Reset elements that are configured */
    sportCfg->clkDir = SPORT_SIMPLE_CLK_DIR_UNKNOWN;
    sportCfg->fsDir = SPORT_SIMPLE_FS_DIR_UNKNOWN;
    sportCfg->dataDir = SPORT_SIMPLE_DATA_DIR_UNKNOWN;
    sportCfg->tdmSlots = SPORT_SIMPLE_TDM_UNKNOWN;
    sportCfg->wordSize = SPORT_SIMPLE_WORD_SIZE_UNKNOWN;
    sportCfg->dataEnable = SPORT_SIMPLE_ENABLE_NONE;
    sportCfg->bitClkOptions = SPORT_SIMPLE_CLK_DEFAULT;
    sportCfg->fsOptions = SPORT_SIMPLE_FS_OPTION_DEFAULT;

    /*
     * Set .clkDir, .fsDir, .dataDir
     *
     * if master, set clk/fs to master, else slave
     * if rx, set to input, else output
     *
     */
    if (master) {
        sportCfg->clkDir = SPORT_SIMPLE_CLK_DIR_MASTER;
        sportCfg->fsDir = SPORT_SIMPLE_FS_DIR_MASTER;
    } else {
        sportCfg->clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE;
        sportCfg->fsDir = SPORT_SIMPLE_FS_DIR_SLAVE;
    }
    if (rx) {
        sportCfg->dataDir = SPORT_SIMPLE_DATA_DIR_RX;
        if (verbose) { syslog_print(" Direction: RX (AD24xx DTX pins)"); }
    } else {
        sportCfg->dataDir = SPORT_SIMPLE_DATA_DIR_TX;
        if (verbose) { syslog_print(" Direction: TX (AD24xx DRX pins)"); }
    }

    /*
     * Set .wordSize
     *
     */
    if (I2SGCFG & 0x10) {
        sportCfg->wordSize = SPORT_SIMPLE_WORD_SIZE_16BIT;
        if (verbose) { syslog_print(" Size: 16-bit"); }
    } else {
        sportCfg->wordSize = SPORT_SIMPLE_WORD_SIZE_32BIT;
        if (verbose) { syslog_print(" Size: 32-bit"); }
    }

    /*
     * Set .tdmSlots
     */
    switch (I2SGCFG & 0x07) {
        case 0:
            sportCfg->tdmSlots = SPORT_SIMPLE_TDM_2;
            if (verbose) { syslog_print(" TDM: 2 (I2S)"); }
            break;
        case 1:
            sportCfg->tdmSlots = SPORT_SIMPLE_TDM_4;
            if (verbose) { syslog_print(" TDM: 4"); }
            break;
        case 2:
            sportCfg->tdmSlots = SPORT_SIMPLE_TDM_8;
            if (verbose) { syslog_print(" TDM: 8"); }
            break;
        case 4:
            sportCfg->tdmSlots = SPORT_SIMPLE_TDM_16;
            if (verbose) { syslog_print(" TDM: 16"); }
            break;
        case 7:
            /*
             * TDM32 with 32-bit word size is not supported with a
             * 24.576MCLK
             */
            if (sportCfg->wordSize == SPORT_SIMPLE_WORD_SIZE_32BIT) {
                goto abort;
            }
            sportCfg->tdmSlots = SPORT_SIMPLE_TDM_32;
            if (verbose) { syslog_print(" TDM: 32"); }
            break;
        default:
            goto abort;
    }

    /*
     * Set .dataEnable
     *
     */
    if (rx) {
        bits = I2SCFG >> 0;
    } else {
        bits = I2SCFG >> 4;
    }
    if ( (xcvrType == A2B_TO_SPORT_CFG_XCVR_AD241x) ||
         (xcvrType == A2B_TO_SPORT_CFG_XCVR_AD242x) ) {
        switch (bits & 0x03) {
            case 0x01:
                sportCfg->dataEnable = SPORT_SIMPLE_ENABLE_PRIMARY;
                if (verbose) { syslog_print(" Data Pins: Primary"); }
                break;
            case 0x02:
                sportCfg->dataEnable = SPORT_SIMPLE_ENABLE_SECONDARY;
                if (verbose) { syslog_print(" Data Pins: Secondary"); }
                break;
            case 0x03:
                sportCfg->dataEnable = SPORT_SIMPLE_ENABLE_BOTH;
                if (verbose) {
                    syslog_print(" Data Pins: Both");
                    syslog_printf(" Interleave: %s", (bits & 0x04) ? "Yes" : "No");
                }
                break;
            default:
                sportCfg->dataEnable = SPORT_SIMPLE_ENABLE_NONE;
                if (verbose) { syslog_print(" Data Pins: None"); }
                break;
        }
    } else if (xcvrType == A2B_TO_SPORT_CFG_XCVR_AD243x) {
        switch (bits & 0x07) {
            case 0:
                sportCfg->dataEnable = SPORT_SIMPLE_ENABLE_NONE;
                if (verbose) { syslog_print(" Data Pins: None"); }
                break;
            case 1:
                sportCfg->dataEnable = SPORT_SIMPLE_ENABLE_PRIMARY;
                if (verbose) { syslog_print(" Data Pins: Primary"); }
                break;
            case 2:
                /* Fall thru */
            case 7:
                sportCfg->dataEnable = SPORT_SIMPLE_ENABLE_BOTH;
                if (verbose) {
                    syslog_print(" Data Pins: Both");
                    syslog_printf(" Interleave: %s", ((bits & 0x07) == 7) ? "Yes" : "No");
                }
                break;
            case 3:  /* Not supported */
            case 4:  /* Not supported */
            default:
                goto abort;
                break;
        }
    } else {
        goto abort;
    }

    /*
     * Set .bitClkOptions
     *
     * Default setting is assert on the rising edge, sample on falling (TDM)
     *
     */
    if (rx) {
        if (I2SCFG & 0x08) {
            sportCfg->bitClkOptions |= SPORT_SIMPLE_CLK_FALLING;
        }
    } else {
        if ((I2SCFG & 0x80) == 0) {
            sportCfg->bitClkOptions |= SPORT_SIMPLE_CLK_FALLING;
        }
    }
    if (sportCfg->bitClkOptions & SPORT_SIMPLE_CLK_FALLING) {
        if (verbose) { syslog_print(" CLK: Assert falling, Sample rising (I2S)"); }
    } else {
        if (verbose) { syslog_print(" CLK: Assert rising, Sample falling"); }
    }

    /*
     * Set .fsOptions
     *
     * Default setting is pulse, rising edge frame sync where the
     * frame sync signal asserts in the same cycle as the MSB of the
     * first data slot (TDM)
     */
    if (I2SGCFG & 0x80) {
        sportCfg->fsOptions |= SPORT_SIMPLE_FS_OPTION_INV;
        if (verbose) { syslog_print(" FS: Falling edge (I2S)"); }
    } else {
        if (verbose) { syslog_print(" FS: Rising edge"); }
    }
    if (I2SGCFG & 0x40) {
        sportCfg->fsOptions |= SPORT_SIMPLE_FS_OPTION_EARLY;
        if (verbose) { syslog_print(" FS: Early (I2S)"); }
    } else {
        if (verbose) { syslog_print(" FS: Not Early"); }
    }
    if (I2SGCFG & 0x20) {
        sportCfg->fsOptions |= SPORT_SIMPLE_FS_OPTION_50;
        if (verbose) { syslog_print(" FS: 50% (I2S)"); }
    } else {
        if (verbose) { syslog_print(" FS: Pulse"); }
    }

    ok = true;

abort:
    if (!ok) {
        memcpy(sportCfg, &backup, sizeof(*sportCfg));
    }
    return(ok);
}

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

/*!
 * @brief  A2B to sport config conversion module
 *
 * Convert A2B I2S registers to a compatible simple SPORT driver
 * configuration.
 *
 * @file      a2b_to_sport.h
 * @version   1.0.0
 * @copyright 2023 Analog Devices, Inc.  All rights reserved.
 *
*/
#ifndef _a2b_to_sport_cfg_h
#define _a2b_to_sport_cfg_h

#include <stdint.h>
#include <stdbool.h>

#include "sport_simple.h"

typedef enum A2B_TO_SPORT_CFG_XCVR {
    A2B_TO_SPORT_CFG_XCVR_UNKNOWN = 0,
    A2B_TO_SPORT_CFG_XCVR_AD241x,
    A2B_TO_SPORT_CFG_XCVR_AD242x,
    A2B_TO_SPORT_CFG_XCVR_AD243x
} A2B_TO_SPORT_CFG_XCVR;

/*!****************************************************************
 * @brief Convert A2B I2S registers to a compatible simple SPORT driver
 * configuration.
 *
 * This function converts A2B I2SGCFG and I2SCFG registers into the
 * equivalent DSP SPORT config
 *
 * @param [in]  master      True if the transceiver is an A2B main/master
 *                          false if an A2B sub/slave
 * @param [in]  rx          True if the SPORT is receiving data, false
 *                          if the SPORT is transmitting data
 * @param [in]  I2SGCFG     I2S Global Config Register
 * @param [in]  I2SCFG      I2S Config Register
 * @param [out] sportCfg    Pointer to a SPORT_SIMPLE_CONFIG struct to
 *                          configure
 * @param [in]  verbose     True for high verbosity, false for no verbosity
 * @param [in]  xcvrType    AD24xx transceiver type
 *
 * @return True if successful, false if the A2B transceiver I2S
 *         configuration is not compatible with the DSP SPORT.
 ******************************************************************/
bool a2b_to_sport_cfg(bool master, bool rx,
    uint8_t I2SGCFG, uint8_t I2SCFG, SPORT_SIMPLE_CONFIG *sportCfg,
    bool verbose, A2B_TO_SPORT_CFG_XCVR xcvrType);

#endif

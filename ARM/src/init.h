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
#ifndef _init_h
#define _init_h

/* Standard includes */
#include <stdint.h>
#include <stdbool.h>

/* Project includes */
#include "context.h"
#include "gpio_pins.h"

void gic_init(void);
void gic_set_irq_prio(void);
void system_clk_init(uint32_t *cclk);
void cgu_ts_init(void);
void gpio_init(void);
void flash_init(APP_CONTEXT *context);
void umm_heap_init(void);
void eth_hardware_init(APP_CONTEXT *context);

void mclk_init(APP_CONTEXT *context);
void disable_mclk(APP_CONTEXT *context);
void enable_mclk(APP_CONTEXT *context);

void adau1761_init(APP_CONTEXT *context);

void spdif_init(APP_CONTEXT *context);

bool a2b_master_init(APP_CONTEXT *context);
void a2b_reset(APP_CONTEXT *context);
bool a2b_restart(APP_CONTEXT *context);
bool a2b_set_mode(APP_CONTEXT *context, A2B_BUS_MODE mode);
bool a2b_sport_start(APP_CONTEXT *context, uint8_t I2SGCFG, uint8_t I2SCFG);
bool a2b_sport_stop(APP_CONTEXT *context);
void a2b_pint_init(APP_CONTEXT *context);

bool a2b2_master_init(APP_CONTEXT *context);
bool a2b2_restart(APP_CONTEXT *context);
bool a2b2_sport_deinit(APP_CONTEXT *context);

void sae_buffer_init(APP_CONTEXT *context);

void audio_routing_init(APP_CONTEXT *context);

void system_reset(APP_CONTEXT *context);
int sam_hw_version(APP_CONTEXT *context);

#endif

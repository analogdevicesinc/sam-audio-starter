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

/* CCES includes */
#include <services/gpio/adi_gpio.h>

/* Project includes */
#include "context.h"
#include "twi_simple.h"

#define PB1             ADI_GPIO_PIN_0
#define PB2             ADI_GPIO_PIN_1
#define PUSHBUTTON_PORT ADI_GPIO_PORT_F

#define A2B_IO7_PIN     ADI_GPIO_PIN_13
#define A2B_IO7_PORT    ADI_GPIO_PORT_F

#define LED_PORT        ADI_GPIO_PORT_D
#define LED1            ADI_GPIO_PIN_1
#define LED2            ADI_GPIO_PIN_2
#define LED3            ADI_GPIO_PIN_3

void system_clk_init(uint32_t *cclk);
void cgu_ts_init(void);
void gpio_init(void);
void gic_init(void);
void heap_init(void);
void flash_init(APP_CONTEXT *context);

int sam_hw_version(APP_CONTEXT *context);

void adau1761_init(APP_CONTEXT *context);
void spdif_init(APP_CONTEXT *context);
bool ad2425_init_master(APP_CONTEXT *context);
void ad2425_reset(APP_CONTEXT *context);
bool ad2425_restart(APP_CONTEXT *context);
bool ad2425_set_mode(APP_CONTEXT *context, A2B_BUS_MODE mode);
bool ad2425_sport_start(APP_CONTEXT *context, uint8_t I2SGCFG, uint8_t I2SCFG);
bool ad2425_sport_stop(APP_CONTEXT *context);

void disable_sport_mclk(APP_CONTEXT *context);
void enable_sport_mclk(APP_CONTEXT *context);
void audio_mclk_24576_mhz(APP_CONTEXT *context);
void debug_signal_init(void);

void emac0_phy_init(APP_CONTEXT *context);

void sae_buffer_init(APP_CONTEXT *context);
void audio_routing_init(APP_CONTEXT *context);

void system_reset(APP_CONTEXT *context);

#endif

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

#ifndef _a2b_irq_h
#define _a2b_irq_h

#include <stdint.h>
#include <services/gpio/adi_gpio.h>

#include "context.h"

/* A2B on J10 */
#define A2B1_PINT_IRQ  (ADI_GPIO_PIN_INTERRUPT_5)
#define A2B1_PINT_PIN  (ADI_GPIO_INT_PIN_7)

typedef enum A2B_BUS_NUM {
    A2B_BUS_NUM_UNKNOWN = 0,
    A2B_BUS_NUM_1,
    A2B_BUS_NUM_2,
    A2B_BUS_NUM_MAX
} A2B_BUS_NUM;

typedef void (*A2B_IRQ_CB)(A2B_BUS_NUM b, uint8_t intSrc, uint8_t intType, void *usr);

void a2b_irq(ADI_GPIO_PIN_INTERRUPT const pint, uint32_t const pins, void *usr);
void a2b_irq_init(APP_CONTEXT *context);
void a2b_irq_disable(APP_CONTEXT *context, A2B_BUS_NUM b);
void a2b_irq_enable(APP_CONTEXT *context, A2B_BUS_NUM b);

void *a2b_irq_register(A2B_BUS_NUM b, uint8_t intType, A2B_IRQ_CB cb, void *usr);
void a2b_irq_unregister(uint8_t intType, void *handle);

#endif

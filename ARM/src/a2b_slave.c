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
/* Standard libary includes */
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Simple driver includes */
#include "twi_simple.h"

/* Application includes */
#include "context.h"
#include "a2b_slave.h"
#include "task_cfg.h"
#include "syslog.h"
#include "init.h"

#define AD242X_I2SGCFG          0x41u
#define AD242X_NODE             0x29u
#define AD242X_NODE_DISCVD      0x20u

#define AD242X_I2SCFG_DATA_EN   0x33u

/* A2B slave mode management task */
portTASK_FUNCTION(a2bSlaveTask, pvParameters)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    TWI_SIMPLE_RESULT result;
    uint8_t A2B_NODE_REG;
    uint8_t A2B_REG;
    uint8_t rbuf[2];
    bool ok;

    while (1) {
        if (context->a2bmode == A2B_BUS_MODE_SUB) {
            A2B_REG = AD242X_NODE;
            result = twi_writeRead(context->a2bTwiHandle, context->cfg.a2bI2CAddr,
                &A2B_REG, sizeof(A2B_REG),
                &A2B_NODE_REG, sizeof(A2B_NODE_REG)
            );
            if (result != TWI_SIMPLE_SUCCESS) {
                continue;
            }
            if (A2B_NODE_REG & AD242X_NODE_DISCVD) {
                if (!context->a2bSlaveActive) {
                    A2B_REG = AD242X_I2SGCFG;
                    result = twi_writeRead(context->a2bTwiHandle, context->cfg.a2bI2CAddr,
                        &A2B_REG, sizeof(A2B_REG),
                        rbuf, sizeof(rbuf)
                    );
                    if (result != TWI_SIMPLE_SUCCESS) {
                        continue;
                    }
                    if (rbuf[1] & AD242X_I2SCFG_DATA_EN) {
                        syslog_printf("A2B Slave SPORT Start (%02x:%02x)", rbuf[0], rbuf[1]);
                        ok = a2b_sport_start(context, rbuf[0], rbuf[1]);
                        if (!ok) {
                            syslog_printf("A2B Slave SPORT Start Failed");
                        }
                        context->a2bSlaveActive = true;
                    }
                }
            } else {
                if (context->a2bSlaveActive) {
                    syslog_printf("Slave SPORT Stop");
                    a2b_sport_stop(context);
                    context->a2bSlaveActive = false;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void a2b_slave_init(APP_CONTEXT *context)
{
    /* Don't start of no A2B */
    if (!context->a2bPresent) {
        return;
    }

    xTaskCreate( a2bSlaveTask, "A2BSlaveTask", GENERIC_TASK_STACK_SIZE,
        context, HOUSEKEEPING_PRIORITY, &context->a2bSlaveTaskHandle );
}

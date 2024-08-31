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
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Application includes */
#include "context.h"
#include "pushbutton.h"
#include "syslog.h"
#include "init.h"
#include "gpio_pins.h"

#ifndef PB1_CMD
#define PB1_CMD "pushbtn1.cmd"
#endif

#ifndef PB2_CMD
#define PB2_CMD "pushbtn2.cmd"
#endif

#define PB_POLL_MS   20
#define PB_DEBOUNCE  2

portTASK_FUNCTION( pushButtonTask, pvParameters )
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    TickType_t pollRate, lastPollTime;
    unsigned debouncePB1, debouncePB2;
    bool lastPB1, lastPB2;
    bool doPB1, doPB2;
    bool PB;

    /* Set polling rate */
    pollRate = pdMS_TO_TICKS(PB_POLL_MS);
    lastPollTime = xTaskGetTickCount();

    /* Setup debounce */
    debouncePB1 = debouncePB2 = PB_DEBOUNCE;
    lastPB1 = lastPB2 = false;
    doPB1 = doPB2 = false;

    while (1) {
        /* Check PB1 */
        PB = gpio_get_pin(gpioPins, PB1);
        if (PB != lastPB1) {
            if (--debouncePB1 == 0) {
                lastPB1 = PB; doPB1 = PB;
                debouncePB1 = PB_DEBOUNCE;
            }
        } else {
            debouncePB1 = PB_DEBOUNCE;
        }
        if (doPB1) {
            syslog_print("PB1 pressed, running " PB1_CMD);
            shell_exec(&context->shell, "run " PB1_CMD);
            doPB1 = false;
        }

        /* Check PB2 */
        PB = gpio_get_pin(gpioPins, PB2);
        if (PB != lastPB2) {
            if (--debouncePB2 == 0) {
                lastPB2 = PB; doPB2 = PB;
                debouncePB2 = PB_DEBOUNCE;
            }
        } else {
            debouncePB2 = PB_DEBOUNCE;
        }
        if (doPB2) {
            syslog_print("PB2 pressed, running " PB2_CMD);
            shell_exec(&context->shell, "run " PB2_CMD);
            doPB2 = false;
        }

        /* Sleep for a while */
        vTaskDelayUntil(&lastPollTime, pdMS_TO_TICKS(pollRate));
    }
}

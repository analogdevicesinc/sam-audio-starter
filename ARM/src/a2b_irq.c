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

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "twi_simple.h"

#include "context.h"
#include "a2b_irq.h"
#include "task_cfg.h"
#include "syslog.h"
#include "umm_malloc.h"

#define A2B_IRQ_DEBUG

#define A2B_INTTYPE_MAX  256

typedef struct _A2B_IRQ_REGS {
    uint8_t intSrc;
    uint8_t intType;
} A2B_IRQ_REGS;

typedef struct _A2B_IRQ_LISTENER {
    A2B_BUS_NUM b;
    A2B_IRQ_CB cb;
    void *usr;
    void *next;
} A2B_IRQ_LISTENER;

#define A2B_IRQ_LOCK()    xSemaphoreTake(a2bIrqMutex, portMAX_DELAY)
#define A2B_IRQ_UNLOCK()  xSemaphoreGive(a2bIrqMutex)

#define AD24xx_REG_INTSTAT   0x15
#define AD24xx_REG_INTSRC    0x16
#define AD24xx_REG_LINTTYPE  0x3E

static A2B_IRQ_LISTENER *listeners[A2B_INTTYPE_MAX] = { 0 };

static QueueHandle_t a2bIrqQueue = NULL;
static SemaphoreHandle_t a2bIrqMutex = NULL;

void a2b_irq(ADI_GPIO_PIN_INTERRUPT const pint, uint32_t const pins, void *usr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usr;
    A2B_BUS_NUM busNum = A2B_BUS_NUM_UNKNOWN;
    BaseType_t wake;

    UNUSED(context);

    if ((pint == A2B1_PINT_IRQ) && (pins & A2B1_PINT_PIN)) {
        busNum = A2B_BUS_NUM_1;
    }

    if (busNum != A2B_BUS_NUM_UNKNOWN) {
        if (a2bIrqQueue) {
            xQueueSendFromISR(a2bIrqQueue, &busNum, &wake);
            portYIELD_FROM_ISR(wake);
        }
    }
}

static TWI_SIMPLE_RESULT a2b_irq_twi_xfer(
    APP_CONTEXT *context, A2B_BUS_NUM b,
    void *wBuf, uint16_t wLen, void *rBuf, uint16_t rLen)
{
    sTWI *a2bTwiHandle;
    uint8_t a2bI2CAddr;
    TWI_SIMPLE_RESULT result;

    if (b == A2B_BUS_NUM_1) {
        a2bTwiHandle = context->a2bTwiHandle;
        a2bI2CAddr = context->cfg.a2bI2CAddr;
    } else {
        return(TWI_SIMPLE_ERROR);
    }

    result = twi_writeRead(a2bTwiHandle, a2bI2CAddr, wBuf, wLen, rBuf, rLen);

    return(result);
}


static portTASK_FUNCTION(a2bIrqTask, pvParameters)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    TWI_SIMPLE_RESULT result;
    A2B_IRQ_REGS irqRegs;
    A2B_BUS_NUM b;
    BaseType_t ok;
    uint8_t regAddr;
    A2B_IRQ_LISTENER *ll;
    A2B_BUS_MODE a2bmode;
    bool a2bIrqDisable;
    char *name;

    UNUSED(context);

    while (1) {
        ok = xQueueReceive(a2bIrqQueue, &b, portMAX_DELAY);
        if (ok == pdTRUE) {
            if (b == A2B_BUS_NUM_1) {
                a2bmode = context->a2bmode;
                a2bIrqDisable = context->a2bIrqDisable;
                name = "a2b";
            } else {
                a2bmode = A2B_BUS_MODE_UNKNOWN;
                a2bIrqDisable = true;
                name = NULL;
            }

            if ((a2bmode != A2B_BUS_MODE_UNKNOWN) && (a2bIrqDisable == false)) {
                if (a2bmode == A2B_BUS_MODE_MAIN) {
                    regAddr = AD24xx_REG_INTSRC;
                    result = a2b_irq_twi_xfer(
                        context, b,
                        &regAddr, sizeof(regAddr), (uint8_t *)&irqRegs, sizeof(irqRegs)
                    );
                } else {
                    regAddr = AD24xx_REG_LINTTYPE;
                    result = a2b_irq_twi_xfer(
                        context, b,
                        &regAddr, sizeof(regAddr),
                        &irqRegs.intType, sizeof(irqRegs.intType)
                    );
                    irqRegs.intSrc = 0;
                }
#ifdef A2B_IRQ_DEBUG
                syslog_printf("%s IRQ: %c:%02X:%02X",
                    name,
                    a2bmode ==  A2B_BUS_MODE_MAIN ? 'M' : 'S',
                    irqRegs.intSrc, irqRegs.intType
                );
#endif
                /* Service active interrupts */
                A2B_IRQ_LOCK();
                ll = listeners[irqRegs.intType];
                while (ll) {
                    ll->cb(b, irqRegs.intSrc, irqRegs.intType, ll->usr);
                    ll = ll->next;
                }
                A2B_IRQ_UNLOCK();
            }
        }
    }
}

void *a2b_irq_register(A2B_BUS_NUM b, uint8_t intType, A2B_IRQ_CB cb, void *usr)
{
    A2B_IRQ_LISTENER *l;
    A2B_IRQ_LISTENER *ll;

    if (intType >= A2B_INTTYPE_MAX) {
        return(NULL);
    }

    l = umm_calloc(1, sizeof(*l));

    l->b = b;
    l->cb = cb;
    l->usr = usr;

    A2B_IRQ_LOCK();

    ll = listeners[intType];
    if (ll == NULL) {
        listeners[intType] = l;
    } else {
        while (ll->next) {
            ll = ll->next;
        }
        ll->next = l;
    }

    A2B_IRQ_UNLOCK();

    return(l);
}

void a2b_irq_unregister(uint8_t intType, void *handle)
{
    A2B_IRQ_LISTENER *l;
    A2B_IRQ_LISTENER *ll;

    l = (A2B_IRQ_LISTENER *)handle;

    A2B_IRQ_LOCK();

    ll = listeners[intType];
    if (ll == NULL) {
        A2B_IRQ_UNLOCK();
        return;
    }

    if (ll == l) {
        listeners[intType] = ll->next;
    } else {
        while (ll->next && (ll->next != l)) {
            ll = ll->next;
        }
        if (ll->next) {
            ll->next = l->next;
        }
    }

    A2B_IRQ_UNLOCK();

    umm_free(l);
}

void a2b_irq_disable(APP_CONTEXT *context, A2B_BUS_NUM b)
{
    if (b == A2B_BUS_NUM_1) {
        context->a2bIrqDisable = true;
    }
}

void a2b_irq_enable(APP_CONTEXT *context, A2B_BUS_NUM b)
{
    TWI_SIMPLE_RESULT result;
    uint8_t regAddr;
    uint8_t intStat;

    if (b == A2B_BUS_NUM_1) {
        context->a2bIrqDisable = false;

        /* Service any pending interrupts */
        regAddr = AD24xx_REG_INTSTAT;
        result = a2b_irq_twi_xfer(
            context, b,
            &regAddr, sizeof(regAddr), (uint8_t *)&intStat, sizeof(intStat)
        );
        if (intStat & 0x01) {
            if (a2bIrqQueue) {
                xQueueSend(a2bIrqQueue, &b, portMAX_DELAY);
            }
        }
    }
}

void a2b_irq_init(APP_CONTEXT *context)
{
    /* Create A2B IRQ processing mutex */
    a2bIrqMutex = xSemaphoreCreateMutex();

    /* Don't start of no A2B */
    if (!context->a2bPresent) {
        return;
    }

    /* Create a IRQ processing queue */
    a2bIrqQueue = xQueueCreate(16, sizeof(A2B_BUS_NUM));

    /* Create IRQ processing task */
    xTaskCreate(a2bIrqTask, "A2BIrqTask", GENERIC_TASK_STACK_SIZE,
        context, A2B_IRQ_TASK_PRIORITY, &context->a2bIrqTaskHandle);
}

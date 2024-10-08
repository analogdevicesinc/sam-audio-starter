/**
 * Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#include <stddef.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "cpu_load.h"

static CPU_LOAD_GET_TIME getTime = NULL;
static TaskHandle_t backgroundTaskHandle = NULL;
static TaskHandle_t idleTaskHandle = NULL;
static bool inIdleTask = false;

static uint32_t ticksPerSecond = 0;
static uint32_t isrCyclesTotal = 0;
static uint32_t idleCyclesTotal = 0;
static uint32_t lastCycles = 0;
static uint32_t cyclesTotal = 0;

static uint32_t cpuLoad = 0;
static uint32_t maxCpuLoad = 0;

volatile static uint32_t inCycles = 0;
volatile static uint32_t outCycles = 0;
volatile static int32_t netstedIntrCount = 0;

void cpuLoadInit(CPU_LOAD_GET_TIME _getTime, uint32_t _ticksPerSecond)
{
    /* Get the idle task handle */
    idleTaskHandle = xTaskGetIdleTaskHandle();

    /* Save the time function */
    getTime = _getTime;

    /* Save the ticks per second */
    ticksPerSecond = _ticksPerSecond;
}

void cpuLoadBackgroundTask(void *taskHandle)
{
    backgroundTaskHandle = (TaskHandle_t)taskHandle;
}

void cpuLoadtaskSwitchHook(void *taskHandle)
{
    static uint32_t idleInCycles;

    if ( (taskHandle == idleTaskHandle) ||
         (taskHandle == backgroundTaskHandle) ) {
        idleInCycles = getTime();
        inIdleTask = true;
    } else {
        if (inIdleTask) {
            idleCyclesTotal += getTime() - idleInCycles;
            inIdleTask = false;
        }
    }
}

void cpuLoadIsrCycles(uint32_t isrCycles)
{
    UBaseType_t isrStat;

    /* Disable nested interrupts */
    isrStat = taskENTER_CRITICAL_FROM_ISR();

    /* Only accumulate ISR cycles in the idle task */
    if (inIdleTask) {
        isrCyclesTotal += isrCycles;
    }

    /* Re-enable nested interrupts */
    taskEXIT_CRITICAL_FROM_ISR(isrStat);
}

void cpuLoadISREnter(void)
{
    UBaseType_t isrStat;

    /* If nesting counter is 0, this is not a nested interrupt call
    Otherwise, do not count the cycles in case of nested interrupts */

    /* Disable nested interrupts */
    isrStat = taskENTER_CRITICAL_FROM_ISR();

    if (netstedIntrCount == 0) {
        inCycles = cpuLoadGetTimeStamp();
    }

    /* Increment the counter for nested intr */
    netstedIntrCount++;

    /* Re-enable nested interrupts */
    taskEXIT_CRITICAL_FROM_ISR(isrStat);
}

void cpuLoadISRExit(void)
{
    UBaseType_t isrStat;

    /* If nesting counter is 1, this is not a nested interrupt call
    Otherwise, do not count the cycles in case of nested interrupts */

    /* Disable nested interrupts */
    isrStat = taskENTER_CRITICAL_FROM_ISR();

    netstedIntrCount--;

    if (netstedIntrCount == 0) {
        outCycles = cpuLoadGetTimeStamp();
        cpuLoadIsrCycles(outCycles - inCycles);
    }

    /* Re-enable nested interrupts */
    taskEXIT_CRITICAL_FROM_ISR(isrStat);
}

uint32_t cpuLoadCalculateLoad(uint32_t *maxLoad)
{
    uint32_t now;

    taskENTER_CRITICAL();

    /* Compute the number of elapsed cycles */
    now = getTime();

    /* Compute the number of elapsed cycles */
    cyclesTotal = now - lastCycles;

    /* Subtract for interrupt cycles during idle tasks */
    idleCyclesTotal -= isrCyclesTotal;

    /* Compute load */
    cpuLoad = (100u * (cyclesTotal - idleCyclesTotal)) / cyclesTotal;
    if (cpuLoad > maxCpuLoad) {
        maxCpuLoad = cpuLoad;
    }

    /* Reset all counters */
    idleCyclesTotal = 0;
    isrCyclesTotal = 0;
    lastCycles = now;

    taskEXIT_CRITICAL();

    /* Report back the results */
    if (maxLoad) {
        *maxLoad = maxCpuLoad;
    }

    return(cpuLoad);
}

uint32_t cpuLoadGetLoad(uint32_t *maxLoad, bool clearMax)
{
    if (maxLoad) {
        *maxLoad = (cpuLoad > maxCpuLoad) ? cpuLoad : maxCpuLoad;
    }
    if (clearMax) {
        maxCpuLoad = 0;
    }
    return(cpuLoad);
}

uint32_t cpuLoadGetTimeStamp(void)
{
    return(getTime());
}

uint32_t cpuLoadCyclesToMicrosecond(uint32_t cycles)
{
    uint32_t uS = 0;

    if (ticksPerSecond) {
        uS = ((1000000ULL) * (uint64_t)cycles) / ticksPerSecond;
    }

    return(uS);
}

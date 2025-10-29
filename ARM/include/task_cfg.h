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

#ifndef _task_cfg_h
#define _task_cfg_h

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* The priorities assigned to the tasks (higher number == higher prio). */
#define HOUSEKEEPING_PRIORITY       (tskIDLE_PRIORITY + 1)
#define STARTUP_TASK_LOW_PRIORITY   (tskIDLE_PRIORITY + 1)
#define TELNET_TASK_PRIORITY        (tskIDLE_PRIORITY + 2)
#define VU_TASK_PRIORITY            (tskIDLE_PRIORITY + 2)
#define UAC20_TASK_PRIORITY         (tskIDLE_PRIORITY + 3)
#define WAV_TASK_PRIORITY           (tskIDLE_PRIORITY + 3)
#define RTP_TASK_PRIORITY           (tskIDLE_PRIORITY + 3)
#define VBAN_TASK_PRIORITY          (tskIDLE_PRIORITY + 3)
#define ETHERNET_PRIORITY           (tskIDLE_PRIORITY + 4)
#define ETHER_WORKER_PRIO           (tskIDLE_PRIORITY + 4)
#define A2B_IRQ_TASK_PRIORITY       (tskIDLE_PRIORITY + 4)
#define STARTUP_TASK_HIGH_PRIORITY  (tskIDLE_PRIORITY + 5)

/* The some shell commands require a little more stack (startup task). */
#define HOUSEKEEPING_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 512)
#define STARTUP_TASK_STACK_SIZE      (configMINIMAL_STACK_SIZE + 8192)
#define TELNET_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE + 8192)
#define PUSHBUTTON_TASK_STACK_SIZE   (configMINIMAL_STACK_SIZE + 8192)
#define UAC20_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE + 1024)
#define VU_TASK_STACK_SIZE           (configMINIMAL_STACK_SIZE + 128)
#define WAV_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE + 128)
#define RTP_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE + 256)
#define VBAN_TASK_STACK_SIZE         (configMINIMAL_STACK_SIZE + 256)
#define ETHERNET_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE + 256)
#define TCPIP_THREAD_STACKSIZE       (configMINIMAL_STACK_SIZE + 256)
#define GENERIC_TASK_STACK_SIZE      (configMINIMAL_STACK_SIZE)

#endif

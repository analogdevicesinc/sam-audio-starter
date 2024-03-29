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

#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Simple service includes */
#include "buffer_track.h"
#include "uac2_soundcard.h"

/* OSS includes */
#include "umm_malloc.h"

/* Application includes */
#include "context.h"
#include "umm_malloc.h"
#include "uac2.h"
#include "util.h"
#include "usb_audio.h"

/* UAC2 soundcard management task */
portTASK_FUNCTION(uac2Task, pvParameters)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    APP_CFG *cfg = &context->cfg;
    uint32_t whatToDo;
    CLD_RV ret;
    uint32_t dataSize;

    /* Allocate and configure the UAC2 Rx (OUT endpoint) ring buffer.
     * The ring buffer unit of measure is SYSTEM_AUDIO_TYPE sized words.
     *
     * Try to fit into L2 first for performance.
     */
    context->uac2OutRx =
        (PaUtilRingBuffer *)umm_malloc(sizeof(PaUtilRingBuffer));
    dataSize = roundUpPow2(USB_OUT_RING_BUFF_FRAMES * cfg->usbOutChannels);
    context->uac2OutRxData =
        umm_calloc_heap(UMM_L2_CACHED_HEAP, dataSize, cfg->usbWordSize);
    if (context->uac2OutRxData == NULL) {
        context->uac2OutRxData =
            umm_calloc_heap(UMM_SDRAM_HEAP, dataSize, cfg->usbWordSize);
    }
    PaUtil_InitializeRingBuffer(context->uac2OutRx,
        cfg->usbWordSize, dataSize, context->uac2OutRxData);

    /* Allocate and configure the UAC2 Tx (IN endpoint) ring buffer.
     * The ring buffer unit of measure is SYSTEM_AUDIO_TYPE sized words.
     *
     * Try to fit into L2 first for performance.
     */
    context->uac2InTx =
        (PaUtilRingBuffer *)umm_malloc(sizeof(PaUtilRingBuffer));
    dataSize = roundUpPow2(USB_IN_RING_BUFF_FRAMES * cfg->usbInChannels);
    context->uac2InTxData =
        umm_calloc_heap(UMM_L2_CACHED_HEAP, dataSize, cfg->usbWordSize);
    if (context->uac2InTxData == NULL) {
        context->uac2InTxData =
            umm_calloc_heap(UMM_SDRAM_HEAP, dataSize, cfg->usbWordSize);
    }
    PaUtil_InitializeRingBuffer(context->uac2InTx,
        cfg->usbWordSize, dataSize, context->uac2InTxData);

    /* Initialize the buffer level tracking module for UAC2 rate feedback.
     * In this system, the CODEC is the clock master for everything.
     * Therefore, only the CODEC output buffer (uac2OutRxData)
     * will be monitored for UAC2 rate feedback.
     */
    bufferTrackInit(getTimeStamp);

    /* Configure UAC2 application settings */
    context->uac2cfg.port = CLD_USB_0;
    context->uac2cfg.usbSampleRate = SYSTEM_SAMPLE_RATE;
    context->uac2cfg.usbInChannels = cfg->usbInChannels;
    context->uac2cfg.usbInWordSizeBits = cfg->usbWordSize * 8;
    context->uac2cfg.usbOutChannels = cfg->usbOutChannels;
    context->uac2cfg.usbOutWordSizeBits = cfg->usbWordSize * 8;
    context->uac2cfg.vendorId = USB_VENDOR_ID;
    context->uac2cfg.productId = USB_PRODUCT_ID;
    context->uac2cfg.mfgString = USB_MFG_STRING;
    context->uac2cfg.productString = USB_PRODUCT_STRING;
    context->uac2cfg.serialNumString = USB_SERIAL_NUMBER_STRING;
    context->uac2cfg.lowLatency = false;
    context->uac2cfg.usbOutStats = &context->uac2stats.rx.ep;
    context->uac2cfg.usbInStats = &context->uac2stats.tx.ep;
    context->uac2cfg.rxCallback = uac2Rx;
    context->uac2cfg.txCallback = uac2Tx;
    context->uac2cfg.rateFeedbackCallback = uac2RateFeedback;
    context->uac2cfg.endpointEnableCallback = uac2EndpointEnabled;
    context->uac2cfg.usrPtr = context;
    context->uac2cfg.timerNum = USB_TIMER;

    /* Initialize, configure, and start the CLD UAC20 library */
    ret = uac2_init();
    ret = uac2_config(&context->uac2cfg);
    ret = uac2_start();

    while (1) {

        /* Run the UAC2 library */
        ret = uac2_run();

        /* Wait for something to do, like a CODEC SPORT DMA complete, or a
         * 10mS poll timeout.  'whatToDo' carries more information about what
         * triggered the notification, but we really don't care at this point.
         */
        whatToDo = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
}

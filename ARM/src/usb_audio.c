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

/* Standard includes */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Simple service includes */
#include "buffer_track.h"
#include "cpu_load.h"
#include "context.h"
#include "util.h"
#include "clock_domain.h"

static bool txPreRoll = true;

static SYSTEM_AUDIO_TYPE rxBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];
static SYSTEM_AUDIO_TYPE txBuffer[SYSTEM_MAX_CHANNELS * SYSTEM_BLOCK_SIZE];

unsigned usbBits2bytes(unsigned bits)
{
    unsigned bSubslotSize;
    if (bits <= 8) {
        bSubslotSize = 1;
    } else if (bits <= 16) {
        bSubslotSize = 2;
    } else {
        bSubslotSize = 4;
    }
    return(bSubslotSize);
}

/*
 * This callback is called whenever audio data is available from the
 * host via the UAC2 OUT endpoint.  This callback runs in an
 * ISR context.
 *
 * Optionally set 'nextData' to the next transfer buffer if using
 * application buffers.
 *
 */
uint16_t uac2Rx(void *data, void **nextData, uint16_t rxSize, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;
    uint32_t inCycles, outCycles;

    UNUSED(context);

    /* Track ISR cycle count for CPU load */
    inCycles = cpuLoadGetTimeStamp();

    unsigned samples;
    unsigned sampleSizeBytes;
    unsigned framesAvailable;
    unsigned frames;

    /* Accumulate the fill level of the USB receive ring buffer for
     * rate feedback calculation.
     */
    samples = PaUtil_GetRingBufferReadAvailable(context->uac2OutRx);
    bufferTrackAccum(UAC2_OUT_BUFFER_TRACK_IDX, samples);

    /* Calculate the number of frames that came in over USB */
    sampleSizeBytes = usbBits2bytes(context->cfg.usbWordSizeBits);
    samples = rxSize / sampleSizeBytes;
    frames = samples / context->cfg.usbOutChannels;

    /* Calculate the number of free frames available in the ring buffer. */
    samples = PaUtil_GetRingBufferWriteAvailable(context->uac2OutRx);
    framesAvailable = samples / context->cfg.usbOutChannels;

    /* Copy in the audio data if there is space */
    if (framesAvailable >= frames ) {
        if (sampleSizeBytes == sizeof(SYSTEM_AUDIO_TYPE)) {
            /* 32-bit -> 32-bit or 16-bit -> 16-bit */
            PaUtil_WriteRingBuffer(
                context->uac2OutRx, data,
                context->cfg.usbOutChannels * frames
            );
        } else {
            /* 16-bit -> 32-bit, 32-bit -> 16-bit */
            copyAndConvert(
                data, sampleSizeBytes, context->cfg.usbOutChannels,
                rxBuffer, sizeof(SYSTEM_AUDIO_TYPE), context->cfg.usbOutChannels,
                frames, false
            );
            PaUtil_WriteRingBuffer(
                context->uac2OutRx, rxBuffer,
                context->cfg.usbOutChannels * frames
            );
        }
    } else {
        context->uac2stats.rx.usbRxOverRun++;
    }

    /* Track ISR cycle count for CPU load */
    outCycles = cpuLoadGetTimeStamp();
    cpuLoadIsrCycles(outCycles - inCycles);

    return(rxSize);
}

/*
 * This callback is called whenever audio data is requested by the
 * host via the UAC2 IN endpoint.  This callback runs in an
 * ISR context.
 *
 * Compliance with section 2.3.1.1 of the Audio Data Formats 2.0
 * Specification must happen here.  This section limits the variation
 * in the packet size by +/- 1 audio frame from the nominal size.
 *
 * The 'minSize' and 'maxSize' parameters define those bounds.  Under
 * normal circumstances, packets must never be smaller than 'minSize'
 * or larger than 'maxSize'.
 *
 * The application must strive to deliver the majority of packets at
 * the nominal size of '(minSize+maxSize)/2'.  The occasional delivery
 * of 'minSize' and 'maxSize' must be spread out as evenly as possible
 * to maximize the host's implicit rate feedback accuracy.
 *
 * if 'minSize' or 'maxSize' are ever zero then there was a problem
 * configuring the endpoint descriptors most likely due to insufficient
 * bandwidth reqired to satisfy the requested number of IN/Tx channels.
 *
 * Optionally set 'nextData' to the next transfer buffer if using
 * application buffers.  This buffer will go out immediately upon return
 * to satisfy the current request.
 */
uint16_t uac2Tx(void *data, void **nextData,
    uint16_t minSize, uint16_t maxSize, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;
    uint32_t inCycles, outCycles;

    UNUSED(context);

    /* Track ISR cycle count for CPU load */
    inCycles = cpuLoadGetTimeStamp();

    bool sampleRateUpdate;
    unsigned samples;
    unsigned sampleSizeBytes;
    uint32_t ringFrames;
    unsigned uacFrames;
    unsigned targetRingFrames;
    int error;
    unsigned size;

    /* Adjustment tracking variables */
    static unsigned adjustCountCurr;
    static unsigned adjustCountThresh;
    static int adjustValue;

    /* Sanity check */
    if ((minSize == 0) || (maxSize == 0)) {
        return(0);
    }

    /*
     * Calculate the number of frames available in the ring buffer.
     */
    samples = PaUtil_GetRingBufferReadAvailable(context->uac2InTx);
    ringFrames = samples / context->cfg.usbInChannels;

    /* Accumulate the fill level of the USB IN (Tx) ring buffer
     * for +/- single frame USB rate adjustments
     */
    bufferTrackAccum(UAC2_IN_BUFFER_TRACK_IDX, samples);

    /* Calculate the nominal number of frames to transmit over USB */
    sampleSizeBytes = usbBits2bytes(context->cfg.usbWordSizeBits);
    uacFrames = ((maxSize + minSize) / 2) /
        (context->cfg.usbInChannels * sampleSizeBytes);

    /* Wait USB_IN_RING_BUFF_FILL frames to be available in the ring buffer.
     * Maintain this level with single frame adjustments over time.  If for
     * some reason it drops to less than 1 frame then re-start the preroll.
     */
    targetRingFrames = USB_IN_RING_BUFF_FILL;

    if (txPreRoll) {
        if (ringFrames < targetRingFrames) {
            txPreRoll = true;
            return(0);
        } else {
            bufferTrackReset(UAC2_IN_BUFFER_TRACK_IDX);
            adjustCountCurr = 0;
            adjustCountThresh = 0;
            adjustValue = 0;
            txPreRoll = false;
        }
    } else {
        if (ringFrames < uacFrames) {
            context->uac2stats.tx.usbTxUnderRun++;
            txPreRoll = true;
            return(0);
        }
    }

    /* See if it is time to compute a new buffer fill level.  Using CGU_TS_CLK
     * as the interval will check the buffer level once per second since
     * CGU_TS_CLK is the fundamental clock for the buffer tracker's timestamps.
     */
    sampleRateUpdate = bufferTrackCheck(UAC2_IN_BUFFER_TRACK_IDX, CGU_TS_CLK, &ringFrames);
    if (sampleRateUpdate) {
        /* Compute the average number of frames in the ring buffer over
         * the last tracking interval.
         */
        ringFrames /= context->cfg.usbInChannels;

        /* Compute the error from the desired fill level. This algorithm
         * will only attempt to reduce the error by half during the next
         * tracking interval to help insure stability.
         */
        error = ((int32_t)ringFrames - (int32_t)(targetRingFrames))/2;

        /* If there's an error, compute which usb frames need to have
         * a sample of adjustment.
         */
        if (error) {
            adjustCountThresh = SYSTEM_SAMPLE_RATE / (uacFrames * abs(error));
            adjustValue = adjustCountThresh ? ((error < 0) ? -1 : 1) : 0;
        } else {
            adjustCountThresh = 0;
            adjustValue = 0;
        }
        adjustCountCurr = 0;
    }

    /* Add or subtract the adjustment frame if necessary */
    if (adjustValue && (++adjustCountCurr == adjustCountThresh)) {
        uacFrames += adjustValue;
        adjustCountCurr = 0;
    }

    /* Copy the audio from the ring buffer */
    if (sampleSizeBytes == sizeof(SYSTEM_AUDIO_TYPE)) {
        PaUtil_ReadRingBuffer(
            context->uac2InTx, data, uacFrames * context->cfg.usbInChannels
        );
    } else {
        PaUtil_ReadRingBuffer(
            context->uac2InTx, txBuffer, uacFrames * context->cfg.usbInChannels
        );
        copyAndConvert(
            txBuffer, sizeof(SYSTEM_AUDIO_TYPE), context->cfg.usbInChannels,
            data, sampleSizeBytes, context->cfg.usbInChannels,
            uacFrames, false
        );
    }

    /* Return the size in bytes to the soundcard service */
    size = uacFrames * context->cfg.usbInChannels * sampleSizeBytes;

    /* Track ISR cycle count for CPU load */
    outCycles = cpuLoadGetTimeStamp();
    cpuLoadIsrCycles(outCycles - inCycles);

    return(size);
}

/*
 * This callback is called whenever a sample rate update is requested
 * by the host.  This callback runs in an ISR context.
 */
uint32_t uac2RateFeedback(void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;
    uint32_t inCycles, outCycles;
    uint32_t rate = SYSTEM_SAMPLE_RATE;

    UNUSED(context);

    /* Track ISR cycle count for CPU load */
    inCycles = cpuLoadGetTimeStamp();

    rate = bufferTrackGetSampleRate(UAC2_OUT_BUFFER_TRACK_IDX);
    if (rate == 0) {
        rate = SYSTEM_SAMPLE_RATE;
    }

    /*
     * Older Windows 10 versions must assume the feedback is for a low-speed
     * bInterval of 1mS and do not accept standard feeback values from the CLD driver
     * on high-speed connections when the bInterval is 125uS.  Multiply
     * the rate feedback by 8 to compensate.
     */
     if (context->cfg.usbRateFeedbackHack) {
        rate *= 8;
    }

    /* Track ISR cycle count for CPU load */
    outCycles = cpuLoadGetTimeStamp();
    cpuLoadIsrCycles(outCycles - inCycles);

    return(rate);
}

/*
 * This callback is called whenever the IN or OUT endpoint is enabled
 * or disabled.  This callback runs in an ISR context.
 */
void uac2EndpointEnabled(UAC2_DIR dir, bool enable, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;

    if (dir == UAC2_DIR_OUT) {
        if (enable) {
            memset(&context->uac2stats.rx, 0, sizeof(context->uac2stats.rx));
        } else {
            PaUtil_FlushRingBuffer(context->uac2OutRx);
        }
        context->uac2RxEnabled = enable;
    } else if (dir == UAC2_DIR_IN) {
        if (enable) {
            memset(&context->uac2stats.tx, 0, sizeof(context->uac2stats.tx));
            txPreRoll = true;
        } else {
            PaUtil_FlushRingBuffer(context->uac2InTx);
        }
        context->uac2TxEnabled = enable;
    }
}


/*
 * Transfers the USB Rx Audio (OUT Endpoint)
 *
 * WARNING: The USB stat and data ISRs run at a higher priority than
 *          serviceUsbOutAudio() and will nest.  All calls to bufferTrackXXX()
 *          must be protected in ISR critical sections.
 *
 */
SAE_MSG_BUFFER * xferUsbRxAudio(APP_CONTEXT *context, SAE_MSG_BUFFER *msg,
    CLOCK_DOMAIN cd)
{
    unsigned samples;
    unsigned frames;
    static bool rxPreRoll = true;
    UBaseType_t isrStat;
    IPC_MSG *ipcMsg;
    IPC_MSG_AUDIO *audio;
    CLOCK_DOMAIN myCd;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_USB_RX);
    if (myCd != cd) {
        return(NULL);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_USB_RX);

    ipcMsg = sae_getMsgBufferPayload(msg);
    audio = &ipcMsg->audio;
    audio->clockDomain = myCd;

#if 1
    /* Sanity check the request */
    if ( (audio->numChannels != USB_DEFAULT_OUT_AUDIO_CHANNELS) ||
         (audio->wordSize != sizeof(SYSTEM_AUDIO_TYPE)) )
    {
        return(NULL);
    }
#endif

    samples = PaUtil_GetRingBufferReadAvailable(context->uac2OutRx);
    frames = samples / USB_DEFAULT_OUT_AUDIO_CHANNELS;

    if (rxPreRoll) {
        /* Must have at least USB_OUT_RING_BUFF_FILL of data waiting */
        if (frames >= USB_OUT_RING_BUFF_FILL) {
            isrStat = taskENTER_CRITICAL_FROM_ISR();
            bufferTrackReset(UAC2_OUT_BUFFER_TRACK_IDX);
            taskEXIT_CRITICAL_FROM_ISR(isrStat);
            rxPreRoll = false;
        }
    } else {
        /* If audio is playing and the ring buffer drops below a
         * requested frame of data, restart the pre-roll process
         */
        if (frames < audio->numFrames) {
            isrStat = taskENTER_CRITICAL_FROM_ISR();
            bufferTrackReset(UAC2_OUT_BUFFER_TRACK_IDX);
            taskEXIT_CRITICAL_FROM_ISR(isrStat);
            rxPreRoll = true;
            if (context->uac2RxEnabled) {
                context->uac2stats.rx.usbRxUnderRun++;
            }
        }
    }

    if (!rxPreRoll) {

        bool sampleRateUpdate;

        /* Accumulate the fill level of the USB OUT (Rx) ring buffer
         * for sample rate feedback
         */
        isrStat = taskENTER_CRITICAL_FROM_ISR();
        bufferTrackAccum(UAC2_OUT_BUFFER_TRACK_IDX, samples);

        /* See if it is time to compute a new buffer fill level (once
         * per second @ CGU_TS_CLK).  If so, also compute a new sample rate
         * feedback value.
         */
        sampleRateUpdate = bufferTrackCheck(UAC2_OUT_BUFFER_TRACK_IDX, CGU_TS_CLK, NULL);
        if (sampleRateUpdate) {
            bufferTrackCalculateSampleRate(
                UAC2_OUT_BUFFER_TRACK_IDX,
                USB_OUT_RING_BUFF_FILL,
                USB_DEFAULT_OUT_AUDIO_CHANNELS,
                SYSTEM_SAMPLE_RATE
            );
        }
        taskEXIT_CRITICAL_FROM_ISR(isrStat);

        /* Get a block of USB OUT (Rx) audio from the ring buffer */
        PaUtil_ReadRingBuffer(
            context->uac2OutRx,
            audio->data, audio->numChannels * audio->numFrames
        );

    } else {
        /* Play silence while prerolling */
        memset(
            audio->data,
            0,
            audio->numChannels * audio->numFrames * audio->wordSize
        );
    }

    return(msg);
}

/*
 * Transfers USB Tx audio (IN Endpoint)
 *
 * WARNING: The USB data transfer ISRs run at a higher priority than
 *          serviceUsbIn() and will nest.  All calls to bufferTrackXXX()
 *          must be protected in critical sections.
 *
 */
SAE_MSG_BUFFER *xferUsbTxAudio(APP_CONTEXT *context, SAE_MSG_BUFFER *msg,
    CLOCK_DOMAIN cd)
{
    unsigned samples;
    unsigned framesAvailable;
    UBaseType_t isrStat;
    IPC_MSG *ipcMsg;
    IPC_MSG_AUDIO *audio;
    CLOCK_DOMAIN myCd;

    myCd = clock_domain_get(context, CLOCK_DOMAIN_BITM_USB_TX);
    if (myCd != cd) {
        return(NULL);
    }
    clock_domain_set_active(context, myCd, CLOCK_DOMAIN_BITM_USB_TX);

    ipcMsg = sae_getMsgBufferPayload(msg);
    audio = &ipcMsg->audio;
    audio->clockDomain = myCd;

#if 1
    /* Sanity check the request */
    if ( (audio->numChannels != USB_DEFAULT_IN_AUDIO_CHANNELS) ||
         (audio->wordSize != sizeof(SYSTEM_AUDIO_TYPE)) )
    {
        return(NULL);
    }
#endif

    if (context->uac2TxEnabled) {

        /* Accumulate the fill level of the USB IN (Tx) ring buffer
         * for +/- single frame USB IN adjustments.  Skip this during pre-roll.
         */
        if (!txPreRoll) {
            samples = PaUtil_GetRingBufferReadAvailable(context->uac2InTx);
            isrStat = taskENTER_CRITICAL_FROM_ISR();
            bufferTrackAccum(UAC2_IN_BUFFER_TRACK_IDX, samples);
            taskEXIT_CRITICAL_FROM_ISR(isrStat);
        }

        /* Calculate the number of free USB_IN_AUDIO_CHANNELS sized
         * frames available in the ring buffer.
         */
        samples = PaUtil_GetRingBufferWriteAvailable(context->uac2InTx);
        framesAvailable = samples / USB_DEFAULT_IN_AUDIO_CHANNELS;

        /* Put a block of USB IN (Tx) audio into the ring buffer */
        if (framesAvailable >= audio->numFrames ) {

            PaUtil_WriteRingBuffer(
                context->uac2InTx,
                audio->data, audio->numChannels * audio->numFrames
            );

            /* Notify the UAC2 task that data is available */
            xTaskNotifyFromISR(context->uac2TaskHandle,
                UAC2_TASK_AUDIO_DATA_READY, eSetValueWithoutOverwrite, NULL
            );

        } else {
            context->uac2stats.tx.usbTxOverRun++;
        }
    }

    return(msg);
}


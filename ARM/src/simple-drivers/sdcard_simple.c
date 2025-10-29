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

/*!
 * @brief     Simple, efficient, RTOS or bare metal master mode SDCARD driver
 *
 *   This SDCARD driver supports:
 *     - FreeRTOS or no RTOS main-loop modes
 *     - Fully protected multi-threaded device transfers
 *
 * @file      sdcard_simple.h
 * @version   2.0.0
 * @copyright 2023 Analog Devices, Inc.  All rights reserved.
 *
*/
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#if defined(__ADSPSC598_FAMILY__)
#include "adi_emsi.h"
void invalidate_l1_l2_buffer(void *_start, void *_end);
#define SD_CARD_INVALIDATE(start,end) \
    invalidate_l1_l2_buffer( \
        (void *)((uintptr_t)start & ~(ADI_CACHE_LINE_LENGTH-1)), \
        (void *)((uintptr_t)(end + ADI_CACHE_LINE_LENGTH) & ~(ADI_CACHE_LINE_LENGTH-1)) \
    )
#define ADI_EMSI
#else
#include <drivers/rsi/adi_rsi.h>
#define SD_CARD_INVALIDATE(start,end)
#define ADI_RSI
#endif

#include <sys/platform.h>

#include "sdcard_simple.h"

#ifdef FREE_RTOS
    #include "FreeRTOS.h"
    #include "semphr.h"
    #include "task.h"
    #define SDCARD_ENTER_CRITICAL()  taskENTER_CRITICAL()
    #define SDCARD_EXIT_CRITICAL()   taskEXIT_CRITICAL()
    #define SDCARD_LOCK()            xSemaphoreTake(sdcard->portLock, portMAX_DELAY);
    #define SDCARD_UNLOCK()          xSemaphoreGive(sdcard->portLock);
#else
    #define SDCARD_ENTER_CRITICAL()
    #define SDCARD_EXIT_CRITICAL()
    #define SDCARD_LOCK()
    #define SDCARD_UNLOCK()
#endif

#define NO_RESPONSE    0x00
#define R1_RESPONSE    0x00
#define R1B_RESPONSE   0x00
#define R2_RESPONSE    0x00
#define R3_RESPONSE    0x00
#define R6_RESPONSE    0x00
#define R7_RESPONSE    0x00

#define SD_MMC_CMD0    0
#define SD_MMC_CMD1    1
#define SD_MMC_CMD2    2
#define SD_MMC_CMD3    3
#define SD_MMC_CMD6    6
#define SD_MMC_CMD7    7
#define SD_MMC_CMD8    8
#define SD_MMC_CMD9    9
#define SD_MMC_CMD12   12
#define SD_MMC_CMD17   17
#define SD_MMC_CMD18   18
#define SD_MMC_CMD24   24
#define SD_MMC_CMD25   25
#define SD_MMC_CMD55   55
#define SD_ACMD6       6
#define SD_ACMD41      41
#define SD_ACMD42      42
#define SD_ACMD13      13
#define SD_ACMD51      51

#define SD_MMC_CMD_GO_IDLE_STATE        (SD_MMC_CMD0 | NO_RESPONSE)
#define MMC_CMD_GET_OCR_VALUE           (SD_MMC_CMD1 | R3_RESPONSE)
#define SD_MMC_CMD_ALL_SEND_CID         (SD_MMC_CMD2 | R2_RESPONSE)
#define SD_CMD_SEND_RELATIVE_ADDR       (SD_MMC_CMD3 | R6_RESPONSE)
#define SD_CMD_SWITCH_FUNCTION          (SD_MMC_CMD6 | R1_RESPONSE)
#define SD_MMC_CMD_SELECT_DESELECT_CARD (SD_MMC_CMD7 | R1B_RESPONSE)
#define SD_CMD_SEND_IF_COND             (SD_MMC_CMD8 | R7_RESPONSE)
#define MMC_CMD_SEND_EXT_CSD            (SD_MMC_CMD8 | R1_RESPONSE)
#define SD_MMC_CMD_SEND_CSD             (SD_MMC_CMD9 | R2_RESPONSE)
#define SD_MMC_CMD_STOP_TRANSMISSION    (SD_MMC_CMD12 | R1B_RESPONSE)
#define SD_MMC_CMD_READ_BLOCK           (SD_MMC_CMD17 | R1_RESPONSE)
#define SD_MMC_CMD_READ_MULTIPLE_BLOCK  (SD_MMC_CMD18 | R1_RESPONSE)
#define SD_MMC_CMD_WRITE_BLOCK          (SD_MMC_CMD24 | R1_RESPONSE)
#define SD_MMC_CMD_WRITE_MULTIPLE_BLOCK (SD_MMC_CMD25 | R1_RESPONSE)
#define SD_MMC_CMD_APP_CMD              (SD_MMC_CMD55 | R1_RESPONSE)
#define SD_CMD_SET_BUS_WIDTH            (SD_ACMD6 | R1_RESPONSE)
#define SD_CMD_DISCONNECT_DAT3_PULLUP   (SD_ACMD42 | R1_RESPONSE)
#define SD_CMD_GET_OCR_VALUE            (SD_ACMD41 | R3_RESPONSE)
#define SD_CMD_GET_MEMORY_STATUS        (SD_ACMD13 | R1_RESPONSE)
#define SD_CMD_SEND_SCR                 (SD_ACMD51 | R1_RESPONSE)

/****************************************************
 *  SD/eMMC OCR Register Bit Masks
 ****************************************************/
#define SD_OCR_CARD_CAPACITY_STATUS (1<<30)
#define SD_OCR_CARD_POWER_UP_STATUS (1<<31)

/****************************************************
 *  SD/eMMC Status Bit Masks
 ****************************************************/
#define CARD_STATUS_READY_FOR_DATA  (1 << 8)

/****************************************************
 *  SD CMD6 Bit Masks
 ****************************************************/
#define CMD6_FUNC_GRP_1_OFFSET      (16)
#define CMD6_HIGH_SPEED_SET_OK      (0x01)

/****************************************************
 *  eMMC RCA
 ****************************************************/
#define EMMC_RCA                    (1)

typedef enum _SDCARD_CSD_STRUCTURE {
    SDCARD_CSD_STRUCTURE_VERSION_1_0          = 0,
    SDCARD_CSD_STRUCTURE_VERSION_2_0          = 1,
    SDCARD_CSD_STRUCTURE_VERSION_RESERVED0    = 2,
    SDCARD_CSD_STRUCTURE_VERSION_RESERVED1    = 3
} SDCARD_CSD_STRUCTURE;

/* SD transfer units and multipliers */
const unsigned long SDCARD_TRANSFER_UNIT[7] = {
    10, 100, 1000, 10000, 0, 0, 0
};
const unsigned long SDCARD_TRANSFER_MULTIPLIER[16] = {
    0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80
};

/****************************************************
 * RSI/EMSI driver defines                          *
 ****************************************************/
#ifdef ADI_RSI
typedef ADI_RSI_HANDLE      ADI_xSI_HANDLE;
typedef ADI_RSI_RESULT      ADI_xSI_RESULT;
typedef ADI_RSI_RESPONSE_TYPE ADI_xSI_RESPONSE_TYPE;
typedef ADI_RSI_TRANSFER    ADI_xSI_TRANSFER;
#define ADI_xSI_SUCCESS     (ADI_RSI_SUCCESS)
#define ADI_xSI_FAILURE     (ADI_RSI_FAILURE)
#define TRANS_READ          (ADI_RSI_TRANSFER_DMA_BLCK_READ)
#define TRANS_WRITE         (ADI_RSI_TRANSFER_DMA_BLCK_WRITE)
#define TRANS_NONE          (ADI_RSI_TRANSFER_NONE)
#define RESPONSE_NONE       (ADI_RSI_RESPONSE_TYPE_NONE)
#define RESPONSE_LONG       (ADI_RSI_RESPONSE_TYPE_LONG)
#define RESPONSE_SHORT      (ADI_RSI_RESPONSE_TYPE_SHORT)
#define CRCDIS              (ADI_RSI_CMDFLAG_CRCDIS)
#define CHKBUSY             (ADI_RSI_CMDFLAG_CHKBUSY)
#define ADI_xSI_MAX_TRANSFER_BYTES (ADI_RSI_MAX_TRANSFER_BYTES)
#else
typedef ADI_EMSI_HANDLE        ADI_xSI_HANDLE;
typedef ADI_EMSI_RESULT        ADI_xSI_RESULT;
typedef ADI_EMSI_RESP_TYPE     ADI_xSI_RESPONSE_TYPE;
typedef ADI_EMSI_GENERAL_TRANS_TYPE ADI_xSI_TRANSFER;
#define ADI_xSI_SUCCESS        ADI_EMSI_SUCCESS
#define ADI_xSI_FAILURE        ADI_EMSI_FAILURE
#define TRANS_READ             (ADI_EMSI_GENERAL_TRANS_TYPE_READ)
#define TRANS_WRITE            (ADI_EMSI_GENERAL_TRANS_TYPE_WRITE)
#define TRANS_NONE             (-1)
#define RESPONSE_NONE          (ADI_EMSI_RESPONSE_TYPE_NONE)
#define RESPONSE_LONG          (ADI_EMSI_RESPONSE_TYPE_LONG)
#define RESPONSE_SHORT         (ADI_EMSI_RESPONSE_TYPE_SHORT)
#define CRCDIS                 (1)
#define CHKBUSY                (2)
#define ADI_xSI_MAX_TRANSFER_BYTES (65536)
#endif

#define MAX_xSI_TRANSFER_COUNT (ADI_xSI_MAX_TRANSFER_BYTES / 512)

struct sSDCARD {
    uint8_t alignedData[MAX_xSI_TRANSFER_COUNT * 512];
#ifdef ADI_EMSI
    uint8_t emsiMemory[ADI_EMSI_DRIVER_MEMORY_SIZE];
    ADI_xSI_RESULT cardPresent;
#endif

    bool open;
    bool started;
    bool taskWaiting;
    bool cmdTimeout;

    ADI_xSI_HANDLE xsiHandle;

#ifdef FREE_RTOS
    SemaphoreHandle_t portLock;
    SemaphoreHandle_t cmdBlock;
#else
    volatile bool sdcardDone;
#endif

    SDCARD_SIMPLE_TYPE type;
    SDCARD_SIMPLE_CARD card;
    uint32_t rca;
    uint64_t capacity;
    uint32_t speed;

    SDCARD_CSD_STRUCTURE csdStruct;
};


/* SDCARD port context containers */
__attribute__ ((aligned (ADI_CACHE_LINE_LENGTH)))
static sSDCARD sdcardContext[SDCARD_END];

static void sdcard_event(void *usr, uint32_t event, void *arg)
{
    sSDCARD *sdcard = (sSDCARD *)usr;
    BaseType_t contextSwitch = pdFALSE;
    bool unblock = false;
#ifdef ADI_RSI
    switch (event) {
        case ADI_RSI_EVENT_INTERRUPT:
        {
            uint32_t interrupts = *((uint32_t *)arg);
            if (interrupts & BITM_MSI_MSKISTAT_CMDDONE) {
                unblock = true;
            }
            break;
        }
        case ADI_RSI_EVENT_CARD_CHANGE:
        {
            int event = *((int *)arg);
            if (event == 1) {
                /* Inserted */
            } else if (event == 0) {
                /* Removed */
            } else {
                /* Unknown */
            }
            break;
        }
        default:
            break;
    }
#else
    if (event & (1 << ADI_EMSI_EVENT_CARD_INSERTION)) {
        sdcard->cardPresent = ADI_xSI_SUCCESS;
    }
    if (event & (1 << ADI_EMSI_EVENT_CARD_REMOVAL)) {
        sdcard->cardPresent = ADI_xSI_FAILURE;
    }
    if (event & (1 << ADI_EMSI_EVENT_CMD_TOUT_ERR)) {
        sdcard->cmdTimeout = true;
        unblock = true;
    }
    if (event & (1 << ADI_EMSI_EVENT_CMD_COMPLETE)) {
        unblock = true;
    }
    if (event & (1 << ADI_EMSI_EVENT_XFER_COMPLETE)) {
        unblock = true;
    }
#endif

    if (unblock) {
#ifdef FREE_RTOS
        if (sdcard->taskWaiting) {
            sdcard->taskWaiting = false;
            xSemaphoreGiveFromISR(sdcard->cmdBlock, &contextSwitch);
            if (contextSwitch == pdTRUE) {
                portYIELD_FROM_ISR(contextSwitch);
            }
        }
#else
        sdcard->sdcardDone = true;
#endif
    }
}

static ADI_xSI_RESULT sdcard_block(sSDCARD *sdcard, unsigned timeout)
{
    ADI_xSI_RESULT result = ADI_xSI_SUCCESS;
#ifdef FREE_RTOS
    BaseType_t rtosResult;
    rtosResult = xSemaphoreTake(sdcard->cmdBlock, pdMS_TO_TICKS(timeout));
    if (rtosResult != pdTRUE) {
        result = ADI_xSI_FAILURE;
    }
#else
    /* TODO: Timeout */
    while (sdcard->sdcardDone == false);
#endif
    return(result);
}


/***********************************************************************
 * Send Command
 ***********************************************************************/
static ADI_xSI_RESULT sdcard_SendCommand(sSDCARD *sdcard,
    uint16_t command, uint32_t argument,
    ADI_xSI_RESPONSE_TYPE resp, ADI_xSI_TRANSFER transfer, uint32_t flags)
{
    ADI_xSI_HANDLE xsiHandle = sdcard->xsiHandle;
    ADI_xSI_RESULT result = ADI_xSI_SUCCESS;

#ifdef ADI_RSI
    result = adi_rsi_SetDataMode(xsiHandle, transfer, ADI_RSI_CEATA_MODE_NONE);
    sdcard->taskWaiting = true;
    result = adi_rsi_SendCommand(xsiHandle, command, argument, flags, resp);
    result = sdcard_block(sdcard, 1000);
#else
    ADI_EMSI_CMD_PARA cmdParam = {
        .eRespType = resp,
        .eCmdCrcCheck = (flags & CRCDIS) ?
            ADI_EMSI_CMD_CRC_CHECK_DISABLE : ADI_EMSI_CMD_CRC_CHECK_ENABLE,
        .eDataPresent = ((transfer == TRANS_READ) || (transfer == TRANS_WRITE)) ?
            ADI_EMSI_DATA_PRESENT_TRUE : ADI_EMSI_DATA_PRESENT_FALSE
    };
    result = adi_emsi_SendCommand(xsiHandle, command, argument, &cmdParam);
#endif

    return result;
}

/***********************************************************************
 * Get Response
 ***********************************************************************/
static ADI_xSI_RESULT sdcard_GetResponse(sSDCARD *sdcard, uint32_t *response,
    ADI_xSI_RESPONSE_TYPE type)
{
    ADI_xSI_HANDLE xsiHandle = sdcard->xsiHandle;
    ADI_xSI_RESULT result = ADI_xSI_SUCCESS;

#ifdef ADI_RSI
    if (type == RESPONSE_SHORT) {
        result = adi_rsi_GetShortResponse(xsiHandle, response);
    } else {
        result = adi_rsi_GetLongResponse(xsiHandle, response);
    }
#else
    uint32_t resp[4];
    result = adi_emsi_GetResponse(xsiHandle, resp);
    if (result == ADI_xSI_SUCCESS) {
        if (type == RESPONSE_SHORT) {
            *response = resp[0];
        } else {
            /* Rearrange EMSI response to look like RSI driver */
            *response++ = resp[3] << 8 | ((resp[2] >> 24) & 0xFF);
            *response++ = resp[2] << 8 | ((resp[1] >> 24) & 0xFF);
            *response++ = resp[1] << 8 | ((resp[0] >> 24) & 0xFF);
            *response = resp[0] << 8;
        }
    }
#endif
    return(result);
}

SDCARD_SIMPLE_RESULT sdcard_present(sSDCARD *sdcard)
{
    SDCARD_SIMPLE_RESULT result = SDCARD_SIMPLE_SUCCESS;
    ADI_xSI_RESULT xsiResult = ADI_xSI_SUCCESS;

    SDCARD_LOCK();

#ifdef ADI_RSI
    xsiResult = adi_rsi_IsCardPresent(sdcard->xsiHandle);
#else
#ifdef SDCARD_POLL_CARD_DETECT
    if (sdcard->card == SDCARD_CARD_TYPE_SD) {
        if (sdcard->started) {
            xsiResult = sdcard_SendCommand(sdcard,
                SD_CMD_GET_MEMORY_STATUS, sdcard->rca, RESPONSE_SHORT, TRANS_NONE, CRCDIS
            );
            if ((xsiResult == ADI_EMSI_TIMED_OUT) || (sdcard->cmdTimeout)) {
                xsiResult = ADI_xSI_FAILURE;
            }
        } else {
            sdcard_SendCommand(sdcard,
                SD_MMC_CMD_GO_IDLE_STATE, 0, RESPONSE_NONE, TRANS_NONE, 0
            );
            sdcard->cmdTimeout = false;
            sdcard_SendCommand(sdcard,
                SD_CMD_SEND_IF_COND, 0x000001AA, RESPONSE_SHORT, TRANS_NONE, CRCDIS
            );
            sdcard_block(sdcard, 100);
        }
        if (sdcard->cmdTimeout) {
            xsiResult = ADI_xSI_FAILURE;
        }
    } else {
        xsiResult = sdcard->cardPresent;
    }
#else
    xsiResult = sdcard->cardPresent;
#endif
#endif

    SDCARD_UNLOCK();

    if (xsiResult != ADI_xSI_SUCCESS) {
        result = SDCARD_SIMPLE_ERROR;
    }

    return(result);
}

/***********************************************************************
 * Ready for data
 ***********************************************************************/
SDCARD_SIMPLE_RESULT sdcard_readyForData(sSDCARD *sdcard)
{
    SDCARD_SIMPLE_RESULT result = SDCARD_SIMPLE_SUCCESS;
    ADI_xSI_RESULT xsiResult = ADI_xSI_SUCCESS;
    uint32_t response = 0;

    if ((sdcard == NULL) || (sdcard->type == SDCARD_UNUSABLE_CARD)) {
        return(SDCARD_SIMPLE_ERROR);
    }

    /* TODO: 1 second timeout */
    while (1) {
        xsiResult = sdcard_SendCommand(sdcard,
            SD_CMD_GET_MEMORY_STATUS, sdcard->rca, RESPONSE_SHORT, TRANS_NONE, CRCDIS
        );
        if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }

        xsiResult = sdcard_GetResponse(sdcard, &response, RESPONSE_SHORT);
        if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }

        if (response & CARD_STATUS_READY_FOR_DATA) {
            break;
        }
    }

abort:
    if (xsiResult != ADI_xSI_SUCCESS) {
        result = SDCARD_SIMPLE_ERROR;
    }

    return(result);
}

/***********************************************************************
 * Write
 ***********************************************************************/
static SDCARD_SIMPLE_RESULT
sdcard_submitDataTransfer(sSDCARD *sdcard,
    uint16_t cmd, bool read, void *data, uint32_t sector, uint32_t count,
    uint32_t length, bool checkReady)
{
    SDCARD_SIMPLE_RESULT result = SDCARD_SIMPLE_SUCCESS;
    ADI_xSI_HANDLE xsiHandle = sdcard->xsiHandle;
    ADI_xSI_RESULT xsiResult = ADI_xSI_SUCCESS;
    ADI_xSI_TRANSFER transfer = 0;

    if (checkReady) {
        result = sdcard_readyForData(sdcard);
        if (result != SDCARD_SIMPLE_SUCCESS) {
            goto abort;
        }
    }

#ifdef ADI_RSI
    void *buf = NULL;
    xsiResult = adi_rsi_SetBlockCntAndLen(xsiHandle, count, length);
    if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
    if (read) {
        xsiResult = adi_rsi_SubmitRxBuffer(xsiHandle, data, length, count);
        transfer = TRANS_READ;
    } else {
        xsiResult = adi_rsi_SubmitTxBuffer(xsiHandle, data, length, count);
        transfer = TRANS_WRITE;
    }
    xsiResult = sdcard_SendCommand(sdcard,
        cmd, sector, RESPONSE_SHORT, transfer, CRCDIS
    );
    if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
    if (read) {
        xsiResult = adi_rsi_GetRxBuffer(xsiHandle, &buf);
    } else {
        xsiResult = adi_rsi_GetTxBuffer(xsiHandle, &buf);
    }
#else
    xsiResult = adi_emsi_SetBlkSze(sdcard->xsiHandle, length);
    transfer = read ? TRANS_READ : TRANS_WRITE;
    xsiResult = adi_emsi_General_Transfer(
        xsiHandle, transfer, count, data
    );
    sdcard->taskWaiting = true;
    xsiResult = sdcard_SendCommand(sdcard,
        cmd, sector, RESPONSE_SHORT, transfer, CRCDIS
    );
    xsiResult = sdcard_block(sdcard, 1000);
    if (read && (xsiResult == ADI_xSI_SUCCESS)) {
        SD_CARD_INVALIDATE(data, (uintptr_t)data + 512 * count);
    }
#endif

abort:
    if (xsiResult != ADI_xSI_SUCCESS) {
        result = SDCARD_SIMPLE_ERROR;
    }

    return(result);
}

SDCARD_SIMPLE_RESULT sdcard_write(sSDCARD *sdcard, void *data, uint32_t sector, uint32_t count)
{
    SDCARD_SIMPLE_RESULT result = SDCARD_SIMPLE_SUCCESS;
    ADI_xSI_RESULT xsiResult = ADI_xSI_SUCCESS;
    uint32_t transferCount;
    uint8_t *dataPtr;
    uint8_t *outData;
    uint16_t cmd;
    bool aligned;

    if ((sdcard == NULL) || (sdcard->type == SDCARD_UNUSABLE_CARD)) {
        return(SDCARD_SIMPLE_ERROR);
    }

    if (sdcard->type != SDCARD_TYPE_SD_V2X_HIGH_CAPACITY) {
        sector *= 512;
    }

    /* Must send unaligned data through an aligned buffer */
    aligned = ((uintptr_t)data & (ADI_CACHE_LINE_LENGTH - 1)) == 0;

    SDCARD_LOCK();

    dataPtr = (uint8_t *)data;
    do {

        transferCount = (count > MAX_xSI_TRANSFER_COUNT) ?
            MAX_xSI_TRANSFER_COUNT : count;

        if (aligned) {
            outData = dataPtr;
        } else {
            memcpy(sdcard->alignedData, dataPtr, transferCount * 512);
            outData = sdcard->alignedData;
        }

        cmd = (transferCount > 1) ?
            SD_MMC_CMD_WRITE_MULTIPLE_BLOCK : SD_MMC_CMD_WRITE_BLOCK;

        result = sdcard_submitDataTransfer(sdcard,
            cmd, false, outData, sector, transferCount, 512, true
        );
        if (result != SDCARD_SIMPLE_SUCCESS) { goto abort; }

        if (transferCount > 1) {
            xsiResult = sdcard_SendCommand(sdcard,
                SD_MMC_CMD_STOP_TRANSMISSION, 0, RESPONSE_SHORT, TRANS_NONE, CRCDIS
            );
            if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
        }

        count -= transferCount; sector += transferCount;
        dataPtr += transferCount * 512;

    } while (count);

abort:
    SDCARD_UNLOCK();
    if (xsiResult != ADI_xSI_SUCCESS) {
        result = SDCARD_SIMPLE_ERROR;
    }

    return(result);
}

/***********************************************************************
 * Read
 ***********************************************************************/
SDCARD_SIMPLE_RESULT sdcard_read(sSDCARD *sdcard, void *data, uint32_t sector, uint32_t count)
{
    SDCARD_SIMPLE_RESULT result = SDCARD_SIMPLE_SUCCESS;
    ADI_xSI_RESULT xsiResult = ADI_xSI_SUCCESS;
    uint32_t transferCount;
    uint8_t *dataPtr;
    uint8_t *inData;
    uint16_t cmd;
    bool aligned;

    if ((sdcard == NULL) || (sdcard->type == SDCARD_UNUSABLE_CARD)) {
        return(SDCARD_SIMPLE_ERROR);
    }

    if (sdcard->type != SDCARD_TYPE_SD_V2X_HIGH_CAPACITY) {
        sector *= 512;
    }

    /* Must send unaligned data through an aligned buffer */
    aligned = ((uintptr_t)data & (ADI_CACHE_LINE_LENGTH - 1)) == 0;

    SDCARD_LOCK();

    dataPtr = (uint8_t *)data;
    do {

        transferCount = (count > MAX_xSI_TRANSFER_COUNT) ?
            MAX_xSI_TRANSFER_COUNT : count;

        if (aligned) {
            inData = dataPtr;
        } else {
            inData = sdcard->alignedData;
        }

        cmd = (transferCount > 1) ?
            SD_MMC_CMD_READ_MULTIPLE_BLOCK : SD_MMC_CMD_READ_BLOCK;

        result = sdcard_submitDataTransfer(sdcard,
            cmd, true, inData, sector, transferCount, 512, true
        );
        if (result != SDCARD_SIMPLE_SUCCESS) { goto abort; }

        if (transferCount > 1) {
            xsiResult = sdcard_SendCommand(sdcard,
                SD_MMC_CMD_STOP_TRANSMISSION, 0, RESPONSE_SHORT, TRANS_NONE, CRCDIS
            );
            if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
        }

        if (!aligned) {
            memcpy(dataPtr, sdcard->alignedData, transferCount * 512);
        }

        count -= transferCount; sector += transferCount;
        dataPtr += transferCount * 512;

    } while (count);

abort:
    SDCARD_UNLOCK();

    if (xsiResult != ADI_xSI_SUCCESS) {
        result = SDCARD_SIMPLE_ERROR;
    }

    return(result);
}

/***********************************************************************
 * Identify / Init
 ***********************************************************************/
static void sdcard_SetSpeed(sSDCARD *sdcard,
    uint32_t sclk, uint32_t cardSpeed)
{
    ADI_xSI_HANDLE xsiHandle = sdcard->xsiHandle;
    uint32_t clkDiv = 0;
    uint32_t rsiFreq;

    rsiFreq = sclk;
    while(rsiFreq > cardSpeed) {
        clkDiv++;
        rsiFreq = sclk / (clkDiv * 2);
    }
#ifdef ADI_RSI
    if (clkDiv == 0) {
        adi_rsi_SetClock(xsiHandle, 1, ADI_RSI_CLK_MODE_ENABLE);
    } else {
        adi_rsi_SetClock(xsiHandle, clkDiv * 2, ADI_RSI_CLK_MODE_ENABLE);
    }
#else
    adi_emsi_ChangeClockFreq(xsiHandle, clkDiv);
#endif
}

static void sdcard_SetIdentifyMode(sSDCARD *sdcard)
{
    ADI_xSI_HANDLE xsiHandle = sdcard->xsiHandle;

    /* Set speed to 400KHz for identification */
    sdcard_SetSpeed(sdcard, SDCLK, 400000);

    /* Set 1 bit bus width transfer for identification */
#ifdef ADI_RSI
    adi_rsi_SetBusWidth(xsiHandle, 1);
#else
    adi_emsi_SetBusWidth(xsiHandle, ADI_EMSI_BUS_WIDTH_1BIT);
#endif
}

/*
 *  Verifies if the device is an SD Version 2.0 or later device based on the response
 */
static SDCARD_SIMPLE_TYPE sdcard_IdentifyV2(sSDCARD *sdcard, uint32_t response)
{
    ADI_xSI_RESULT result;
    uint32_t ocr = 0x00000000;

    SDCARD_SIMPLE_TYPE type = SDCARD_UNUSABLE_CARD;

    if ((response & 0x000001FF) == 0x000001AA) {
        /* TODO: 1 second timeout */
        do {
            if (sdcard->card == SDCARD_CARD_TYPE_SD) {
                result = sdcard_SendCommand(sdcard,
                    SD_MMC_CMD_APP_CMD, 0, RESPONSE_SHORT, TRANS_NONE, CRCDIS
                );
                result = sdcard_GetResponse(sdcard, &response, RESPONSE_SHORT);
                result = sdcard_SendCommand(sdcard,
                    SD_CMD_GET_OCR_VALUE, 0x40FF8000, RESPONSE_SHORT, TRANS_NONE, CRCDIS
                );
                result = sdcard_GetResponse(sdcard, &ocr, RESPONSE_SHORT);
            } else {
                result = sdcard_SendCommand(sdcard,
                    MMC_CMD_GET_OCR_VALUE, 0x40FF8000, RESPONSE_SHORT, TRANS_NONE, CRCDIS
                );
                result = sdcard_GetResponse(sdcard, &ocr, RESPONSE_SHORT);
            }
        } while ((ocr & SD_OCR_CARD_POWER_UP_STATUS) == 0);

        if (ocr & SD_OCR_CARD_CAPACITY_STATUS) {
            type = SDCARD_TYPE_SD_V2X_HIGH_CAPACITY;
        } else {
            type = SDCARD_TYPE_SD_V2X;
        }
    }

    return type;
}

static SDCARD_SIMPLE_RESULT sdcard_Start(sSDCARD *sdcard)
{
    SDCARD_SIMPLE_RESULT result = SDCARD_SIMPLE_SUCCESS;
    ADI_xSI_HANDLE xsiHandle = sdcard->xsiHandle;
    ADI_xSI_RESULT xsiResult = ADI_xSI_SUCCESS;
    uint8_t *extCSD = sdcard->alignedData;
    uint32_t cardResp = 0;
    uint8_t cmd6Resp[64] = { 0 };
    uint8_t scr[8] = { 0 };
    uint32_t cid[4] = { 0 };
    uint32_t csd[4] = { 0 };

    /* Reset card info */
    sdcard->type = SDCARD_UNUSABLE_CARD;
    sdcard->capacity = 0;

    /* Set initialize phy mode */
    sdcard_SetIdentifyMode(sdcard);

    /* Put card in idle state (reset) */
    xsiResult = sdcard_SendCommand(sdcard,
        SD_MMC_CMD_GO_IDLE_STATE, 0, RESPONSE_NONE, TRANS_NONE, 0
    );

    /* Check SDv2/eMMC voltage range */
    if (sdcard->card == SDCARD_CARD_TYPE_SD) {
        xsiResult = sdcard_SendCommand(sdcard,
            SD_CMD_SEND_IF_COND, 0x000001AA, RESPONSE_SHORT, TRANS_NONE, CRCDIS
        );
        if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
        xsiResult = sdcard_GetResponse(sdcard, &cardResp, RESPONSE_SHORT);
    } else {
        cardResp = 0x000001AA;
    }

    /* SDv2 card */
    sdcard->type = sdcard_IdentifyV2(sdcard, cardResp);
    if (sdcard->type == SDCARD_UNUSABLE_CARD) {
        xsiResult = ADI_xSI_FAILURE;
        goto abort;
    }

    /* Get CID */
    xsiResult = sdcard_SendCommand(sdcard,
        SD_MMC_CMD_ALL_SEND_CID, 0, RESPONSE_LONG, TRANS_NONE, 0
    );
    if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
    xsiResult = sdcard_GetResponse(sdcard, cid, RESPONSE_LONG);

    /* Get SD RCA / Set MMC RCA */
    if (sdcard->card == SDCARD_CARD_TYPE_SD) {
        xsiResult = sdcard_SendCommand(sdcard,
            SD_CMD_SEND_RELATIVE_ADDR, 0, RESPONSE_SHORT, TRANS_NONE, 0
        );
        if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
        xsiResult = sdcard_GetResponse(sdcard, &sdcard->rca, RESPONSE_SHORT);
        sdcard->rca &= 0xFFFF0000;
    } else {
        sdcard->rca = EMMC_RCA << 16;
        xsiResult = sdcard_SendCommand(sdcard,
            SD_CMD_SEND_RELATIVE_ADDR, sdcard->rca, RESPONSE_SHORT, TRANS_NONE, 0
        );
        if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
    }

    /* Get CSD */
    xsiResult = sdcard_SendCommand(sdcard,
        SD_MMC_CMD_SEND_CSD, sdcard->rca, RESPONSE_LONG, TRANS_NONE, CRCDIS
    );
    if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
    xsiResult = sdcard_GetResponse(sdcard, csd, RESPONSE_LONG);

    /* Calculate the default max clock speed */
    uint8_t tran_speed = csd[0] & 0x000000FF;
    uint32_t unit, mul, maxClock;
    unit = SDCARD_TRANSFER_UNIT[tran_speed & 0x7];
    mul = SDCARD_TRANSFER_MULTIPLIER[(tran_speed >> 3) & 0xF];
    maxClock = unit * mul * 1000;

    /* Configure the card interface */
    if (sdcard->card == SDCARD_CARD_TYPE_SD) {

        /* Select the card */
        xsiResult = sdcard_SendCommand(sdcard,
            SD_MMC_CMD_SELECT_DESELECT_CARD, sdcard->rca, RESPONSE_SHORT, TRANS_NONE, CRCDIS
        );
        if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }

        /* Get SCR register */
        xsiResult = sdcard_SendCommand(sdcard,
            SD_MMC_CMD_APP_CMD, sdcard->rca, RESPONSE_SHORT, TRANS_NONE, CRCDIS
        );
        if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
        xsiResult = sdcard_submitDataTransfer(sdcard,
            SD_CMD_SEND_SCR, true, scr, 0x00000000, 1, 8, false
        );
        if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }

        /* Increase speed (CMD6 valid for all v1.10+ cards) */
        if (SDCLK_MAX > maxClock) {
            /* Confirm card spec >= 1.10 */
            if ((scr[0] & 0x0F) >= 1) {
                /* Set high-speed mode in function group 1 */
                xsiResult = sdcard_submitDataTransfer(sdcard,
                    SD_CMD_SWITCH_FUNCTION, true, cmd6Resp, 0x80fffff1, 1, 64, true);
                if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
                /* Confirm function executed */
                if (cmd6Resp[CMD6_FUNC_GRP_1_OFFSET] & CMD6_HIGH_SPEED_SET_OK) {
                    maxClock = 50000000;
                }
            }
        }
        maxClock = (maxClock > SDCLK_MAX) ? SDCLK_MAX : maxClock;
        sdcard_SetSpeed(sdcard, SDCLK, maxClock);

        /* Set 4-bit bus width */
        if (scr[1] & 0x04) {
            xsiResult = sdcard_SendCommand(sdcard,
                SD_MMC_CMD_APP_CMD, sdcard->rca, RESPONSE_SHORT, TRANS_NONE, CRCDIS
            );
            if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
            xsiResult = sdcard_SendCommand(sdcard,
                SD_CMD_DISCONNECT_DAT3_PULLUP, 0, RESPONSE_SHORT, TRANS_NONE, 0
            );
            if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
            xsiResult = sdcard_SendCommand(sdcard,
                SD_MMC_CMD_APP_CMD, sdcard->rca, RESPONSE_SHORT, TRANS_NONE, CRCDIS
            );
            if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
            xsiResult = sdcard_SendCommand(sdcard,
                SD_CMD_SET_BUS_WIDTH, 2, RESPONSE_SHORT, TRANS_NONE, CRCDIS
            );
            if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
#ifdef ADI_RSI
            adi_rsi_SetBusWidth(xsiHandle, 4);
#else
            adi_emsi_SetBusWidth(xsiHandle, ADI_EMSI_BUS_WIDTH_4BIT);
#endif
        }
    } else {
#ifdef ADI_RSI
        xsiResult = ADI_RSI_FAILURE; goto abort;
#else
        xsiResult = adi_emsi_SetAppUse(sdcard->xsiHandle,
            ADI_EMSI_APP_USE_NORMAL_DATATRANSFER, false);
        xsiResult = adi_emsi_SetRca(sdcard->xsiHandle, sdcard->rca >> 16);
        xsiResult = adi_emsi_SelectCard(sdcard->xsiHandle, 1);
        xsiResult = adi_emsi_SetSpeedMode(sdcard->xsiHandle,
            ADI_EMSI_SPEED_MODE_LEGACY_OR_DSSDR);
        maxClock = (maxClock > EMMCCLK_MAX) ? EMMCCLK_MAX : maxClock;
        sdcard_SetSpeed(sdcard, SDCLK, maxClock);
        xsiResult = adi_emsi_SetBusWidth(sdcard->xsiHandle,
            ADI_EMSI_BUS_WIDTH_4BIT);
        xsiResult = adi_emsi_SetBlkSze(sdcard->xsiHandle, 512);
#endif
    }

    /* Calculate the size */
    if (sdcard->card == SDCARD_CARD_TYPE_SD) {
        sdcard->csdStruct = (csd[0] & 0xC0000000) >> 30;
        if (sdcard->csdStruct == SDCARD_CSD_STRUCTURE_VERSION_1_0) {
            uint32_t read_bl_len = (csd[1] & 0x000F0000) >> 16;
            uint32_t c_size = ((csd[1] & 0x000003FF) << 2) | ((csd[2] & 0xC0000000) >> 30);
            uint32_t c_size_mult = (csd[2] & 0x00038000) >> 15;
            uint64_t mult, block_nr, block_len;
            if (sdcard->card == SDCARD_CARD_TYPE_SD) {
                mult = c_size_mult << 8;
                block_nr = (c_size + 1) * mult;
                block_len = read_bl_len << 12;
                sdcard->capacity = mult * block_nr * block_len;
            } else {
                mult = (1 << (c_size_mult + 2));
                block_nr = (c_size + 1) * mult;
                block_len = (1 << read_bl_len);
                sdcard->capacity = block_nr * block_len;
            }
        } else if (sdcard->csdStruct == SDCARD_CSD_STRUCTURE_VERSION_2_0) {
            uint64_t c_size;
            c_size = ((csd[1] & 0x0000003F) << 16) | ((csd[2] & 0xFFFF0000) >> 16);
            sdcard->capacity = (c_size + 1) * 512 * 1024;
        } else {
            sdcard->type = SDCARD_UNUSABLE_CARD;
            goto abort;
        }
    } else {
        uint32_t *sector_count;
        xsiResult = sdcard_submitDataTransfer(
            sdcard, MMC_CMD_SEND_EXT_CSD, true, extCSD, 0, 1, 512, true);
        if (xsiResult != ADI_xSI_SUCCESS) { goto abort; }
        sector_count = (uint32_t *)&extCSD[212];
        sdcard->capacity = (uint64_t)(*sector_count) * 512;
    }

    /* Save the max speed */
    sdcard->speed = maxClock;

abort:
    if ((xsiResult != ADI_xSI_SUCCESS) ||
        (sdcard->type == SDCARD_UNUSABLE_CARD)) {
        result = SDCARD_SIMPLE_ERROR;
        sdcard->type = SDCARD_UNUSABLE_CARD;
        sdcard->capacity = 0;
    }
    return(result);
}

/***********************************************************************
 * Start/Stop Functions
 ***********************************************************************/
SDCARD_SIMPLE_RESULT sdcard_start(sSDCARD *sdcard)
{
    SDCARD_SIMPLE_RESULT result = SDCARD_SIMPLE_SUCCESS;

    if (sdcard == NULL) {
        return(SDCARD_SIMPLE_ERROR);
    }

    SDCARD_LOCK();
    result = sdcard_Start(sdcard);
    if (result == SDCARD_SIMPLE_SUCCESS) {
        sdcard->started = true;
    }
    SDCARD_UNLOCK();

    return(result);
}

SDCARD_SIMPLE_RESULT sdcard_stop(sSDCARD *sdcard)
{
    SDCARD_SIMPLE_RESULT result = SDCARD_SIMPLE_SUCCESS;

    if (sdcard == NULL) {
        return(SDCARD_SIMPLE_ERROR);
    }

    SDCARD_LOCK();
    sdcard->type = SDCARD_UNUSABLE_CARD;
    sdcard->capacity = 0;
    sdcard->started = false;
    sdcard_SetIdentifyMode(sdcard);
    SDCARD_UNLOCK();

    return(result);
}

/***********************************************************************
 * Info Function
 ***********************************************************************/
SDCARD_SIMPLE_RESULT sdcard_info(sSDCARD *sdcard, SDCARD_SIMPLE_INFO *info)
{
    SDCARD_SIMPLE_RESULT result = SDCARD_SIMPLE_SUCCESS;

    if (sdcard == NULL) {
        return(SDCARD_SIMPLE_ERROR);
    }

    SDCARD_LOCK();
    if (info) {
        info->card = sdcard->card;
        info->type = sdcard->type;
        info->capacity = sdcard->capacity;
        info->speed = sdcard->speed;
    }
    SDCARD_UNLOCK();
    return(result);
}

/***********************************************************************
 * Init/Deinit Functions
 ***********************************************************************/
SDCARD_SIMPLE_RESULT sdcard_init(void)
{
    SDCARD_SIMPLE_RESULT result = SDCARD_SIMPLE_SUCCESS;
    uint8_t port;
    sSDCARD *sdcard;

    memset(sdcardContext, 0, sizeof(sdcardContext));

    for (port = SDCARD0; port < SDCARD_END; port++) {

        sdcard = &sdcardContext[port];

#ifdef FREE_RTOS
        sdcard->portLock = xSemaphoreCreateMutex();
        if (sdcard->portLock == NULL) {
            result = SDCARD_SIMPLE_ERROR;
        }
        sdcard->cmdBlock = xSemaphoreCreateCounting(1, 0);
        if (sdcard->cmdBlock == NULL) {
            result = SDCARD_SIMPLE_ERROR;
        }
#endif

        sdcard->open = false;

    }

    return(result);
}

SDCARD_SIMPLE_RESULT sdcard_deinit(void)
{
    SDCARD_SIMPLE_RESULT result = SDCARD_SIMPLE_SUCCESS;
    uint8_t port;
    sSDCARD *sdcard;

    for (port = SDCARD0; port < SDCARD_END; port++) {

        sdcard = &sdcardContext[port];

#ifdef FREE_RTOS
        if (sdcard->cmdBlock) {
            vSemaphoreDelete(sdcard->cmdBlock);
            sdcard->cmdBlock = NULL;
        }
        if (sdcard->portLock) {
            vSemaphoreDelete(sdcard->portLock);
            sdcard->portLock = NULL;
        }
#endif

    }

    return(result);
}

/***********************************************************************
 * Open/Close Functions
 ***********************************************************************/
SDCARD_SIMPLE_RESULT sdcard_open(SDCARD_SIMPLE_PORT port,
    SDCARD_SIMPLE_CARD card, sSDCARD **sdcardHandle)
{
    SDCARD_SIMPLE_RESULT result = SDCARD_SIMPLE_SUCCESS;
    ADI_xSI_RESULT xsiResult;
    sSDCARD *sdcard;

    if (port >= SDCARD_END) {
        return(SDCARD_SIMPLE_INVALID_PORT);
    }

    sdcard = &sdcardContext[port];

    SDCARD_LOCK();

    if (sdcard->open == true) {
        result = SDCARD_SIMPLE_PORT_BUSY;
    }

    if (result == SDCARD_SIMPLE_SUCCESS) {
        sdcard->card = card;
#ifdef ADI_RSI
        if (card == SDCARD_CARD_TYPE_EMMC) {
            result = SDCARD_SIMPLE_ERROR;
        }
        if (result == SDCARD_SIMPLE_SUCCESS) {
            xsiResult = adi_rsi_Open(port, &sdcard->xsiHandle);
            if (xsiResult != ADI_xSI_SUCCESS) {
                result = SDCARD_SIMPLE_ERROR;
            }
        }
#else
        xsiResult = adi_emsi_Open(
            port,
            sdcard->card == SDCARD_CARD_TYPE_EMMC ?
                ADI_EMSI_CARD_TYPE_EMMC : ADI_EMSI_CARD_TYPE_SDCARD,
            sdcard->emsiMemory, sizeof(sdcard->emsiMemory),
            &sdcard->xsiHandle
        );
        if (xsiResult != ADI_xSI_SUCCESS) {
            result = SDCARD_SIMPLE_ERROR;
        }
#endif

        if (result == SDCARD_SIMPLE_SUCCESS) {
#ifdef ADI_RSI
            xsiResult = adi_rsi_RegisterCallback(sdcard->xsiHandle,
                sdcard_event, sdcard);
            xsiResult = adi_rsi_UnmaskInterrupts(sdcard->xsiHandle,
                BITM_MSI_MSKISTAT_CMDDONE);
            xsiResult = adi_rsi_UnmaskCardEvents (sdcard->xsiHandle,
                ADI_RSI_CARD_INSERTION | ADI_RSI_CARD_REMOVAL);
#else
            xsiResult = adi_emsi_RegisterCallback(
                sdcard->xsiHandle, sdcard_event, sdcard);
            /* This generally keeps the EMSI driver from sending commands to the card */
            xsiResult = adi_emsi_SetAppUse(
                sdcard->xsiHandle, ADI_EMSI_APP_USE_BOOTING_OPERATION, false);
            adi_emsi_SetSpeedMode(
                sdcard->xsiHandle, ADI_EMSI_SPEED_MODE_LEGACY_OR_DSSDR);
            xsiResult = adi_emsi_SetCardConnector(sdcard->xsiHandle,
                sdcard->card == SDCARD_CARD_TYPE_SD ? true : false);
            if ( (xsiResult == ADI_EMSI_CARD_INSERTED) ||
                 (xsiResult == ADI_EMSI_SUCCESS) ) {
                sdcard->cardPresent = ADI_xSI_SUCCESS;
            } else {
                sdcard->cardPresent = ADI_xSI_FAILURE;
            }
            adi_emsi_SetTuning(sdcard->xsiHandle, true);
            adi_emsi_SetDmaType(sdcard->xsiHandle, ADI_EMSI_DMA_USED_ADMA2, 0);
            adi_emsi_SetBlkSze(sdcard->xsiHandle, 512);
            sdcard_SetIdentifyMode(sdcard);
#endif
            *sdcardHandle = sdcard;
            sdcard->open = true;
        } else {
            *sdcardHandle = NULL;
        }

    }
    SDCARD_UNLOCK();

    return(result);
}

SDCARD_SIMPLE_RESULT sdcard_close(sSDCARD **sdcardHandle)
{
    SDCARD_SIMPLE_RESULT result = SDCARD_SIMPLE_SUCCESS;
    ADI_xSI_RESULT xsiResult;
    sSDCARD *sdcard = *sdcardHandle;

    if (*sdcardHandle == NULL) {
        return (SDCARD_SIMPLE_ERROR);
    }

    SDCARD_LOCK();

#ifdef ADI_RSI
    xsiResult = adi_rsi_Close(sdcard->xsiHandle);
#else
    xsiResult = adi_emsi_Close(sdcard->xsiHandle);
#endif

    sdcard->open = false;
    *sdcardHandle = NULL;

    SDCARD_UNLOCK();

    return(result);
}

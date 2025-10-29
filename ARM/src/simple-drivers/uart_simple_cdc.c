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
 * Simple UART compatibility interface on top of the CLD
 * CDC + UAC library.
 *
 *   This UART driver supports:
 *     - FreeRTOS or no RTOS main-loop modes
 *     - Fully protected multi-threaded device transfers
 *     - Blocking transfers
 *
 * Copyright 2020 Analog Devices, Inc.  All rights reserved.
 *
*/
#include <string.h>

#ifdef FREE_RTOS
    #include "FreeRTOS.h"
    #include "semphr.h"
    #include "task.h"
    #define UART_ENTER_CRITICAL()  taskENTER_CRITICAL()
    #define UART_EXIT_CRITICAL()   taskEXIT_CRITICAL()
#else
    #define UART_ENTER_CRITICAL()
    #define UART_EXIT_CRITICAL()
#endif

#include "uart_simple_cdc.h"
#include "cdc.h"

#define UART_BUFFER_SIZE        (16384)
#define UART_DMA_BUFFER_SIZE    (512)
#define UART_END_CDC            (UART1)

#define UART_CDC_WORD_SIZE      (sizeof(uint32_t))
#define UART_CDC_WORD_MASK      (UART_CDC_WORD_SIZE-1)

typedef enum UART_SIMPLE_INT_RESULT
{
    UART_SIMPLE_INT_TX_OK,
    UART_SIMPLE_INT_TX_ERROR,
    UART_SIMPLE_INT_TX_FIFO_EMPTY,
    UART_SIMPLE_INT_RX_OK,
} UART_SIMPLE_INT_RESULT;

struct sUART {

    // UART receive buffer
    uint8_t rx_buffer[UART_BUFFER_SIZE];
    uint16_t rx_buffer_readptr;
    volatile uint16_t rx_buffer_writeptr;

    // UART transmit buffer
    uint8_t tx_buffer[UART_BUFFER_SIZE];
    uint8_t tx_dma_buffer[UART_DMA_BUFFER_SIZE] __attribute__ ((aligned(4)));
    volatile uint16_t tx_buffer_readptr;
    uint16_t tx_buffer_writeptr;
    uint16_t tx_size;

    // read/write timeouts mode
    int32_t readTimeout;
    int32_t writeTimeout;

    // misc
    bool transmitting;
    bool open;
    bool rxSleeping;
    bool txSleeping;

#ifdef FREE_RTOS
    SemaphoreHandle_t portLock;
    SemaphoreHandle_t portRxLock;
    SemaphoreHandle_t portTxLock;
    SemaphoreHandle_t portRxBlock;
    SemaphoreHandle_t portTxBlock;
    TickType_t rtosReadTimeout;
    TickType_t rtosWriteTimeout;
#else
    volatile bool uartDone;
#endif

};

/* UART port context containers.  Must currently be in uncached
 * memory since the CDC+UAC library is in DMA mode.
 *
 * FIXME: Remove uncached requirement
 */
__attribute__ ((section(".l3_uncached_data")))
    static sUART uartCdcContext[UART_END_CDC];

static bool uart_cdc_initialized = false;

/* Quick copy less that UART_CDC_WORD_SIZE buffers */
static inline void uart_cdc_quick_copy(uint8_t *out, uint8_t *in, unsigned len)
{
    if (len == 1) {
        out[0] = in[0];
    } else if (len == 2) {
        out[0] = in[0]; out[1] = in[1];
    } else {
        out[0] = in[0]; out[1] = in[1]; out[2] = in[2];
    }
}

UART_SIMPLE_INT_RESULT _uart_cdc_isr_readFromTXBuffer(sUART *uart, uint8_t *val)
{
    CLD_USB_Data_Transmit_Return_Type ok;
    UART_SIMPLE_INT_RESULT ret;
    uintptr_t aligned;
    uint8_t *buf;

    /* First check if write buffer is empty */
    if (uart->tx_buffer_writeptr == uart->tx_buffer_readptr) {
        return UART_SIMPLE_INT_TX_FIFO_EMPTY;
    }

    /* fetch latest chunk from TX buffer */
    if (uart->tx_buffer_writeptr < uart->tx_buffer_readptr) {
        uart->tx_size = UART_BUFFER_SIZE - uart->tx_buffer_readptr;
    } else {
        uart->tx_size = uart->tx_buffer_writeptr - uart->tx_buffer_readptr;
    }
    uart->tx_size = (uart->tx_size > UART_DMA_BUFFER_SIZE) ?
        UART_DMA_BUFFER_SIZE : uart->tx_size;

    /* Attempt to maintain 4-byte word alignment for direct DMA */
    buf = &uart->tx_buffer[uart->tx_buffer_readptr];
    aligned = (uintptr_t)buf & UART_CDC_WORD_MASK;
    if (aligned != 0) {
        if (uart->tx_size >= UART_CDC_WORD_SIZE) {
            uart->tx_size = UART_CDC_WORD_SIZE - aligned;
        }
        uart_cdc_quick_copy(uart->tx_dma_buffer, buf, uart->tx_size);
        buf = uart->tx_dma_buffer;
    } else {
        if (uart->tx_size >= UART_CDC_WORD_SIZE) {
            uart->tx_size &= ~UART_CDC_WORD_MASK;
        }
    }

    /* Send out the chunk */
    ok = cdc_tx_serial_data(uart->tx_size, buf, 100);

    /* Return the result */
    if (ok == CLD_USB_TRANSMIT_SUCCESSFUL) {
        uart->transmitting = true;
        ret = UART_SIMPLE_INT_TX_OK;
    } else {
        ret = UART_SIMPLE_INT_TX_ERROR;
    }

    return ret;
}

void _uart_cdc_tx_complete(CDC_TX_STATUS status, void *usrPtr)
{
    sUART *uart = (sUART *)usrPtr;
    UART_SIMPLE_INT_RESULT result;
#ifdef FREE_RTOS
    BaseType_t rtosResult;
    BaseType_t contextSwitch = pdFALSE;
#endif

    /* Increment the TX read pointer upon completion */
    uart->tx_buffer_readptr += uart->tx_size;
    if (uart->tx_buffer_readptr >= UART_BUFFER_SIZE) {
        uart->tx_buffer_readptr = 0;
    }

    /* Send the next chunk and update the transmitting status */
    result = _uart_cdc_isr_readFromTXBuffer(uart, NULL);
    if (result != UART_SIMPLE_INT_TX_OK) {
        uart->transmitting = false;
    }

#ifdef FREE_RTOS
    /* Wake any blocked threads */
    if (uart->txSleeping) {
        uart->txSleeping = false;
        rtosResult = xSemaphoreGiveFromISR(uart->portTxBlock, &contextSwitch);
        portYIELD_FROM_ISR(contextSwitch);
    }
#endif
}

void _uart_cdc_rx_complete(unsigned char *buffer,
    unsigned short length, void *usrPtr)
{
    sUART *uart = (sUART *)usrPtr;
    unsigned short idx;
    unsigned short size;
    bool full;
#ifdef FREE_RTOS
    BaseType_t rtosResult;
    BaseType_t contextSwitch = pdFALSE;
#endif

    idx = 0;
    do {
        full = ((uart->rx_buffer_writeptr + 1) % UART_BUFFER_SIZE) == uart->rx_buffer_readptr;
        if (!full) {
            if (uart->rx_buffer_writeptr < uart->rx_buffer_readptr) {
                size = uart->rx_buffer_readptr - uart->rx_buffer_writeptr - 1;
            } else {
                size = UART_BUFFER_SIZE - uart->rx_buffer_writeptr;
            }
            if (size > length) {
                size = length;
            }
            if (size < UART_CDC_WORD_SIZE) {
                uart_cdc_quick_copy(&uart->rx_buffer[uart->rx_buffer_writeptr], &buffer[idx], size);
            } else {
                memcpy(&uart->rx_buffer[uart->rx_buffer_writeptr], &buffer[idx], size);
            }
            length -= size; idx += size; uart->rx_buffer_writeptr += size;
            if (uart->rx_buffer_writeptr >= UART_BUFFER_SIZE) {
                uart->rx_buffer_writeptr = 0;
            }
        }
    } while (length && !full);

#ifdef FREE_RTOS
    /* Wake any blocked threads if new data is available */
    if ((idx > 0) && (uart->rxSleeping)) {
        uart->rxSleeping = false;
        rtosResult = xSemaphoreGiveFromISR(uart->portRxBlock, &contextSwitch);
        portYIELD_FROM_ISR(contextSwitch);
    }
#endif

}

UART_SIMPLE_RESULT uart_cdc_read(sUART *uart, uint8_t *in, uint16_t *inLen)
{
    UART_SIMPLE_RESULT result = UART_SIMPLE_SUCCESS;
    bool empty;
    int i;

#ifdef FREE_RTOS
    BaseType_t rtosResult;
#endif

#ifdef FREE_RTOS
    rtosResult = xSemaphoreTake(uart->portRxLock, portMAX_DELAY);
    if (rtosResult != pdTRUE) {
        result = UART_SIMPLE_ERROR;
    }
#endif

#ifdef FREE_RTOS
    UART_ENTER_CRITICAL();
    empty = (uart->rx_buffer_writeptr == uart->rx_buffer_readptr);
    if (empty) {
        uart->rxSleeping = true;
    }
    UART_EXIT_CRITICAL();
    if (empty) {
        rtosResult = xSemaphoreTake(uart->portRxBlock, uart->readTimeout);
    }
#else
    if (uart->readTimeout == UART_SIMPLE_TIMEOUT_INF) {
        do {
            empty = (uart->rx_buffer_writeptr == uart->rx_buffer_readptr);
        } while (empty);
    }
#endif

    UART_ENTER_CRITICAL();
    if (uart->rx_buffer_writeptr < uart->rx_buffer_readptr) {
        i = UART_BUFFER_SIZE - uart->rx_buffer_readptr;
    } else {
        i = uart->rx_buffer_writeptr - uart->rx_buffer_readptr;
    }
    UART_EXIT_CRITICAL();
    i = *inLen < i ? *inLen : i;
    if (i < UART_CDC_WORD_SIZE) {
        uart_cdc_quick_copy(in, &uart->rx_buffer[uart->rx_buffer_readptr], i);
    } else {
        memcpy(in, &uart->rx_buffer[uart->rx_buffer_readptr], i);
    }
    UART_ENTER_CRITICAL();
    uart->rx_buffer_readptr += i;
    if (uart->rx_buffer_readptr >= UART_BUFFER_SIZE) {
        uart->rx_buffer_readptr = 0;
    }
    UART_EXIT_CRITICAL();

#ifdef FREE_RTOS
    rtosResult = xSemaphoreGive(uart->portRxLock);
    if (rtosResult != pdTRUE) {
        result = UART_SIMPLE_ERROR;
    }
#endif

    *inLen = i;

    return(result);
}

UART_SIMPLE_RESULT uart_cdc_initiate_write(sUART *uart)
{
    UART_SIMPLE_RESULT result = UART_SIMPLE_SUCCESS;
    UART_SIMPLE_INT_RESULT intResult;
    UART_ENTER_CRITICAL();
    if (!uart->transmitting) {
        intResult = _uart_cdc_isr_readFromTXBuffer(uart, NULL);
        if (intResult != UART_SIMPLE_INT_TX_OK) {
            result = UART_SIMPLE_ERROR;
        }
    }
    UART_EXIT_CRITICAL();
    return(result);
}

UART_SIMPLE_RESULT uart_cdc_write(sUART *uart, uint8_t *out, uint16_t *outLen)
{
    UART_SIMPLE_RESULT result = UART_SIMPLE_SUCCESS;
    int i, size, remaining;
    bool full;
    bool goToSleep;
    bool transmitting;
#ifdef FREE_RTOS
    BaseType_t rtosResult;
#endif

#ifdef FREE_RTOS
    rtosResult = xSemaphoreTake(uart->portTxLock, portMAX_DELAY);
#endif

    i = 0; remaining = *outLen;
    while (remaining) {

#ifdef FREE_RTOS
        UART_ENTER_CRITICAL();
        full = ((uart->tx_buffer_writeptr + 1) % UART_BUFFER_SIZE) == uart->tx_buffer_readptr;
        transmitting = uart->transmitting;
        goToSleep = full & transmitting;
        if (goToSleep) {
            uart->txSleeping = true;
        }
        UART_EXIT_CRITICAL();
        if (goToSleep) {
            rtosResult = xSemaphoreTake(uart->portTxBlock, portMAX_DELAY);
            if (rtosResult != pdTRUE) {
                result = UART_SIMPLE_ERROR;
            }
        } else {
            if (full & !transmitting) {
                break;
            }
        }
#else
        do {
            full = ((uart->tx_buffer_writeptr+1) % UART_BUFFER_SIZE) == uart->tx_buffer_readptr;
        } while (full);
#endif

        UART_ENTER_CRITICAL();
        if (uart->tx_buffer_writeptr >= uart->tx_buffer_readptr) {
            size = UART_BUFFER_SIZE - uart->tx_buffer_writeptr;
        } else {
            size = uart->tx_buffer_readptr - uart->tx_buffer_writeptr;
        }
        size = (size > remaining) ? remaining : size;
        if (size < UART_CDC_WORD_SIZE) {
            uart_cdc_quick_copy(&uart->tx_buffer[uart->tx_buffer_writeptr], &out[i], size);
        } else {
            memcpy(&uart->tx_buffer[uart->tx_buffer_writeptr], &out[i], size);
        }
        remaining -= size; i += size;
        uart->tx_buffer_writeptr += size;
        if (uart->tx_buffer_writeptr >= UART_BUFFER_SIZE) {
            uart->tx_buffer_writeptr = 0;
        }
        UART_EXIT_CRITICAL();
    }

    /* Report back the bytes written */
    *outLen = i;

    /* Kick off a write if needed */
    uart_cdc_initiate_write(uart);

#ifdef FREE_RTOS
    rtosResult = xSemaphoreGive(uart->portTxLock);
#endif

    return(result);
}

UART_SIMPLE_RESULT uart_cdc_setProtocol(sUART *uartHandle,
    UART_SIMPLE_SPEED speed, UART_SIMPLE_WORD_LENGTH length,
    UART_SIMPLE_PARITY parity, UART_SIMPLE_STOP_BITS stop)
{
    return(UART_SIMPLE_SUCCESS);
}

#include <stdio.h>
UART_SIMPLE_RESULT uart_cdc_setTimeouts(sUART *uartHandle,
    int32_t readTimeout, int32_t writeTimeout)
{
    UART_SIMPLE_RESULT result = UART_SIMPLE_SUCCESS;
    sUART *uart = uartHandle;

#ifdef FREE_RTOS
    BaseType_t rtosResult;
#endif

#ifdef FREE_RTOS
    rtosResult = xSemaphoreTake(uart->portLock, portMAX_DELAY);
    if (rtosResult != pdTRUE) {
        result = UART_SIMPLE_ERROR;
    }
#endif

    if ((readTimeout != UART_SIMPLE_TIMEOUT_NO_CHANGE) &&
        (readTimeout != uart->readTimeout)) {
        uart->readTimeout = readTimeout;
#ifdef FREE_RTOS
        if (uart->readTimeout == UART_SIMPLE_TIMEOUT_INF) {
            uart->rtosReadTimeout = portMAX_DELAY;
        } else if (uart->readTimeout == UART_SIMPLE_TIMEOUT_NONE) {
            uart->rtosReadTimeout = 0;
        } else {
            uart->rtosReadTimeout = pdMS_TO_TICKS(uart->readTimeout);
        }
#endif
    }
    if ((writeTimeout != UART_SIMPLE_TIMEOUT_NO_CHANGE) &&
        (writeTimeout != uart->writeTimeout)) {
        uart->writeTimeout = writeTimeout;
#ifdef FREE_RTOS
        if (uart->writeTimeout == UART_SIMPLE_TIMEOUT_INF) {
            uart->rtosWriteTimeout = portMAX_DELAY;
        } else if (uart->writeTimeout == UART_SIMPLE_TIMEOUT_NONE) {
            uart->rtosWriteTimeout = 0;
        } else {
            uart->rtosWriteTimeout = pdMS_TO_TICKS(uart->readTimeout);
        }
#endif
    }

#ifdef FREE_RTOS
    rtosResult = xSemaphoreGive(uart->portLock);
    if (rtosResult != pdTRUE) {
        result = UART_SIMPLE_ERROR;
    }
#endif

    return(result);
}

UART_SIMPLE_RESULT uart_cdc_open(UART_SIMPLE_PORT port, sUART **uartHandle)
{
    UART_SIMPLE_RESULT result = UART_SIMPLE_SUCCESS;
    sUART *uart;
#ifdef FREE_RTOS
    BaseType_t rtosResult;
#endif

    if (port >= UART_END_CDC) {
        return(UART_SIMPLE_INVALID_PORT);
    }

    uart = &uartCdcContext[port];

#ifdef FREE_RTOS
    rtosResult = xSemaphoreTake(uart->portLock, portMAX_DELAY);
    if (rtosResult != pdTRUE) {
        result = UART_SIMPLE_ERROR;
    }
#endif

    if (uart->open == true) {
        result = UART_SIMPLE_PORT_BUSY;
    }

    if (result == UART_SIMPLE_SUCCESS) {

        cdc_register_tx_callback(_uart_cdc_tx_complete, uart);
        cdc_register_rx_callback(_uart_cdc_rx_complete, uart);

        memset(uart->rx_buffer, 0, sizeof(uart->rx_buffer));
        uart->rx_buffer_readptr = 0;
        uart->rx_buffer_writeptr = 0;

        memset(uart->tx_buffer, 0, sizeof(uart->tx_buffer));
        uart->tx_buffer_readptr = 0;
        uart->tx_buffer_writeptr = 0;

        uart->readTimeout = UART_SIMPLE_TIMEOUT_INF;
        uart->writeTimeout = UART_SIMPLE_TIMEOUT_INF;

#ifdef FREE_RTOS
        uart->rtosReadTimeout = portMAX_DELAY;
        uart->rtosWriteTimeout = portMAX_DELAY;
#endif

    }

    if (result == UART_SIMPLE_SUCCESS) {
        *uartHandle = uart;
        uart->open = true;
    } else {
        *uartHandle = NULL;
    }

#ifdef FREE_RTOS
    rtosResult = xSemaphoreGive(uart->portLock);
    if (rtosResult != pdTRUE) {
        result = UART_SIMPLE_ERROR;
    }
#endif

    return(result);
}

UART_SIMPLE_RESULT uart_cdc_close(sUART **uartHandle)
{
    UART_SIMPLE_RESULT result = UART_SIMPLE_SUCCESS;
    sUART *uart = *uartHandle;
#ifdef FREE_RTOS
    BaseType_t rtosResult;
#endif

    if (*uartHandle == NULL) {
        return (UART_SIMPLE_ERROR);
    }

#ifdef FREE_RTOS
    rtosResult = xSemaphoreTake(uart->portLock, portMAX_DELAY);
    if (rtosResult != pdTRUE) {
        result = UART_SIMPLE_ERROR;
    }
#endif

    uart->open = false;

    cdc_register_tx_callback(NULL, NULL);
    cdc_register_rx_callback(NULL, NULL);

    *uartHandle = NULL;

#ifdef FREE_RTOS
    rtosResult = xSemaphoreGive(uart->portLock);
    if (rtosResult != pdTRUE) {
        result = UART_SIMPLE_ERROR;
    }
#endif

    return(result);
}

UART_SIMPLE_RESULT uart_cdc_init(void)
{
    UART_SIMPLE_RESULT result = UART_SIMPLE_SUCCESS;
    uint8_t port;
    sUART *uart;

    if (uart_cdc_initialized == true) {
        return(UART_SIMPLE_ERROR);
    }

    memset(uartCdcContext, 0, sizeof(uartCdcContext));

    for (port = UART0; port < UART_END_CDC; port++) {

        uart = &uartCdcContext[port];

#ifdef FREE_RTOS
        uart->portLock = xSemaphoreCreateMutex();
        uart->portRxLock = xSemaphoreCreateMutex();
        uart->portTxLock = xSemaphoreCreateMutex();

        if (uart->portLock == NULL) {
            result = UART_SIMPLE_ERROR;
        }

        if (uart->portRxLock == NULL) {
            result = UART_SIMPLE_ERROR;
        }

        if (uart->portTxLock == NULL) {
            result = UART_SIMPLE_ERROR;
        }

        uart->portRxBlock = xSemaphoreCreateCounting(1, 0);
        if (uart->portRxBlock == NULL) {
            result = UART_SIMPLE_ERROR;
        }

        uart->portTxBlock = xSemaphoreCreateCounting(1, 0);
        if (uart->portTxBlock == NULL) {
            result = UART_SIMPLE_ERROR;
        }
#endif
        uart->open = false;

    }

    uart_cdc_initialized = true;

    return(result);
}


UART_SIMPLE_RESULT uart_cdc_deinit(void)
{
    UART_SIMPLE_RESULT result = UART_SIMPLE_SUCCESS;
    uint8_t port;
    sUART *uart;

    for (port = UART0; port < UART_END_CDC; port++) {

        uart = &uartCdcContext[port];

#ifdef FREE_RTOS
        if (uart->portRxBlock) {
            vSemaphoreDelete(uart->portRxBlock);
            uart->portRxBlock = NULL;
        }

        if (uart->portTxBlock) {
            vSemaphoreDelete(uart->portTxBlock);
            uart->portTxBlock = NULL;
        }

        if (uart->portLock) {
            vSemaphoreDelete(uart->portLock);
            uart->portLock = NULL;
        }

        if (uart->portRxLock) {
            vSemaphoreDelete(uart->portRxLock);
            uart->portLock = NULL;
        }

        if (uart->portRxLock) {
            vSemaphoreDelete(uart->portRxLock);
            uart->portLock = NULL;
        }
#endif

    }

    return(result);
}

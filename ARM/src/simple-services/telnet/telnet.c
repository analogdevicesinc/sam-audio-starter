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

/*
 * https://github.com/seanmiddleditch/libtelnet
 * http://pcmicro.com/netfoss/telnet.html
 */

#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "libtelnet.h"
#include "telnet_cfg.h"
#include "telnet.h"
#include "shell.h"
#include "uart_stdio.h"
#include "uart_stdio_io.h"
#include "cces_hacks.h"

#include "lwip/opt.h"
#include "lwipopts.h"
#include <lwip/sockets.h>

#ifndef TELNET_CALLOC
#define TELNET_CALLOC calloc
#endif

#ifndef TELNET_FREE
#define TELNET_FREE free
#endif

typedef struct TELNET_CONTEXT {
    SHELL_CONTEXT *shell;
    int sock;
    fd_set fdset;
    in_addr_t clientAddr;
    int timeout;
    telnet_t *libtelnet;
    char c;
    bool cValid;
    int mode;
} TELNET_CONTEXT;

static void telnet_terminate(TELNET_CONTEXT *t)
{
    /* Close the socket */
    lwip_close(t->sock);

    /* Free libtelnet resources */
    telnet_free(t->libtelnet);

    /* Free shell resources */
    shell_deinit(t->shell);

    /* Free the shell context */
    if (t->shell) {
        SHELL_FREE(t->shell);
    }

    /* Free remaining resources */
    SHELL_FREE(t);
    THREAD_FREE_STDIO();
    vTaskDelete(NULL);
}

static void telnet_term_out(char data, void *usr)
{
    TELNET_CONTEXT *t = (TELNET_CONTEXT *)usr;

    if ((data == '\n') && (t->mode == UART_STDIO_IO_MODE_COOKED)) {
        static const char crlf[] = "\r\n";
        telnet_send(t->libtelnet, crlf, sizeof(crlf)-1);
    } else {
        telnet_send(t->libtelnet, &data, sizeof(data));
    }
}

void libtelnetEvent(telnet_t *telnet, telnet_event_t *event, void *user_data)
{
    TELNET_CONTEXT *t = (TELNET_CONTEXT *)user_data;
    int n;

    switch (event->type) {
        case TELNET_EV_DATA:
            t->c = event->data.buffer[0];
            t->cValid = true;
            break;
        case TELNET_EV_SEND:
            n = lwip_write(t->sock, event->data.buffer, event->data.size);
            if (n < 0) {
                telnet_terminate(t);
            }
            break;
        case TELNET_EV_WARNING:
        case TELNET_EV_ERROR:
            asm("nop;");
            break;
        case TELNET_EV_WILL:
        case TELNET_EV_WONT:
        case TELNET_EV_DO:
        case TELNET_EV_DONT:
        case TELNET_EV_SUBNEGOTIATION:
        default:
            asm("nop;");
            break;
    }

}

static int telnet_term_in(int mode, void *usr)
{
    TELNET_CONTEXT *t = (TELNET_CONTEXT *)usr;
    struct timeval timeout;
    int n;
    char c = -1;

    if (mode == STDIO_TIMEOUT_NONE) {
        mode = TERM_INPUT_DONT_WAIT;
    } else if (mode == STDIO_TIMEOUT_INF) {
        mode = TERM_INPUT_WAIT;
    }

    if (mode == TERM_INPUT_DONT_WAIT) {
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
    } else if (mode == TERM_INPUT_WAIT) {
        timeout.tv_sec = 86400;
        timeout.tv_usec = 0;
    } else {
        timeout.tv_sec = mode / 1000000;
        timeout.tv_usec = mode - (timeout.tv_sec * 1000000);
    }

    do {
        FD_ZERO(&t->fdset);
        FD_SET(t->sock, &t->fdset);
        n = lwip_select(t->sock + 1, &t->fdset, NULL, NULL, &timeout);
        if (n < 0) {
            telnet_terminate(t);
        } else if (n > 0) {
            if (FD_ISSET(t->sock, &t->fdset)) {
                n = lwip_read(t->sock, &c, sizeof(c));
                if (n < 0) {
                    telnet_terminate(t);
                }
                if (n != sizeof(c)) {
                    return(-1);
                }
                telnet_recv(t->libtelnet, &c, sizeof(c));
                if (t->cValid) {
                    t->cValid = false;
                    return(t->c);
                }
            }
        } else {
            return(-1);
        }
    } while (1);

    return(c);
}

int telnet_stdout(unsigned char *ptr, int len, void *usr)
{
    TELNET_CONTEXT *t = (TELNET_CONTEXT *)usr;
    int i;
    for (i = 0; i < len; i++) {
        telnet_term_out(ptr[i], t);
    }
    return(len);
}

int telnet_stdin(unsigned char *buffer, int len, void *usr)
{
    TELNET_CONTEXT *t = (TELNET_CONTEXT *)usr;
    int rlen = 0;
    int c;
    c = telnet_term_in(t->mode, t);
    if (c >= 0) {
        buffer[0] = c;
        rlen = 1;
    }
    return(rlen);
}

void telnet_set_read_timeout(int timeout, void *usr)
{
    TELNET_CONTEXT *t = (TELNET_CONTEXT *)usr;
    t->timeout = timeout;
}

int telnet_set_mode(int mode, void *usr)
{
    TELNET_CONTEXT *t = (TELNET_CONTEXT *)usr;
    int oldMode = t->mode;
    t->mode = mode;
    return(oldMode);
}

static const telnet_telopt_t telopts[] = {
    { TELNET_TELOPT_ECHO,      TELNET_WILL, TELNET_DONT },
    { TELNET_TELOPT_SGA,       TELNET_WILL, TELNET_DO   },
    { TELNET_TELOPT_BINARY,    TELNET_WILL, TELNET_DO   },
    { -1, 0, 0 }
};

portTASK_FUNCTION(telnetClientTask, pvParameters)
{
    TELNET_CONTEXT *t = (TELNET_CONTEXT *)pvParameters;

    UART_STDIO_IO telnetIO = {
        .out = telnet_stdout,
        .in = telnet_stdin,
        .set_read_timeout = telnet_set_read_timeout,
        .set_mode = telnet_set_mode,
        .usr = t
    };

    /* This thread uses stdio */
    THREAD_INIT_STDIO();

    /* Create a shell context */
    t->shell = SHELL_CALLOC(1, sizeof(*t->shell));

    /* Set crlf mode */
    t->mode = UART_STDIO_IO_MODE_COOKED;

    /* Create a libtelnet context */
    t->libtelnet = telnet_init(
        telopts, libtelnetEvent, /*0*/ TELNET_FLAG_NVT_EOL, (void *)t);

    /* We will echo */
    telnet_negotiate(t->libtelnet, TELNET_WILL, TELNET_TELOPT_ECHO);

    /* We will do binary mode */
    telnet_negotiate(t->libtelnet, TELNET_WILL, TELNET_TELOPT_BINARY);

    /* You do binary mode */
    telnet_negotiate(t->libtelnet, TELNET_DO, TELNET_TELOPT_BINARY);

    /* We will suppress go-ahead */
    telnet_negotiate(t->libtelnet, TELNET_WILL, TELNET_TELOPT_SGA);

    /* Initialize the shell */
    shell_init(t->shell, telnet_term_out, telnet_term_in, SHELL_MODE_BLOCKING, (void *)t);

    /* Store telnet I/O context in TLS */
    vTaskSetThreadLocalStoragePointer(
        NULL, configSTDIO_APP_TLS_POINTER, (void *)&telnetIO
    );

    /* Drop into the shell */
    shell_start(t->shell);

    /* Getting here means the exit command was issued */
    telnet_terminate(t);
}

portTASK_FUNCTION(telnetTask, pvParameters)
{
    int fd;
    int sock;
    int result;
    struct sockaddr_in sockname = {
        .sin_family = AF_INET,
        .sin_port = htons(TELNET_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)};

    TELNET_CONTEXT *t;

    fd = lwip_socket(PF_INET, SOCK_STREAM, 0);
    assert(fd >= 0);
    result = lwip_bind(fd, (struct sockaddr *)&sockname, sizeof(struct sockaddr));
    assert(result >= 0);
    result = lwip_listen(fd, TELNET_MAX_CONNECTIONS);
    assert(result >= 0);

    while (1)
    {
        socklen_t addrlen = sizeof(struct sockaddr);
        if ((sock = lwip_accept(fd, (struct sockaddr *)&sockname, &addrlen)) >= 0)
        {
            t = TELNET_CALLOC(1, sizeof(*t));
            t->clientAddr = sockname.sin_addr.s_addr;
            t->sock = sock;
            xTaskCreate(telnetClientTask, "TelnetClientTask", STARTUP_TASK_STACK_SIZE,
                t, STARTUP_TASK_LOW_PRIORITY, NULL);
        }
    }
}

void telnet_start(APP_CONTEXT *context)
{
    xTaskCreate(telnetTask, "TelnetTask", GENERIC_TASK_STACK_SIZE,
        NULL, STARTUP_TASK_LOW_PRIORITY, &context->telnetTaskHandle);
}

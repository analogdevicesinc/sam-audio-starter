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

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "shell_printf.h"
#include "term.h"
#include "shell.h"
#include "syslog.h"

/* Use lightweight printf */
#include "printf.h"

int shell_vprintf(SHELL_CONTEXT *ctx, const char *fmt, va_list ap)
{
    va_list va;
    char *str;
    va = ap;
    int len = vsnprintf(NULL, 0, fmt, va);
    va = ap;
    str = SHELL_MALLOC(len + 1);
    vsnprintf(str, len + 1, fmt, va);
    switch (ctx->redirect) {
        case 0:
            break;
        case 1:
            term_putstr(&ctx->t, str, len);
            break;
        case 2:
            syslog_print(str);
            break;
        case 3:
            if (ctx->redirectFile) {
                fwrite(str, 1, strlen(str), ctx->redirectFile);
            }
            break;
        default:
            term_putstr(&ctx->t, str, len);
            break;

    }
    SHELL_FREE(str);
    return(len);
}

int shell_printf(SHELL_CONTEXT *ctx, const char *fmt, ...)
{
    int len;
    va_list args;
    va_start(args, fmt);
    len = shell_vprintf(ctx, fmt, args);
    va_end(args);
    return(len);
}

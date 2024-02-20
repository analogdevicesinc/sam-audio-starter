/*  $NetBSD: getline.c,v 1.2 2014/09/16 17:23:50 christos Exp $ */

/*-
 * Copyright (c) 2011 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Christos Zoulas.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This file has been modified by Analog Devices, Inc. to build in the
 * SHARC Audio Starter environment.
 *
 * Source: https://github.com/freebsd/freebsd-src/blob/main/contrib/file/src/getline.c
 * Version: 12.4.0
 * Commit: 4460e5b02d2e07d8aa286fff8f644a442b376cae
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include "getline_cfg.h"

#ifndef GETLINE_MALLOC
#define GETLINE_MALLOC  malloc
#endif
#ifndef GETLINE_FREE
#define GETLINE_FREE    free
#endif
#ifndef GETLINE_REALLOC
#define GETLINE_REALLOC realloc
#endif

ssize_t
getdelim(char **buf, size_t *bufsiz, int delimiter, FILE *fp)
{
    char *ptr, *eptr;


    if (*buf == NULL || *bufsiz == 0) {
        *bufsiz = BUFSIZ;
        if ((*buf = GETLINE_MALLOC(*bufsiz)) == NULL)
            return -1;
    }

    for (ptr = *buf, eptr = *buf + *bufsiz;;) {
        int c = fgetc(fp);
        if (c == -1) {
            if (feof(fp)) {
                ssize_t diff = (ssize_t)(ptr - *buf);
                if (diff != 0) {
                    *ptr = '\0';
                    return diff;
                }
            }
            return -1;
        }
        *ptr++ = c;
        if (c == delimiter) {
            *ptr = '\0';
            return ptr - *buf;
        }
        if (ptr + 2 >= eptr) {
            char *nbuf;
            size_t nbufsiz = *bufsiz * 2;
            ssize_t d = ptr - *buf;
            if ((nbuf = GETLINE_REALLOC(*buf, nbufsiz)) == NULL)
                return -1;
            *buf = nbuf;
            *bufsiz = nbufsiz;
            eptr = nbuf + nbufsiz;
            ptr = nbuf + d;
        }
    }
}

ssize_t
getline(char **buf, size_t *bufsiz, FILE *fp)
{
    return getdelim(buf, bufsiz, '\n', fp);
}

#ifdef TEST
int
main(int argc, char *argv[])
{
    char *p = NULL;
    ssize_t len;
    size_t n = 0;

    while ((len = getline(&p, &n, stdin)) != -1)
        (void)printf("%" SIZE_T_FORMAT "d %s", len, p);
    GETLINE_FREE(p);
    return 0;
}
#endif

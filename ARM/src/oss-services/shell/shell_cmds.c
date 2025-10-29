/**
 * Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "FreeRTOS.h"
#include "task.h"

#include "context.h"
#include "syslog.h"
#include "twi_simple.h"
#include "shell.h"
#include "shell_printf.h"
#include "term.h"
#include "xmodem.h"
#include "util.h"
#include "clock_domain.h"
#include "uart_stdio.h"
#include "_getopt.h"

#ifdef printf
#undef printf
#endif

#ifdef vprintf
#undef vprintf
#endif

#define printf(...) shell_printf(ctx, __VA_ARGS__)
#define vprintf(x,y) shell_vprintf(ctx, x, y)

/***********************************************************************
 * Main application context
 **********************************************************************/
static APP_CONTEXT *context = &mainAppContext;

/***********************************************************************
 * Misc helper functions
 **********************************************************************/
static void dumpBytes(SHELL_CONTEXT *ctx, unsigned char *rdata, unsigned addr, unsigned rlen)
{
    unsigned i;
    for (i = 0; i < rlen; i++) {
        if ((i % 16) == 0) {
            if (i) {
                printf("\n");
            }
            printf("%08x: ", addr + i);
        }
        printf("%02x ", rdata[i]);
    }
    printf("\n");
}

static unsigned str2bytes(char *str, unsigned char **bytes)
{
    char *delim = " ,";
    char *saveptr = NULL;
    char *token = NULL;
    unsigned wlen = 0;
    unsigned allocLen = 0;
    unsigned char *wdata = NULL;

    wlen = 0; allocLen = 0;
    token = strtok_r(str, delim, &saveptr);
    while (token) {
        if ((wlen + 1) > allocLen) {
            allocLen += 256;
            wdata = SHELL_REALLOC(wdata, allocLen * sizeof(*wdata));
        }
        wdata[wlen] = strtoul(token, NULL, 0);
        wlen++;
        token = strtok_r(NULL, delim, &saveptr);
    }

    if (bytes) {
        *bytes = wdata;
    }

    return(wlen);
}

enum {
    INVALID_FS = -1,
    SPIFFS_FS = 0,
    SD_FS,
    EMMC_FS
};

static int fsVolOK(SHELL_CONTEXT *ctx, APP_CONTEXT *context, char *fs)
{
    if (strcmp(fs, SPIFFS_VOL_NAME) == 0) {
        if (context->spiffsHandle) {
            return SPIFFS_FS;
        } else {
            printf("SPIFFS has not been initialized!\n");
            return INVALID_FS;
        }
    }
#if defined(SDCARD_VOL_NAME) && !defined(SDCARD_USE_EMMC)
    if (strcmp(fs, SDCARD_VOL_NAME) == 0) {
        if (context->sdcardHandle) {
            return SD_FS;
        } else {
            printf("SDCARD has not been initialized!\n");
            return INVALID_FS;
        }
    }
#endif
#if defined(EMMC_VOL_NAME) && defined(SDCARD_USE_EMMC)
    if (strcmp(fs, EMMC_VOL_NAME) == 0) {
        if (context->sdcardHandle) {
            return EMMC_FS;
        } else {
            printf("EMMC has not been initialized!\n");
            return INVALID_FS;
        }
    }
#endif
    printf("Invalid drive name specified.\n");
    return INVALID_FS;
}

static bool encodeUtf8(SHELL_CONTEXT *ctx, long int codepoint, char * buf)
{
    char u, v, w, x, y, z;
    bool ok = true;

    if(buf != NULL) {
        if(codepoint > 0){
            if(codepoint <= 0x7FU) {
                /* Input format: 0yyyzzzz */
                /* Output format: 0yyyzzzz */
                buf[0] = (char) codepoint;
                buf[1] = 0;
            } else if(codepoint <= 0x7FFU) {
                /* Input format: 00000xxx yyyyzzzz */
                x = (codepoint & 0x700U) >> 8;
                y = (codepoint & 0x0F0U) >> 4;
                z = codepoint & 0x0FU;
                /* Output format: 110xxxyy 10yyzzzz */
                buf[0] = 0xC0 | (x << 2) | (y >> 2);
                buf[1] = 0x80 | ((y & 0x03) << 4) | z;
                buf[2] = 0;
            } else if(codepoint <= 0xFFFFU) {
                /* Input format: wwwwxxxx yyyyzzzz */
                w = (codepoint & 0xF000U) >> 12;
                x = (codepoint & 0x0F00U) >> 8;
                y = (codepoint & 0x00F0U) >> 4;
                z = codepoint & 0x000FU;
                /* Output format: 1110wwww 10xxxxyy 10yyzzzz */
                buf[0] = 0xE0 | w;
                buf[1] = 0x80 | (x << 2) | (y >> 2);
                buf[2] = 0x80 | ((y & 0x03) << 4) | z;
                buf[3] = 0;
            } else if(codepoint <= 0x10FFFFU) {
                /* Input format: 000uvvvv wwwwxxxx yyyyzzzz */
                u = (codepoint & 0x100000U) >> 20;
                v = (codepoint & 0x0F0000U) >> 16;
                w = (codepoint & 0x00F000U) >> 12;
                x = (codepoint & 0x000F00U) >> 8;
                y = (codepoint & 0x0000F0U) >> 4;
                z = codepoint & 0x000FU;
                /* Output format: 11110uvv 10vvwwww 10xxxxyy 10yyzzzz */
                buf[0] = 0xF0 | (u << 2) | (v >> 2);
                buf[1] = 0x80 | ((v & 0x03) << 4) | w;
                buf[2] = 0x80 | (x << 2) | (y >> 2);
                buf[3] = 0x80 | ((y & 0x03) << 4) | z;
                buf[4] = 0;
            } else {
                printf("Invalid unicode character (must be less than 10FFFF)\n");
                ok = false;
            }
        } else {
            printf("Invalid unicode character (must be hex string)\n");
            ok = false;
        }
    } else {
        ok = false;
    }
    return ok;
}

void copyFileName(char ** dest, char * src)
{
    if(src) {
        if (*dest) {
            SHELL_FREE(*dest);
            *dest = NULL;
        }
        *dest = SHELL_MALLOC(strlen(src) + 1);
        (void) strcpy(*dest, src);
    }
}

#define FILE_COUNT_CHARACTERS   (11)    /* Max unsigned value + underscore, e.g. "_4294967295" */

void insertCountToFileName(char ** dest, char * src, unsigned fcount)
{
    char *fname = NULL;
    char *temp = NULL;
    char *ext = NULL;
    size_t nameLen, extLen;

    if(fcount > 0) {
        /* Split file name and extension (if it exists). */
        temp = (char *) strrchr(src, '.');
        if(temp) {
            extLen = strlen(temp);
            nameLen = strlen(src) - extLen;
            ext = SHELL_MALLOC(extLen + 1);
            if(ext == NULL) {
                return;
            }
            (void) strcpy(ext, temp);
            temp = NULL;
        } else {
            nameLen = strlen(src);
            extLen = 0;
        }

        /* Build file name */
        fname = SHELL_MALLOC(nameLen + extLen + FILE_COUNT_CHARACTERS + 1);
        if(fname != NULL) {
            (void) strncpy(fname, src, nameLen);
            (void) sprintf(&fname[nameLen], "_%d%s",  fcount, ext ? ext : "");
        }
        copyFileName(dest, fname);
    } else {
        copyFileName(dest, src);
    }

    if(fname) {
        SHELL_FREE(fname);
    }
    if(ext) {
        SHELL_FREE(ext);
    }
}

/***********************************************************************
 * XMODEM helper functions
 **********************************************************************/
typedef struct XMODEM_STATE {
    SHELL_CONTEXT *ctx;
} XMODEM_STATE;

static void shell_xmodem_putchar(int c, void *usr)
{
    XMODEM_STATE *x = (XMODEM_STATE *)usr;
    SHELL_CONTEXT *ctx = x->ctx;
    TERM_STATE *t = &ctx->t;

    t->term_out(c, t->usr);
}

static int shell_xmodem_getchar(int timeout, void *usr)
{
    XMODEM_STATE *x = (XMODEM_STATE *)usr;
    SHELL_CONTEXT *ctx = x->ctx;
    TERM_STATE *t = &ctx->t;
    int c;

    c = t->term_in(timeout, t->usr);

    return(c);
}

typedef struct FLASH_WRITE_STATE {
    XMODEM_STATE xmodem;
    const FLASH_INFO *flash;
    unsigned addr;
    unsigned maxAddr;
    unsigned eraseBlockSize;
} FLASH_WRITE_STATE;

void flashDataWrite(void *usr, void *data, int size)
{
   FLASH_WRITE_STATE *state = (FLASH_WRITE_STATE *)usr;
   int err;

   if (size > 0) {
      if ((state->addr % state->eraseBlockSize) == 0) {
         err = flash_erase(state->flash, state->addr, state->eraseBlockSize);
         if (err != FLASH_OK) {
            return;
         }
      }
      if ((state->addr + size) <= state->maxAddr) {
         err = flash_program(state->flash, state->addr, (const unsigned char *)data, size);
         if (err != FLASH_OK) {
            return;
         }
         state->addr += size;
      }
   }
}

typedef struct FILE_XFER_STATE {
    XMODEM_STATE xmodem;
    FILE *f;
    void *data;
    int size;
} FILE_XFER_STATE;

void fileDataWrite(void *usr, void *data, int size)
{
    FILE_XFER_STATE *state = (FILE_XFER_STATE *)usr;
    size_t wsize;

    /*
     * Need to double-buffer the data in order to strip off the
     * trailing packet bytes at the end of an xmodem transfer.
     */
    if (state->data == NULL) {
        state->data = SHELL_MALLOC(1024);
        memcpy(state->data, data, size);
        state->size = size;
    } else {
        if (data) {
            wsize = fwrite(state->data, 1, state->size, state->f);
            memcpy(state->data, data, size);
            state->size = size;
        } else {
            uint8_t *buf = (uint8_t *)state->data;
            while (state->size && buf[state->size-1] == '\x1A') {
               state->size--;
            }
            wsize = fwrite(state->data, 1, state->size, state->f);
            if (state->data) {
                SHELL_FREE(state->data);
                state->data = NULL;
            }
        }
    }
}

void fileDataRead(void *usr, void *data, int size)
{
    FILE_XFER_STATE *state = (FILE_XFER_STATE *)usr;
    size_t rsize;

    if (size > 0) {
        rsize = fread(data, 1, size, state->f);
    }
}

int confirmDanger(SHELL_CONTEXT *ctx, char *warnStr)
{
    char c;

    printf( "%s\n", warnStr );
    printf( "Are you sure you want to continue? [y/n]" );

    c = term_getch( &ctx->t, TERM_INPUT_WAIT );
    printf( "%c\n", isprint( c ) ? c : ' ' );

    if( tolower(c) == 'y' ) {
        return(1);
    }

    return(0);
}

/***********************************************************************
 * CMD: dump
 **********************************************************************/
const char shell_help_dump[] = "[addr] <len>\n"
    "  addr - Starting address to dump\n"
    "  len - Number of bytes to dump (default 1)\n";
const char shell_help_summary_dump[] = "Hex dump of flash contents";

#include "flash.h"

#define DUMP_SIZE 512

void shell_dump(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    uintptr_t addrReq = 0;
    unsigned len = 1;
    unsigned lenReq = 1;
    unsigned bytesRead = 0;
    uintptr_t addrCur;
    int ok;
    uint8_t *buf;

    if (argc < 2) {
        printf("Invalid arguments\n");
        return;
    }

    addrReq = strtoul(argv[1], NULL, 0);
    addrCur = addrReq;
    if (argc > 2) {
        lenReq = strtoul(argv[2], NULL, 0);
    }

    do {
        len = (lenReq - bytesRead);
        if (len > DUMP_SIZE) {
            len = DUMP_SIZE;
        }
        buf = SHELL_MALLOC(len);
        if (buf) {
            ok = flash_read(context->flashHandle, addrCur, buf, len);
            if (ok == FLASH_OK) {
                dumpBytes(ctx, buf, addrCur, len);
                bytesRead += len;
                addrCur += len;
            }
            SHELL_FREE(buf);
        }
    } while ((bytesRead < lenReq) && (ok == FLASH_OK));
}

/***********************************************************************
 * CMD: fdump
 **********************************************************************/
const char shell_help_fdump[] =
  "[file] <start> <size>\n"
  "  file - File to dump\n"
  "  start - Start offset in bytes (default: 0)\n"
  "  size - Size in bytes (default: full file)\n";
const char shell_help_summary_fdump[] = "Dumps the contents of a file in hex";

void shell_fdump(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    FILE *f;
    size_t start = 0;
    size_t size = SIZE_MAX;
    uint8_t *buf = NULL;
    size_t rlen;
    int c;

    if( argc < 2 ) {
        printf("No file given\n");
        return;
    }

    if (argc > 2) {
        start = (size_t)strtoul(argv[2], NULL, 0);
    }

    if (argc > 3) {
        size = (size_t)strtoul(argv[3], NULL, 0);
    }

    f = fopen(argv[1], "r" );
    if (f) {
        buf = SHELL_MALLOC(DUMP_SIZE);
        fseek(f, start, SEEK_SET);
        do {
            rlen = (size < DUMP_SIZE) ? size : DUMP_SIZE;
            rlen = fread(buf, sizeof(*buf), rlen, f);
            if (rlen) {
                dumpBytes(ctx, buf, start, rlen);
                size -= rlen;
                start += rlen;
                c = term_getch(&ctx->t, TERM_INPUT_DONT_WAIT);
            }
        } while (size && rlen && (c < 0));
        if (buf) {
            SHELL_FREE(buf);
        }
        fclose(f);
    } else {
        printf("Unable to open '%s'\n", argv[1]);
    }
}

/***********************************************************************
 * CMD: recv
 **********************************************************************/
const char shell_help_recv[] = "<file>\n"
    "  Transfer and save to file\n";
const char shell_help_summary_recv[] = "Receive a file via XMODEM";

void shell_recv( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    FILE_XFER_STATE fileState = { 0 };
    int oldMode;
    long size;

    if( argc != 2 ) {
        printf( "Usage: recv <file>\n" );
        return;
    }

    fileState.xmodem.ctx = ctx;

    fileState.f = fopen( argv[ 1 ], "wb");
    if( fileState.f == NULL) {
        printf( "unable to open file %s\n", argv[ 1 ] );
        return;
    }
    printf( "Prepare your terminal for XMODEM send ... " );
    oldMode = uart_stdio_set_mode(UART_STDIO_MODE_RAW);
    size = XmodemReceiveCrc(fileDataWrite, &fileState, INT_MAX,
        shell_xmodem_getchar, shell_xmodem_putchar);
    uart_stdio_set_mode(oldMode);
    printf("\n"); delay(250);
    if (size < 0) {
        printf( "XMODEM Error: %ld\n", size);
    } else {
        printf( "Received and saved as %s\n", argv[ 1 ] );
    }
    fclose( fileState.f );
}

/***********************************************************************
 * CMD: send
 **********************************************************************/
const char shell_help_send[] = "<file1> [<file2> ...]\n";
const char shell_help_summary_send[] = "Send files via YMODEM.";

#include "uart_stdio.h"

typedef struct YMODEM_STATE {
    FILE_XFER_STATE state;
    const char *fname;
    size_t fsize;
} YMODEM_STATE;

static void ymodem_hdr(void *usr, void *xmodemBuffer, int xmodemSize)
{
    YMODEM_STATE *y = (YMODEM_STATE *)usr;
    snprintf(xmodemBuffer, xmodemSize, "%s%c%u", y->fname, 0, (unsigned)y->fsize);
}

static void ymodem_end(void *xs, void *xmodemBuffer, int xmodemSize)
{
}

void shell_send(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    size_t size;
    FILE *fp = NULL;
    int ret = -1;
    int oldMode;
    int i;

    YMODEM_STATE y = {
        .state.xmodem.ctx = ctx,
    };

    if (argc < 2) {
        printf("Usage: %s <file1> [<file2> ...]\n", argv[0]);
        return;
    }

    printf ("Prepare your terminal for YMODEM receive...\n");
    oldMode = uart_stdio_set_mode(UART_STDIO_MODE_RAW);
    for (i = 1; i < argc; i++) {
        fp = fopen( argv[i], "rb");
        if (fp) {
            fseek(fp, 0, SEEK_END);
            size = ftell(fp);
            fseek(fp, 0, SEEK_SET);
            y.fname = argv[i]; y.fsize = size; y.state.f = fp;
            ret = XmodemTransmit(ymodem_hdr, &y, 128, 0, 1,
                shell_xmodem_getchar, shell_xmodem_putchar);
            if (ret >= 0) {
                ret = XmodemTransmit(fileDataRead, &y, y.fsize, 1, 0,
                    shell_xmodem_getchar, shell_xmodem_putchar);
            }
            fclose(fp);
            if (ret < 0) {
                break;
            }
        }
    }
    if (ret >= 0) {
        ret = XmodemTransmit(ymodem_end, &y, 128, 0, 1,
            shell_xmodem_getchar, shell_xmodem_putchar);
    }
    uart_stdio_set_mode(oldMode);
    if (ret < 0) {
        printf( "YMODEM Error: %ld\n", ret);
    }

}

/**********************************************************************
 * CMD: i2c
 **********************************************************************/
const char shell_help_i2c[] = "<i2c_port> <i2c_addr> <mem_addr> <wdata> <length> [addr_len]\n"
  "  i2c_port - I2C port to probe\n"
  "  i2c_addr - I2C device address\n"
  "  mem_addr - Starting memory address\n"
  "  wdata - Comma separated string of bytes to write (i.e. \"1,0x02,3\")\n"
  "          Empty quotes for read only.\n"
  "  length - Number of bytes to read.  Zero for write only.\n"
  "  addr_len - Number of address bytes: 1 = 1-byte address length, 2 = 2-byte address length (default 1)\n";
const char shell_help_summary_i2c[] = "Executes an I2C write/read transaction";

void shell_i2c(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    sTWI *twiHandle;
    TWI_SIMPLE_PORT twiPort;
    uint16_t  reg_addr;
    uint8_t  i2c_addr;
    uint16_t  length;
    uint16_t  addrLength;
    uint8_t *twiWrBuffer = NULL;
    uint8_t *wdata = NULL;
    uint8_t twiWrLen;
    uint8_t *twiRdBuffer = NULL;
    TWI_SIMPLE_RESULT result;

    if (argc < 5) {
        printf( "Invalid arguments. Type help [<command>] for usage.\n" );
        return;
    }

    twiPort = (TWI_SIMPLE_PORT)strtol(argv[1], NULL, 0);
    twiHandle = NULL;

    if (twiPort >= TWI_END) {
        printf("Invalid I2C port!\n");
        return;
    }

    i2c_addr = strtol(argv[2], NULL, 0);
    reg_addr = strtol(argv[3], NULL, 0);
    twiWrLen = str2bytes(argv[4], &wdata);

    length = strtol(argv[5], NULL, 0);
    if (length < 0) {
        printf("Invalid read length!\n");
        return;
    }

    addrLength = (reg_addr > 255) ? 2 : 1;
    if (argc > 6) {
        addrLength = strtol(argv[6], NULL, 0);
        if ((addrLength < 1) || (addrLength > 2)) {
            printf("Invalid address length!\n");
            return;
        }
    }

    /* Allocate a buffer to read into */
    if (length) {
        twiRdBuffer = SHELL_MALLOC(length);
    }

    /* Allocate a write buffer */
    twiWrBuffer = SHELL_MALLOC(addrLength + twiWrLen);

    /* See if requested TWI port is already open globally */
    if ((twiPort == TWI0) && context->twi0Handle) {
        twiHandle = context->twi0Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI1) && context->twi1Handle) {
        twiHandle = context->twi1Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI2) && context->twi2Handle) {
        twiHandle = context->twi2Handle;
        result = TWI_SIMPLE_SUCCESS;
    }

    if (twiHandle != NULL) {

        /* Print header if reading */
        if (length > 0) {
            printf ( "I2C Device (0x%02x): addr 0x%04x, bytes read %d (0x%02x), ",
                i2c_addr, reg_addr, length, length );

            printf ( (addrLength == 2) ? "2-byte address length\n" : "1-byte address length\n" );
        }

        /* Do write/read of peripheral at the specified address */
        if (result == TWI_SIMPLE_SUCCESS) {
            if (addrLength == 2) {
                twiWrBuffer[0] = (reg_addr >> 8) & 0xFF;
                twiWrBuffer[1] = (reg_addr >> 0) & 0xFF;
            } else {
                twiWrBuffer[0] = reg_addr;
            }
            memcpy(twiWrBuffer + addrLength, wdata, twiWrLen);
            twiWrLen += addrLength;
            result = twi_writeRead(twiHandle, i2c_addr, twiWrBuffer, twiWrLen, twiRdBuffer, length);
            if (result == TWI_SIMPLE_SUCCESS) {
                if (length > 0) {
                    dumpBytes(ctx, twiRdBuffer, reg_addr, length);
                }
            } else {
                printf("twi twi_writeRead() error %d\n", result);
            }
        }
    } else {
        printf("I2C port %d is not configured in this project!\n", twiPort);
    }

    /* Free the write buffers */
    if (wdata) {
        SHELL_FREE(wdata);
    }
    if (twiWrBuffer) {
        SHELL_FREE(twiWrBuffer);
    }

    /* Free the read buffer */
    if (twiRdBuffer) {
        SHELL_FREE(twiRdBuffer);
    }
}

/***********************************************************************
 * CMD: i2c_probe
 **********************************************************************/
const char shell_help_i2c_probe[] = "<i2c_port>\n"
  "  i2c_port - I2C port to probe\n";
const char shell_help_summary_i2c_probe[] = "Probe an I2C port for active devices";

void shell_i2c_probe(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    sTWI *twiHandle;
    TWI_SIMPLE_RESULT result;
    TWI_SIMPLE_PORT twiPort;
    int i;

    if (argc != 2) {
        printf( "Invalid arguments. Type help [<command>] for usage.\n" );
        return;
    }

    twiPort = (TWI_SIMPLE_PORT)strtol(argv[1], NULL, 0);
    twiHandle = NULL;

    if (twiPort >= TWI_END) {
        printf("Invalid I2C port!\n");
        return;
    }

    /* See if requested TWI port is already open globally */
    if ((twiPort == TWI0) && context->twi0Handle) {
        twiHandle = context->twi0Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI1) && context->twi1Handle) {
        twiHandle = context->twi1Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI2) && context->twi2Handle) {
        twiHandle = context->twi2Handle;
        result = TWI_SIMPLE_SUCCESS;
    }

    if (twiHandle != NULL) {
        if (result == TWI_SIMPLE_SUCCESS) {
            printf ( "Probing I2C port %d:\n", twiPort);
            for (i = 0; i < 128; i++) {
                result = twi_write(twiHandle, i, NULL, 0);
                if (result == TWI_SIMPLE_SUCCESS) {
                    printf(" Found device 0x%02x\n", i);
                }
            }
        }
    } else {
        printf("I2C port %d is not configured in this project!\n", twiPort);
    }
}

/***********************************************************************
 * CMD: syslog
 **********************************************************************/
const char shell_help_syslog[] = "\n";
const char shell_help_summary_syslog[] = "Show the live system log";

#include "syslog.h"

#define MAX_TS_LINE  32
#define MAX_LOG_LINE 256

void shell_syslog(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    char *line;
    char *lbuf;
    char *ts;
    int c;

    ts = SHELL_MALLOC(MAX_TS_LINE);
    lbuf = SHELL_MALLOC(MAX_LOG_LINE);

    c = 0;
    do {
        line = syslog_next(ts, MAX_TS_LINE, lbuf, MAX_LOG_LINE);
        if (line) {
            printf("%s %s\n", ts, line);
        }
        c = term_getch(&ctx->t, TERM_INPUT_DONT_WAIT);
#ifdef FREE_RTOS
        if ((line == NULL) && (c < 0)) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
#endif
    } while (c < 0);

    if (ts) {
        SHELL_FREE(ts);
    }
    if (lbuf) {
        SHELL_FREE(lbuf);
    }
}

/***********************************************************************
 * CMD: drive
 **********************************************************************/
#include "fs_devman.h"

const char shell_help_drive[] = "<device> <action>\n"
  "  device - The device to take action on\n"
  "  action - The action to take on the device\n"
  " Valid actions\n"
  "  default - Sets the requested device as the default file system\n"
  " No arguments\n"
  "  Show all available devices\n";
const char shell_help_summary_drive[] = "Shows/supports filesystem device information";

void shell_drive(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    char *device;
    char *action;
    char *dirName;
    unsigned int dirIdx;
    const char *pcDeviceDefault;

    /* Local Inits */
    pcDeviceDefault = NULL;

    if((argc == 2) || (argc > 3))
    {
       printf("Invalid arguments. Type help [<command>] for usage.\n");
       return;
    }

    /* Get the default device */
    fs_devman_get_default(&pcDeviceDefault);

    if(argc == 1)
    {
        printf("Available devices are:\n");
        for(dirIdx = 0u; dirIdx < fs_devman_get_num_devices(); dirIdx++)
        {
            if(fs_devman_get_deviceName(dirIdx, (const char **)&dirName) == FS_DEVMAN_OK)
            {
                printf("%s %s\n",
                    dirName, (pcDeviceDefault == dirName) ? "(Default)" : "");
            }
        }
    }
    else
    {
        /* Get the device and action and verify the arguments */
        device = argv[1];
        action = argv[2];

        if(fs_devman_is_device_valid(device) == true)
        {
            if(strcmp(action, "default") == 0)
            {
                if(fs_devman_set_default(device) == FS_DEVMAN_OK)
                {
                   printf("Succesfully set %s to default drive!\n", device);
                }
                else
                {
                   printf("Could not set %s to default drive!\n", device);
                }
            }
            else
            {
                printf("Invalid action. Type help [<command>] for usage.\n");
            }
        }
    }
}

/***********************************************************************
 * CMD: ls
 **********************************************************************/
const char shell_help_ls[] = "<device>\n";
const char shell_help_summary_ls[] = "Shows a device directory listing";

static void shell_ls_helper( SHELL_CONTEXT *ctx, const char *crtname, int recursive, int *phasdirs )
{
  void *d;
  uint32_t total = 0;
  FS_DEVMAN_DIRENT *ent;
  int ndirs = 0;
  unsigned year ,month, day, hour, min, sec;

  if( ( d = fs_devman_opendir( crtname ) ) != NULL )
  {
    total = 0;
    printf( "%s", crtname );
    while( ( ent = fs_devman_readdir( d ) ) != NULL )
    {
      printf("\n");
      if( ent->flags & FS_DEVMAN_DIRENT_FLAG_DIR )
      {
        printf( "%12s ", "<DIR>" );
        ndirs = ndirs + 1;
        if( phasdirs )
          *phasdirs = 1;
      }
      else
      {
        printf( "%12u ", ( unsigned )ent->fsize );
        total = total + ent->fsize;
      }
      year = ((ent->fdate >> 9) & 0x7F) + 1980;
      month = (ent->fdate >> 5) & 0xF;
      day = (ent->fdate >> 0) & 0x1F;
      printf("%04u-%02u-%02u ", year, month, day);
      hour = (ent->ftime >> 11) & 0x1F;
      min = (ent->ftime >> 5) & 0x3F;
      sec = ((ent->ftime >> 0) & 0x1F) * 2;
      printf("%02u:%02u:%02u ", hour, min, sec);
      printf( " %s", ent->fname );
    }
    fs_devman_closedir( d );
    printf("\n");
  }
}

void shell_ls(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    FS_DEVMAN_RESULT result;
    int phasdirs;
    char *device;

    if (argc == 1) {
        result = fs_devman_get_default((const char **)&device);
        if (result != FS_DEVMAN_OK) {
            return;
        }
    } else {
        device = argv[1];
    }

    shell_ls_helper(ctx, device, 0, &phasdirs );
}

/***********************************************************************
 * CMD: format
 **********************************************************************/
const char shell_help_format[] = "["
    SPIFFS_VOL_NAME
#if defined(EMMC_VOL_NAME) && defined(SDCARD_USE_EMMC)
    " | " EMMC_VOL_NAME
#endif
    "]\n";
const char shell_help_summary_format[] = "Formats an internal flash filesystem";

#include "fs_devman.h"

#include "spiffs_fs.h"
#include "fs_dev_spiffs.h"
#if defined(EMMC_VOL_NAME) && defined(SDCARD_USE_EMMC)
#include "ff.h"
#endif

void shell_format(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    FS_DEVMAN_RESULT fsdResult;
    FS_DEVMAN_DEVICE *device;
    const char *ddname;
    int fs = INVALID_FS;
    bool dd = false;

    fs = fsVolOK(ctx, context, (argc > 1) ? argv[1] : "");
    if (fs == INVALID_FS) {
        return;
    }

    printf("Be patient, this may take a while.\n");
    printf("Formatting...\n");

    if (fs == SPIFFS_FS) {
        if (fs_devman_is_device_valid(SPIFFS_VOL_NAME)) {
            fsdResult = fs_devman_get_default(&ddname);
            if (fsdResult == FS_DEVMAN_OK) {
                dd = (strcmp(ddname, SPIFFS_VOL_NAME) == 0);
            }
            fs_devman_unregister(SPIFFS_VOL_NAME);
        }
        spiffs_format(context->spiffsHandle);
        device = fs_dev_spiffs_device();
        fsdResult = fs_devman_register(SPIFFS_VOL_NAME, device, context->spiffsHandle);
        if (dd) {
            fsdResult = fs_devman_set_default(SPIFFS_VOL_NAME);
        }
    }
#if defined(EMMC_VOL_NAME) && defined(SDCARD_USE_EMMC)
    if (fs == EMMC_FS) {
        FRESULT res;
        MKFS_PARM opt = {
            .fmt = FM_FAT32
        };
        res = f_mkfs(EMMC_VOL_NAME, &opt, NULL, 512*16);
        if (res == FR_OK) {
            context->sdRemount = true;
        }
    }
#endif
    if (fs == SD_FS) {
        printf("Not supported.\n");
    }

    printf("Done.\n");
}

/***********************************************************************
 * CMD: discover
 **********************************************************************/
#include "init.h"
#include "a2b_xml.h"
#include "adi_a2b_cmdlist.h"
#include "a2b_irq.h"

const char shell_help_discover[] = "<a2b.xml> <verbose> <i2c_port> <i2c_addr>\n"
  "  a2b.xml  - A SigmaStudio A2B XML config export file\n"
  "             default 'a2b.xml'\n"
  "  verbose  - Print out results to 0:none, 1:stdout, 2:syslog\n"
  "             default: 1\n"
  "  i2c_port - Set the AD242x transceiver I2C port.\n"
  "             default: 0 (TWI0)\n"
  "  i2c_addr - Set the AD242x transceiver I2C address\n"
  "             default: 0x68\n";
const char shell_help_summary_discover[] = "Discovers an A2B network";

static ADI_A2B_CMDLIST_RESULT shell_discover_twi_read(
    void *twiHandle, uint8_t address,
    void *in, uint16_t inLen, void *usr)
{
    TWI_SIMPLE_RESULT twiResult;
    twiResult = twi_read(twiHandle, address, in, inLen);
    return (twiResult == TWI_SIMPLE_SUCCESS ?
        ADI_A2B_CMDLIST_SUCCESS : ADI_A2B_CMDLIST_A2B_I2C_READ_ERROR);
    }

static ADI_A2B_CMDLIST_RESULT shell_discover_twi_write(
    void *twiHandle, uint8_t address,
    void *out, uint16_t outLen, void *usr)
{
    TWI_SIMPLE_RESULT twiResult;
    twiResult = twi_write(twiHandle, address, out, outLen);
    return (twiResult == TWI_SIMPLE_SUCCESS ?
        ADI_A2B_CMDLIST_SUCCESS : ADI_A2B_CMDLIST_A2B_I2C_WRITE_ERROR);
}

static ADI_A2B_CMDLIST_RESULT shell_discover_twi_write_write(
    void *twiHandle, uint8_t address,
    void *out, uint16_t outLen, void *out2, uint16_t out2Len, void *usr)
{
    TWI_SIMPLE_RESULT twiResult;
    twiResult = twi_writeWrite(twiHandle, address, out, outLen, out2, out2Len);
    return (twiResult == TWI_SIMPLE_SUCCESS ?
        ADI_A2B_CMDLIST_SUCCESS : ADI_A2B_CMDLIST_A2B_I2C_WRITE_ERROR);
}

static ADI_A2B_CMDLIST_RESULT shell_discover_twi_write_read(
    void *twiHandle, uint8_t address,
    void *out, uint16_t outLen, void *in, uint16_t inLen, void *usr)
{
    TWI_SIMPLE_RESULT twiResult;
    twiResult = twi_writeRead(twiHandle, address, out, outLen, in, inLen);
    return (twiResult == TWI_SIMPLE_SUCCESS ?
        ADI_A2B_CMDLIST_SUCCESS : ADI_A2B_CMDLIST_A2B_I2C_WRITE_ERROR);
}

static void shell_discover_delay(uint32_t mS, void *usr)
{
    vTaskDelay(pdMS_TO_TICKS(mS));
}

static uint32_t shell_discover_get_time(void *usr)
{
    return(xTaskGetTickCount() * (1000 / configTICK_RATE_HZ));
}

static void *shell_discover_get_buffer(uint16_t size, void *usr)
{
    return(SHELL_MALLOC(size));
}

static void shell_discover_free_buffer(void *buffer, void *usr)
{
    SHELL_FREE(buffer);
}

static void shell_discover_log (
    bool newLine, void *usr, const char *fmt, va_list va)
{
    static char *logline = NULL;
    char *str = NULL;
    char *l;
    va_list _va;
    int len;

    /* Flush if newline or done */
    if (newLine || (fmt == NULL)) {
        if (logline) {
            syslog_print(logline);
            SHELL_FREE(logline);
            logline = NULL;
        }
    }

    /* Make new string */
    if (fmt) {
        _va = va;
        len = vsnprintf(NULL, 0, fmt, _va);
        _va = va;
        str = SHELL_MALLOC(len + 1);
        vsnprintf(str, len + 1, fmt, _va);
    }

    /* Concat */
    if (str) {
        len = logline ? strlen(logline) : 0;
        len += strlen(str) + 1;
        l = SHELL_REALLOC(logline, len);
        if (logline == NULL) {
            *l = '\0';
        }
        logline = l;
        strcat(logline, str);
    }

    /* Free */
    if (str) {
        SHELL_FREE(str);
    }
}


typedef int (*PF)(SHELL_CONTEXT *ctx, const char *restrict format, ...);

int shell_syslog_vprintf(SHELL_CONTEXT *ctx, const char *restrict format, ...)
{
    va_list va;
    va_start(va, format);
    syslog_vprintf((char *)format, va);
    va_end(va);
    return(0);
}

void shell_discover(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    const char *fileName = "a2b.xml";
    ADI_A2B_CMDLIST_RESULT cmdListResult;
    void *a2bInitSequence;
    uint32_t a2bIinitLength;
    int verbose;
    sTWI *twiHandle;
    TWI_SIMPLE_RESULT result;
    TWI_SIMPLE_PORT twiPort;
    uint8_t ad2425I2CAddr;
    A2B_CMD_TYPE a2bCmdType;
    ADI_A2B_CMDLIST *list;
    ADI_A2B_CMDLIST_EXECUTE_INFO execInfo;
    ADI_A2B_CMDLIST_SCAN_INFO scanInfo;
    ADI_A2B_CMDLIST_OVERRIDE_INFO overrideInfo;
    PF pf;

    ADI_A2B_CMDLIST_CFG cfg = {
        .twiRead = shell_discover_twi_read,
        .twiWrite = shell_discover_twi_write,
        .twiWriteRead = shell_discover_twi_write_read,
        .twiWriteWrite = shell_discover_twi_write_write,
        .delay = shell_discover_delay,
        .getTime = shell_discover_get_time,
        .getBuffer = shell_discover_get_buffer,
        .freeBuffer = shell_discover_free_buffer,
        .log = shell_discover_log,
        .usr = ctx
    };

    /* Determine file name */
    if (argc >= 2) {
        fileName = (const char *)argv[1];
    } else {
        fileName = "a2b.xml";
    }

    /* Determine verbosity */
    if (argc >= 3) {
        verbose = strtol(argv[2], NULL, 0);
    } else {
        verbose = 1;
    }
    if (verbose == 1) {
        pf = shell_printf;
    } else if (verbose == 2) {
        pf = shell_syslog_vprintf;
    } else {
        pf = NULL;
    }

    /* Determine TWI port */
    if (argc >= 4) {
        twiPort = (TWI_SIMPLE_PORT)strtol(argv[3], NULL, 0);
    } else {
        twiPort = TWI0;
    }
    if (twiPort >= TWI_END) {
        if (pf) {
            pf(ctx, "Invalid I2C port!\n");
        }
        return;
    }

    /* See if requested TWI port is already open globally */
    twiHandle = NULL;
    if ((twiPort == TWI0) && context->twi0Handle) {
        twiHandle = context->twi0Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI1) && context->twi1Handle) {
        twiHandle = context->twi1Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI2) && context->twi2Handle) {
        twiHandle = context->twi2Handle;
        result = TWI_SIMPLE_SUCCESS;
    }
    if (twiHandle == NULL) {
        if (pf) {
            pf(ctx, "I2C port %d is not configured in this project!\n", twiPort);
        }
        return;
    }

    /* Save the TWI handle to use */
    cfg.handle = (void *)twiHandle;

    /* Determine AD2425 I2C address */
    if (argc >= 5) {
        ad2425I2CAddr = strtol(argv[4], NULL, 0);
    } else {
        ad2425I2CAddr = context->cfg.a2bI2CAddr;
    }

    /* Load the A2B network init */
    a2bIinitLength = a2b_xml_load(fileName, &a2bInitSequence, &a2bCmdType);

    /* If successful, play out the binary init sequence */
    if (a2bIinitLength && a2bInitSequence) {

        /* Open a command list instance */
        cmdListResult = adi_a2b_cmdlist_open(&list, &cfg);

        /* Set the command list */
        cmdListResult =  adi_a2b_cmdlist_set(
            list, 0x68, a2bInitSequence, a2bIinitLength, a2bCmdType
        );

        /* Clear the overrides */
        memset(&overrideInfo, 0, sizeof(overrideInfo));

        /* Override default SigmaStudio address */
        overrideInfo.masterAddr_override = true;
        overrideInfo.masterAddr = ad2425I2CAddr;

        /* Confirm master I2S/TDM settings and override if they don't match */
        cmdListResult = adi_a2b_cmdlist_scan(
            list, &scanInfo
        );
        if (scanInfo.I2SGCFG_valid && (scanInfo.I2SGCFG != SYSTEM_I2SGCFG)) {
            if (pf) {
                pf(ctx, "WARNING: I2SGCFG mismatch (expected %02x, got %02x)\n",
                    SYSTEM_I2SGCFG, scanInfo.I2SGCFG);
                pf(ctx, "         Overriding...\n");
            }
            overrideInfo.I2SGCFG_override = true;
            overrideInfo.I2SGCFG = SYSTEM_I2SGCFG;
        }
        if (scanInfo.I2SCFG_valid && (scanInfo.I2SCFG != SYSTEM_I2SCFG)) {
            if (pf) {
                pf(ctx, "WARNING: I2SCFG mismatch (expected %02x, got %02x)\n",
                    SYSTEM_I2SCFG, scanInfo.I2SCFG);
                pf(ctx, "         Overriding...\n");
            }
            overrideInfo.I2SCFG_override = true;
            overrideInfo.I2SCFG = SYSTEM_I2SCFG;
        }

        /* Process any overrides */
        cmdListResult = adi_a2b_cmdlist_override(list, &overrideInfo);

        /* Disable A2B IRQ processing */
        a2b_irq_disable(context, A2B_BUS_NUM_1);

        /* Reset the transceiver and delay */
        static uint8_t AD2425_RESET[] = { 0x12, 0x84};
        cmdListResult = adi_a2b_cmdlist_node_twi_transfer(
            list, -1, false, false, 0,
            AD2425_RESET, sizeof(AD2425_RESET), NULL, 0, false);
        cmdListResult = adi_a2b_cmdlist_delay(list, 100);

        /* Run the command list */
        cmdListResult = adi_a2b_cmdlist_execute(list, &execInfo);

        /* Reenable A2B IRQ processing */
        a2b_irq_enable(context, A2B_BUS_NUM_1);

        if (pf) {
            pf(ctx, "A2B config lines processed: %lu\n", execInfo.linesProcessed);
            pf(ctx, "A2B discovery result: %s\n", execInfo.resultStr);
            pf(ctx, "A2B nodes discovered: %d\n", execInfo.nodesDiscovered);
        }

        /* Close the command list */
        cmdListResult = adi_a2b_cmdlist_close(&list);

        /* Free the network config */
        a2b_xml_free(a2bInitSequence, a2bIinitLength, a2bCmdType);

    } else {
        if (pf) {
            pf(ctx, "Error loading '%s' A2B init XML file\n", fileName);
        }
    }

}

/***********************************************************************
 * CMD: df
 **********************************************************************/
const char shell_help_df[] = "["
    SPIFFS_VOL_NAME
#if defined(EMMC_VOL_NAME) && defined(SDCARD_USE_EMMC)
    " | " EMMC_VOL_NAME
#elif defined(SDCARD_VOL_NAME)
    " | " SDCARD_VOL_NAME
#endif
    "]\n";
const char shell_help_summary_df[] = "Shows internal filesystem disk full status";

#if defined(EMMC_VOL_NAME) && defined(SDCARD_USE_EMMC)
#define DF_SD_EMMC_VOL_NAME  EMMC_VOL_NAME
#elif defined(SDCARD_VOL_NAME)
#define DF_SD_EMMC_VOL_NAME  SDCARD_VOL_NAME
#endif

#include "spiffs.h"
#ifdef DF_SD_EMMC_VOL_NAME
#include "ff.h"
#endif

void shell_df( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    int fs = INVALID_FS;
    int sf = 0;
    int emmc = 0;
    int sd = 0;

    if (argc > 1) {
        fs = fsVolOK(ctx, context, (argc > 1) ? argv[1] : SPIFFS_VOL_NAME);
        if (fs == INVALID_FS) {
            return;
        }
        sf = (fs == SPIFFS_FS); sd = (fs == SD_FS); emmc = (fs == EMMC_FS);
    } else {
        sf = 1; sd = 1; emmc = 1;
    }

    printf("%-10s %10s %10s %10s %5s\n", "Filesystem", "1K-blocks", "Used", "Available", "Use %");

    if (sf) {
        s32_t serr; u32_t ssize; u32_t sused;
        serr = SPIFFS_info(context->spiffsHandle, &ssize, &sused);
        if (serr == SPIFFS_OK) {
          ssize /= 1024; sused /= 1024;
          printf("%-10s %10u %10u %10u %5u\n", SPIFFS_VOL_NAME,
            (unsigned)ssize, (unsigned)sused, (unsigned)(ssize - sused),
            (unsigned)((100 * sused) / ssize));
        }
    }
#ifdef DF_SD_EMMC_VOL_NAME
    if (sd || emmc) {
        FRESULT res;
        FATFS *fs;
        DWORD fre_clust, fre_blk, tot_blk, used_blk;
        res = f_getfree(DF_SD_EMMC_VOL_NAME, &fre_clust, &fs);
        tot_blk = ((fs->n_fatent - 2) * fs->csize)/2;
        fre_blk = (fre_clust * fs->csize)/2;
        used_blk = tot_blk - fre_blk;
        if (res == FR_OK) {
            printf("%-10s %10u %10u %10u %5u\n", DF_SD_EMMC_VOL_NAME,
                (unsigned)tot_blk, (unsigned)used_blk, (unsigned)fre_blk,
                (unsigned)((100 * used_blk) / tot_blk));
        }
    }
#endif
}

/***********************************************************************
 * CMD: rm/del
 **********************************************************************/
const char shell_help_rm[] = "<file1> [<file2> ...]\n";
const char shell_help_summary_rm[] = "Removes a file";

#include <stdio.h>

void shell_rm( SHELL_CONTEXT *ctx, int argc, char **argv )
{
  int i;

  if (argc < 2) {
    printf( "Usage: rm <file1> [<file2> ...]\n" );
    return;
  }

  for (i = 1; i < argc; i++) {
    if (remove(argv[i]) != 0) {
      printf("Unable to remove '%s'\n", argv[i]);
    }
  }
}

/***********************************************************************
 * CMD: cat
 **********************************************************************/
const char shell_help_cat[] = "\n";
const char shell_help_summary_cat[] = "Print file on standard output";

void shell_cat( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    FILE *handle;
    unsigned i;
    unsigned char c;

    if( argc < 2 ) {
        printf( "Usage: cat <filename1> [<filename2> ...]\n" );
        return;
    }
    for( i = 1; i < argc; i ++ ) {
        if( ( handle = fopen( argv[ i ], "r" ) ) > 0 )
        {
            while (fread(&c, sizeof(c), 1, handle) > 0) {
                printf("%c", c);
            }
            fclose(handle);
        } else {
            printf( "Unable to open '%s'\n", argv[ i ] );
        }
    }
}

/***********************************************************************
 * CMD: copy/cp
 **********************************************************************/
const char shell_help_cp[] = "<src> <dst>\n";
const char shell_help_summary_cp[] = "Copy source file <src> to <dst>";

#define SHELL_COPY_BUFSIZE    256

void shell_cp( SHELL_CONTEXT *ctx, int argc, char **argv )
{
   FILE *fps = NULL, *fpd = NULL;
   void *buf = NULL;
   size_t datalen, datawrote, total = 0;

   if( argc != 3 ) {
      printf( "Usage: cp <source> <destination>\n" );
      return;
   }

   if( ( fps = fopen( argv[ 1 ], "r" ) ) == NULL ) {
      printf( "Unable to open %s for reading\n", argv[ 1 ] );
   } else {
      if( ( fpd = fopen( argv[ 2 ], "w" ) ) == NULL ) {
         printf( "Unable to open %s for writing\n", argv[ 2 ] );
      } else {
         if( ( buf = SHELL_MALLOC( SHELL_COPY_BUFSIZE ) ) == NULL ) {
            printf( "Not enough memory\n" );
         } else {
            while( 1 ) {
               datalen = fread( buf, 1, SHELL_COPY_BUFSIZE, fps );
               datawrote = fwrite( buf, 1, datalen, fpd );
               if( datawrote < datalen ) {
                  printf( "Copy error (no space left on target?)\n" );
                  break;
               }
               total += datalen;
               if( datalen < SHELL_COPY_BUFSIZE ) {
                  break;
               }
            }
            fflush( fpd );
            printf( "%u bytes copied\n", ( unsigned int )total );
         }
      }
   }

   if( fps ) {
      fclose( fps );
   }
   if( fpd ) {
      fclose( fpd );
   }
   if( buf ) {
      SHELL_FREE( buf );
   }
}


/***********************************************************************
 * CMD: run
 **********************************************************************/
const char shell_help_run[] =
    "[options] <file1> [<file2> ...]\n"
    "  Valid options\n"
    "    -l <loop count> - Runs cmd file(s) \"loop count\" times\n"
    "                      Values: Greater than 0\n";
const char shell_help_summary_run[] = "Runs a command file";

void shell_run( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    FILE *f = NULL;
    char *cmd = NULL;
    char *str = NULL;
    bool ok = true;
    int i;
    int c = -1;
    int loopCount = 1;
    int lineCount;
    bool printedHelp = false;

    struct option_data opt_data = OPT_DATA_INIT;
    int opt;
    int opt_offset;

    /* Parse options. */
    while ((opt = _getopt(argc, argv, ":l:", &opt_data)) != -1) {
        switch(opt) {
            case 'l':
                loopCount = strtol(opt_data.optarg, NULL, 0);
                if (loopCount <= 0) {
                    ok = false;
                    printf("Value must be a positive integer\n");
                }
                break;
            case '?':
                ok = false;
                printf("Option '%c' is not a valid option\n", opt_data.optopt);
                break;
            case ':':
                ok = false;
                printf("Option '%c' requires an argument\n", opt_data.optopt);
                break;
            default:
                ok = false;
                printf("Error parsing options\n");
                break;
        }
    }
    if (!ok) {
        return;
    }

    opt_offset = opt_data.optind - 1;

    if (argc < (opt_offset + 2)) {
        printf( "Usage: run <file1> [<file2> ...]\n" );
        return;
    }

    cmd = SHELL_MALLOC(SHELL_MAX_LINE_LEN);

    while((loopCount > 0) && (c != KC_CTRL_C)) {
        for (i = (opt_offset + 1); i < argc; i++) {
            f = fopen(argv[i], "r");
            if (f) {
                str = NULL;
                lineCount = 0;
                do {
                    str = fgets(cmd, SHELL_MAX_LINE_LEN, f);
                    if (str == cmd) {
                        lineCount++;
                        if (cmd[0] == ';' || cmd[0] == '#') {
                            continue;
                        }
                        shell_exec(ctx, cmd);
                    }
                    c = term_getch(&ctx->t, TERM_INPUT_DONT_WAIT);
                    if((c > 0) && (c != KC_CTRL_C) && !printedHelp) {
                        printf("Press CTRL + C to abort run command\n");
                        printedHelp = true;
                    }
                } while (str && (c != KC_CTRL_C));
                fclose(f);
                if(c == KC_CTRL_C) {
                    printf("Run command aborted %s:%d\n", argv[i], lineCount);
                    break;
                }
            } else {
                if (ctx->interactive) {
                    printf("Failed to open '%s'\n", argv[i]);
                } else {
                    syslog_printf("Failed to open '%s'\n", argv[i]);
                }
            }
        }
        loopCount--;
    }

    SHELL_FREE(cmd);
}

/***********************************************************************
 * CMD: stacks
 **********************************************************************/
const char shell_help_stacks[] = "\n";
const char shell_help_summary_stacks[] = "Report task stack usage";

static void shell_print_task_stack(SHELL_CONTEXT *ctx, TaskHandle_t task)
{
    if (task) {
        printf(" %s: %u\n",
            pcTaskGetName(task), (unsigned)uxTaskGetStackHighWaterMark(task)
        );
    }
}

void shell_stacks( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    APP_CONTEXT *context = &mainAppContext;

    printf("High Water Marks are in 32-bit words (zero is bad).\n");
    printf("Task Stask High Water Marks:\n");

    shell_print_task_stack(ctx, context->startupTaskHandle);
    shell_print_task_stack(ctx, context->houseKeepingTaskHandle);
    shell_print_task_stack(ctx, context->pollStorageTaskHandle);
    shell_print_task_stack(ctx, context->pushButtonTaskHandle);
    shell_print_task_stack(ctx, context->uac2TaskHandle);
    shell_print_task_stack(ctx, context->wavSrcTaskHandle);
    shell_print_task_stack(ctx, context->wavSinkTaskHandle);
    shell_print_task_stack(ctx, context->a2bSlaveTaskHandle);
    shell_print_task_stack(ctx, context->a2bIrqTaskHandle);
    shell_print_task_stack(ctx, context->telnetTaskHandle);
    shell_print_task_stack(ctx, context->vuTaskHandle);
    shell_print_task_stack(ctx, context->rtpRxTaskHandle);
    shell_print_task_stack(ctx, context->rtpTxTaskHandle);
    shell_print_task_stack(ctx, context->vbanRxTaskHandle);
    shell_print_task_stack(ctx, context->vbanTxTaskHandle);
}

/***********************************************************************
 * CMD: cpu
 **********************************************************************/
const char shell_help_cpu[] = "\n";
const char shell_help_summary_cpu[] = "Report cpu usage";

#include "cpu_load.h"
#include "clock_domain.h"

void shell_cpu( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    uint32_t percentCpuLoad, maxCpuLoad;
    int i;

    percentCpuLoad = cpuLoadGetLoad(&maxCpuLoad, true);
    printf("ARM CPU Load: %u%% (%u%% peak)\n",
        (unsigned)percentCpuLoad, (unsigned)maxCpuLoad);

    printf("SHARC0 Load:\n");
    for (i = 0; i < CLOCK_DOMAIN_MAX; i++) {
        printf(" %s: %lu cycles\n", clock_domain_str(i), context->sharc0Cycles[i]);
    }

    printf("SHARC1 Load:\n");
    for (i = 0; i < CLOCK_DOMAIN_MAX; i++) {
        printf(" %s: %lu cycles\n", clock_domain_str(i), context->sharc1Cycles[i]);
    }
}

/***********************************************************************
 * CMD: usb
 **********************************************************************/
#include "buffer_track.h"
#include "cpu_load.h"
#include "clocks.h"
#include "clock_domain.h"

const char shell_help_usb[] = " [ [in|out| [domain [a2b|system] ] ] [reset] ] \n";
const char shell_help_summary_usb[] = "View/set/clear runtime USB settings/metrics";

void shell_usb( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    bool showIn = true;
    bool showOut = true;
    int clockDomainMask;

    if (argc >= 2) {
        if (strcmp(argv[1], "out") == 0) {
            showIn = false;
            clockDomainMask = CLOCK_DOMAIN_BITM_USB_RX;
        } else if (strcmp(argv[1], "in") == 0) {
            showOut = false;
            clockDomainMask = CLOCK_DOMAIN_BITM_USB_TX;
        }
    }

    if (argc >= 3) {
        if (strcmp(argv[2], "domain") == 0) {
            if (argc >= 4) {
                if (strcmp(argv[3], "a2b") == 0) {
                    clock_domain_set(context, CLOCK_DOMAIN_A2B, clockDomainMask);
                } else if (strcmp(argv[3], "system") == 0) {
                    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, clockDomainMask);
                } else {
                    printf("Bad domain\n");
                }
                return;
            } else {
                printf("No domain\n");
            }
            return;
        }
        if (strcmp(argv[2], "reset") == 0) {
            if (showOut) {
                uac2_reset_stats(UAC2_DIR_OUT);
            }
            if (showIn) {
                uac2_reset_stats(UAC2_DIR_IN);
            }
            return;
        }
    }

    /* USB OUT Stats */
    if (showOut) {
        printf("USB OUT (Rx):\n");
        printf("  Total Pkts: %u\n", (unsigned)context->uac2stats.rx.ep.count);
        printf("  Overruns: %u\n", (unsigned)context->uac2stats.rx.usbRxOverRun);
        printf("  Underruns: %u\n", (unsigned)context->uac2stats.rx.usbRxUnderRun);
        printf("  Last Pkt Time: %uuS\n",
            (unsigned)cpuLoadCyclesToMicrosecond(context->uac2stats.rx.ep.lastPktTime)
        );
        printf("  Max Pkt Time: %uuS\n",
            (unsigned)cpuLoadCyclesToMicrosecond(context->uac2stats.rx.ep.maxPktTime)
        );
        printf("  Last Pkt Size: %u (%u, %u)\n",
            (unsigned)context->uac2stats.rx.ep.lastPktSize,
            (unsigned)context->uac2stats.rx.ep.minPktSize,
            (unsigned)context->uac2stats.rx.ep.maxPktSize
        );

        printf("  Buffer Fill: %u\n",
            (unsigned)bufferTrackGetFrames(UAC2_OUT_BUFFER_TRACK_IDX, context->cfg.usbOutChannels));
        printf("  Sample Rate Feedback: %u\n",
            (unsigned)bufferTrackGetSampleRate(UAC2_OUT_BUFFER_TRACK_IDX));
        printf("  Clock Domain: %s\n",
            clock_domain_str(clock_domain_get(context, CLOCK_DOMAIN_BITM_USB_RX)));
    }

    /* USB IN Stats */
    if (showIn) {
        printf("USB IN (Tx):\n");
        printf("  Total Pkts: %u\n", (unsigned)context->uac2stats.tx.ep.count);
        printf("    Failed: %u\n", (unsigned)context->uac2stats.tx.ep.failed);
        printf("    Aborted: %u\n", (unsigned)context->uac2stats.tx.ep.aborted);
        printf("    OK: %u\n", (unsigned)context->uac2stats.tx.ep.ok);
        printf("  Overruns: %u\n", (unsigned)context->uac2stats.tx.usbTxOverRun);
        printf("  Underruns: %u\n", (unsigned)context->uac2stats.tx.usbTxUnderRun);
        printf("  Last Pkt Time: %uuS\n",
            (unsigned)cpuLoadCyclesToMicrosecond(context->uac2stats.tx.ep.lastPktTime)
        );
        printf("  Max Pkt Time: %uuS\n",
            (unsigned)cpuLoadCyclesToMicrosecond(context->uac2stats.tx.ep.maxPktTime)
        );
        printf("  Last Pkt Size: %u (%u, %u)\n",
            (unsigned)context->uac2stats.tx.ep.lastPktSize,
            (unsigned)context->uac2stats.tx.ep.minPktSize,
            (unsigned)context->uac2stats.tx.ep.maxPktSize
        );
        printf("  Buffer Fill: %u\n", (unsigned)bufferTrackGetFrames(1, context->cfg.usbInChannels));
        printf("  Clock Domain: %s\n",
            clock_domain_str(clock_domain_get(context, CLOCK_DOMAIN_BITM_USB_TX)));
    }
}

/***********************************************************************
 * CMD: fsck
 **********************************************************************/
const char shell_help_fsck[] = "["SPIFFS_VOL_NAME"]\n";
const char shell_help_summary_fsck[] = "Check the internal filesystem";

#include "spiffs.h"

void shell_fsck(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    int fs = INVALID_FS;
    s32_t ok;

    fs = fsVolOK(ctx, context, (argc > 1) ? argv[1] : SPIFFS_VOL_NAME);
    if (fs == INVALID_FS) {
        return;
    }

    printf("Be patient, this may take a while.\n");
    printf("Checking...\n");

    if (fs == SPIFFS_FS) {
        ok = SPIFFS_check(context->spiffsHandle);
        if (ok == SPIFFS_OK) {
            printf(SPIFFS_VOL_NAME " OK\n");
        } else {
            printf(SPIFFS_VOL_NAME " CORRUPT: %d\n", (int)ok);
        }
    }

    if ((fs == EMMC_FS) || (fs == SD_FS)) {
        printf("Not supported.\n");
    }
}

/***********************************************************************
 * CMD: update
 **********************************************************************/
#include "flash_map.h"

const char shell_help_update[] = "\n";
const char shell_help_summary_update[] = "Updates the firmware via xmodem";

void shell_update(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    char *warn = "Updating the firmware is DANGEROUS - DO NOT REMOVE POWER!";
    FLASH_WRITE_STATE state;
    const FLASH_INFO *flash;
    int oldMode;
    long size;

    /* Confirm action */
    if (confirmDanger(ctx, warn) == 0) {
        return;
    }

    /* Get a handle to the system flash */
    flash = context->flashHandle;
    if (flash == NULL) {
        printf("Flash not initialized!\n");
        return;
    }

    /* Configure the update */
    state.flash = flash;
    state.addr = APP_OFFSET;
    state.maxAddr = APP_OFFSET + APP_SIZE;
    state.eraseBlockSize = ERASE_BLOCK_SIZE;
    state.xmodem.ctx = ctx;

    /* Start the update */
    printf( "Start XMODEM transfer now... ");
    oldMode = uart_stdio_set_mode(UART_STDIO_MODE_RAW);
    size = XmodemReceiveCrc(flashDataWrite, &state, INT_MAX,
        shell_xmodem_getchar, shell_xmodem_putchar);
    uart_stdio_set_mode(oldMode);

    /* Wait a bit */
    printf("\n"); delay(250);

    /* Display results */
    if (size < 0) {
       printf( "XMODEM Error: %ld\n", size);
    } else {
       printf("Received %ld bytes.\n", size);
    }

    printf("Done.\n");
}

/***********************************************************************
 * CMD: meminfo
 **********************************************************************/
const char shell_help_meminfo[] = "\n";
const char shell_help_summary_meminfo[] = "Displays UMM_MALLOC heap statistics";

#include "umm_malloc_cfg.h"
#include "umm_malloc_heaps.h"
#include "sae.h"

const static char *heapNames[] = UMM_HEAP_NAMES;

void shell_meminfo(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    int i;
    int ok;

    /* UMM Malloc */
    UMM_HEAP_INFO ummHeapInfo;
    for (i = 0; i < UMM_NUM_HEAPS; i++) {
        printf("Heap %s Info:\n", heapNames[i]);
        ok = umm_integrity_check((umm_heap_t)i);
        if (ok) {
            umm_info((umm_heap_t)i, &ummHeapInfo, NULL, 0);
            printf("  Entries: Total  %8i, Allocated %8i, Free %8i\n",
                ummHeapInfo.totalEntries,
                ummHeapInfo.usedEntries,
                ummHeapInfo.freeEntries
            );
            printf("   Blocks: Total  %8i, Allocated %8i, Free %8i\n",
                ummHeapInfo.totalBlocks,
                ummHeapInfo.usedBlocks,
                ummHeapInfo.freeBlocks
            );
            printf("   Contig: Blocks %8i,     Bytes %8i\n",
                ummHeapInfo.maxFreeContiguousBlocks,
                ummHeapInfo.maxFreeContiguousBlocks * umm_block_size()
            );
        }
        printf("  Heap Integrity: %s\n", ok ? "OK" : "Corrupt");
    }


    /* FreeRTOS */
    HeapStats_t rtosHeapStats;
    vPortGetHeapStats(&rtosHeapStats);

    printf("FreeRTOS Info:\n");
    printf("     Free: Blocks %8u,   Current %8u, Min  %8u\n",
        (unsigned)rtosHeapStats.xNumberOfFreeBlocks,
        (unsigned)rtosHeapStats.xAvailableHeapSpaceInBytes,
        (unsigned)rtosHeapStats.xMinimumEverFreeBytesRemaining
    );

    /* SHARC Audio Engine (SAE) */
    SAE_HEAP_INFO saeHeapInfo;
    SAE_RESULT saeOk;

    printf("SHARC Audio Engine Info:\n");
    saeOk = sae_heapInfo(context->saeContext, &saeHeapInfo);
    if (saeOk == SAE_RESULT_OK) {
        printf("  Blocks:  Total  %8u, Allocated %8u, Free %8u\n",
            (unsigned)saeHeapInfo.totalBlocks,
            (unsigned)saeHeapInfo.allocBlocks,
            (unsigned)saeHeapInfo.freeBlocks
        );
        printf("    Size:    Free %8u, Allocated %8u, Max  %8u\n",
            (unsigned)saeHeapInfo.freeSize,
            (unsigned)saeHeapInfo.allocSize,
            (unsigned)saeHeapInfo.maxContigFreeSize
        );
    } else {
        printf(" ERROR!\n");
    }
}

/***********************************************************************
 * CMD: cmdlist
 **********************************************************************/
const char shell_help_cmdlist[] = "<cmdlist.xml> <verbose> <i2c_port>\n"
  "  cmdlist.xml  - A SigmaStudio XML command list file\n"
  "                 default 'cmdlist.xml'\n"
  "  verbose  - Print out results to 0:none, 1:stdout, 2:syslog\n"
  "             default: 1\n"
  "  i2c_port - Set the AD242x transceiver I2C port.\n"
  "             default: 0 (TWI0)\n";
const char shell_help_summary_cmdlist[] = "Plays a SigmaStudio XML command list";

void shell_cmdlist(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    const char *fileName = "cmdlist.xml";
    ADI_A2B_CMDLIST_RESULT cmdListResult;
    void *a2bInitSequence;
    uint32_t a2bIinitLength;
    int verbose;
    sTWI *twiHandle;
    TWI_SIMPLE_RESULT result;
    TWI_SIMPLE_PORT twiPort;
    A2B_CMD_TYPE a2bCmdType;
    ADI_A2B_CMDLIST *list;
    PF pf;

    ADI_A2B_CMDLIST_CFG cfg = {
        .twiRead = shell_discover_twi_read,
        .twiWrite = shell_discover_twi_write,
        .twiWriteRead = shell_discover_twi_write_read,
        .twiWriteWrite = shell_discover_twi_write_write,
        .delay = shell_discover_delay,
        .getTime = shell_discover_get_time,
        .getBuffer = shell_discover_get_buffer,
        .freeBuffer = shell_discover_free_buffer,
        .log = shell_discover_log,
        .usr = context
    };

    /* Determine file name */
    if (argc >= 2) {
        fileName = (const char *)argv[1];
    }

    /* Determine verbosity */
    if (argc >= 3) {
        verbose = strtol(argv[2], NULL, 0);
    } else {
        verbose = 1;
    }
    if (verbose == 1) {
        pf = shell_printf;
    } else if (verbose == 2) {
        pf = shell_syslog_vprintf;
    } else {
        pf = NULL;
    }

    /* Determine TWI port */
    if (argc >= 4) {
        twiPort = (TWI_SIMPLE_PORT)strtol(argv[3], NULL, 0);
    } else {
        twiPort = TWI0;
    }
    if (twiPort >= TWI_END) {
        if (pf) {
            pf(ctx, "Invalid I2C port!\n");
        }
        return;
    }

    /* See if requested TWI port is already open globally */
    twiHandle = NULL;
    if ((twiPort == TWI0) && context->twi0Handle) {
        twiHandle = context->twi0Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI1) && context->twi1Handle) {
        twiHandle = context->twi1Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI2) && context->twi2Handle) {
        twiHandle = context->twi2Handle;
        result = TWI_SIMPLE_SUCCESS;
    }
    if (twiHandle == NULL) {
        if (pf) {
            pf(ctx, "I2C port %d is not configured in this project!\n", twiPort);
        }
        return;
    }

    /* Save the TWI handle to use */
    cfg.handle = (void *)twiHandle;

    /* Load the command list */
    a2bIinitLength = a2b_xml_load(fileName, &a2bInitSequence, &a2bCmdType);

    /* If successful, play out the binary init sequence */
    if (a2bIinitLength && a2bInitSequence) {

        /* Open a command list instance */
        cmdListResult = adi_a2b_cmdlist_open(&list, &cfg);

        /* Set the command list */
        cmdListResult =  adi_a2b_cmdlist_set(
            list, 0, a2bInitSequence, a2bIinitLength, a2bCmdType
        );

        /* Run the command list */
        cmdListResult = adi_a2b_cmdlist_play(list);
        if (cmdListResult != ADI_A2B_CMDLIST_SUCCESS) {
            if (pf) {
                pf(ctx, "Error processing command list\n");
            }
        }

        /* Close the command list */
        adi_a2b_cmdlist_close(&list);

        /* Free the network config */
        a2b_xml_free(a2bInitSequence, a2bIinitLength, a2bCmdType);

    } else {
        if (pf) {
            pf(ctx, "Error loading '%s' A2B init XML file\n", fileName);
        }
    }

}

/***********************************************************************
 * CMD: resize
 **********************************************************************/
const char shell_help_resize[] = "[columns [lines]]\n";
const char shell_help_summary_resize[] = "Resize/Sync terminal window";

#include "term.h"

void shell_resize(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    uint32_t now;
    unsigned cols, lines;

    /* Resync screen size */
    term_sync_size(&ctx->t);
    now = rtosTimeMs();
    while (rtosTimeMs() < (now + 100)) {
        term_getch(&ctx->t, TERM_INPUT_DONT_WAIT);
    }

    /* Get latest size */
    cols = term_get_cols(&ctx->t);
    lines = term_get_lines(&ctx->t);

    /* Set terminal window size */
    if (argc > 1) {
        cols = atoi(argv[1]);
        if (argc > 2) {
            lines = atoi(argv[2]);
        }
        term_set_size(&ctx->t, cols, lines);
    }

    /* Show latest size */
    if (argc == 1) {
        printf("Size: %u x %u\n", cols, lines);
        return;
    }
}

/***********************************************************************
 * CMD: shell
 **********************************************************************/
const char shell_help_shell[] =
  "[redirect] [value] [arg]\n"
  "  redirect - Send output to 0:none, 1:stdout, 2:syslog, 3:file\n"
  "             For file, include file name as [arg]\n"
  "             default: 1\n";
const char shell_help_summary_shell[] = "Set shell parameters";

void shell_shell(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    if (argc >= 2) {
        if (strcasecmp(argv[1], "redirect") == 0) {
            if (argc >= 3) {
                if (ctx->redirectFile) {
                    fclose(ctx->redirectFile);
                    ctx->redirectFile = NULL;
                }
                ctx->redirect = atoi(argv[2]);
                if ((ctx->redirect < 0) || (ctx->redirect > 3)) {
                    ctx->redirect = 1;
                }
                if (ctx->redirect == 3) {
                    if (argc > 3) {
                        ctx->redirectFile = fopen(argv[3], "w");
                    }
                    if (ctx->redirectFile == NULL) {
                        ctx->redirect = 1;
                        printf("Invalid file\n");
                    }
                }
            } else {
                printf("Shell redirect: %d\n", ctx->redirect);
            }
        } else {
            printf("Invalid shell subcommand\n");
        }
    }
}

/***********************************************************************
 * CMD: test
 **********************************************************************/
const char shell_help_test[] = "\n";
const char shell_help_summary_test[] = "Test command";

void shell_test(SHELL_CONTEXT *ctx, int argc, char **argv )
{
}

/***********************************************************************
 * CMD: vu
 **********************************************************************/
const char shell_help_vu[] =
    "[options] [domain [ a2b | system ] ]\n"
    "  Valid options\n"
    "    -c <channels> - Values: 1 - 64\n"
    "    -d <domain>   - Values \"a2b\", \"system\"\n"
    "    -s <outchar>  - Values: ASCII character (e.g. \"T\") or \n"
    "                             UTF-8 hex string (e.g \"25A0\")\n"
    "  domain          - Only set/get the VU meter clock comain\n"
    "                    Values: \"a2b\" or \"system\"";
const char shell_help_summary_vu[] = "Show VU meters";

#include <math.h>
#include "vu_audio.h"

#define VU_UPDATE_DELAY_MS    75

static int shell_vu_set_domain(const char *str)
{
    int ret = 0;
    if (strcmp(str, "a2b") == 0) {
        clock_domain_set(context, CLOCK_DOMAIN_A2B, CLOCK_DOMAIN_BITM_VU_IN);
    } else if (strcmp(str, "system") == 0) {
        clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_VU_IN);
    } else {
        ret = -1;
    }
    return(ret);
}

void shell_vu(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    unsigned idx;
    unsigned channels = SYSTEM_MAX_CHANNELS;
    SYSTEM_AUDIO_TYPE vu[VU_MAX_CHANNELS];
    SYSTEM_AUDIO_TYPE value;
    unsigned vu_pix[VU_MAX_CHANNELS];
    double db, dbLine;
    unsigned lines, cols;
    unsigned vu_lines, offset;
    unsigned l;
    int c;
    char *screen[2] = { NULL, NULL };
    char p;
    unsigned screenIdx = 0;
    unsigned screenSize;
    unsigned screenPix;
    bool split_screen;
    unsigned ch;
    unsigned line, col;
    struct option_data opt_data = OPT_DATA_INIT;
    int opt;
    int opt_offset;
    int err = 0;
    char ylabel[16];
    unsigned dbNext;

    char *HIDE_CURSOR = "\x1B[?25l";
    char *SHOW_CURSOR = "\x1B[?25h";
    char *RESET_MODE = "\x1B[0m";
    char *BLACK = "\x1B[30;40m";

    /* Black background with block char, 1 space between channels */
    char *RED = "\x1B[31;40m";
    char *YELLOW = "\x1B[33;40m";
    char *GREEN = "\x1B[32;40m";
    char *WHITE = "\x1B[37;40m";

    long int unicodechar;
    char outchar[5] = "|";

    /* Parse options. */
    while ((opt = _getopt(argc, argv, ":c:d:s:", &opt_data)) != -1) {
        switch(opt) {
            case 'c':
                channels = strtol(opt_data.optarg, NULL, 0);
                if ((channels < 1) || (channels > VU_MAX_CHANNELS)) {
                    err = 1;
                    printf("Bad channels\n");
                }
                break;
            case 'd':
                err = shell_vu_set_domain(opt_data.optarg);
                if (err != 0) {
                    err = 2;
                    printf("Bad domain\n");
                }
                break;
            case 's':
                if(strlen(opt_data.optarg) == 1) {
                    /* Use character as it is */
                    if( (opt_data.optarg[0] >= 0x20) && (opt_data.optarg[0] <= 0x7E) ){
                        (void) strcpy(outchar, opt_data.optarg);
                    }
                } else {
                    /* Encode to UTF-8 */
                    unicodechar = strtol(opt_data.optarg, NULL, 16);
                    if(false == encodeUtf8(ctx, unicodechar, outchar)) {
                        err = 3;
                    }
                }
                break;
            case '?':
                err = -1;
                printf("Option '%c' is not a valid option\n", opt_data.optopt);
                break;
            case ':':
                err = -2;
                printf("Option '%c' requires an argument\n", opt_data.optopt);
                break;
            default:
                err = -3;
                printf("Error parsing options\n");
                break;
        }
    }

    if(err != 0){
        return;
    }

    opt_offset = opt_data.optind - 1;

    /* Short circuit only getting/setting clock domain */
    if ((argc >= (opt_offset + 2)) && (strcmp(argv[opt_offset + 1], "domain") == 0)) {
        CLOCK_DOMAIN cd;
        if (argc == (opt_offset + 2)) {
            cd = clock_domain_get(context, CLOCK_DOMAIN_BITM_VU_IN);
            printf("%s\n", clock_domain_str(cd));
            return;
        } else {
            err = shell_vu_set_domain(argv[opt_offset + 2]);
            if (err != 0) {
                printf("Bad domain\n");
            }
        }
        return;
    }

    channels = getVU(context, NULL, channels);

    char *color = NULL;
    char *old_color = NULL;

    /* Get screen info */
    lines = term_get_lines(&ctx->t);
    cols = term_get_cols(&ctx->t);

    /* Make screen buffers */
    screenSize = lines * cols;
    screen[0] = SHELL_MALLOC(screenSize);
    memset(screen[0], 'B', screenSize);
    screen[1] = SHELL_MALLOC(screenSize);
    memset(screen[1], 'B', screenSize);

    /* Calculate some values */
    if (channels > 32) {
        split_screen = true;
        vu_lines = lines / 2;
    } else {
        split_screen = false;
        vu_lines = lines;
    }

    /* Clear the screen */
    term_clrscr(&ctx->t);

    /* Hide cursor */
    term_putstr(&ctx->t, HIDE_CURSOR, strlen(HIDE_CURSOR));

    /* Minimal y axis */
    term_putstr(&ctx->t, WHITE, strlen(WHITE));
    offset = 1;
    for (int i = 0; i < (split_screen ? 2 : 1); i++) {
        dbLine = 72.0 / (double)(vu_lines-1);
        db = 0.0; dbNext = split_screen ? 24 : 12;
        term_gotoxy(&ctx->t, 1, offset);
        term_putstr(&ctx->t, "0dB", 3);
        for (idx = 0; idx <= vu_lines-1 && dbNext < 72; idx++) {
            if (db >= dbNext) {
                term_gotoxy(&ctx->t, 1, idx + offset);
                sprintf(ylabel, "-%ddB", dbNext);
                term_putstr(&ctx->t, ylabel, strlen(ylabel));
                dbNext += split_screen ? 24 : 12;
            }
            db += dbLine;
        }
        term_gotoxy(&ctx->t, 1, vu_lines + offset - 1);
        term_putstr(&ctx->t, "-72dB", 5);
        offset += vu_lines;
    }

    do {
        /* Get VU linear values */
        getVU(context, vu, channels);

        /* Convert to db then to screen height */
        for (idx = 0; idx < channels; idx++) {
            value = vu[idx];
            if (value > 0) {
                db = 20.0 * log10((double)value / 2147483647.0);
                /* Scale for VU meter -72dB = 0, 0dB = 'vu_lines' */
                db = ((double)vu_lines / 72.0) * db + (double)vu_lines;
                /* Round up to show full scale */
                db += 0.5;
                /* Always show something if a signal is present */
                if ((db < 1.0) && value) {
                    db = 1.0;
                }
            } else {
                db = 0.0;
            }
            vu_pix[idx] = (unsigned)db;
            if (vu_pix[idx] > vu_lines) {
                vu_pix[idx] = vu_lines;
            }
            if (vu_pix[idx] < 1) {
                vu_pix[idx] = 1;
            }
        }

        /* Render into screen buffer */
        for (int i = 0; i < (split_screen ? 2 : 1); i++) {
            if (i == 0) { ch = channels > 32 ? 32 : channels; }
            if (i == 1) { ch = channels - 32; }
            offset = (cols - 32 * 2) / 2;
            for (idx = 0; idx < ch; idx++) {
                for (l = 0; l < vu_lines; l++) {
                    if (l < vu_pix[idx+32*i]) {
                        if (l > ((vu_lines * 2) / 3)) {
                            p = 'R';
                        } else if (l > (vu_lines / 3)) {
                            p = 'Y';
                        } else {
                            p = 'G';
                        }
                    } else {
                        p = 'B';
                    }
                    line = ((lines / (i+1)) - 1) - l;
                    col = offset + idx * 2;
                    screenPix = line * cols + col;
                    screen[screenIdx][screenPix] = p;
                }
            }
        }

        /* Draw only differences */
        for (line = 0; line < lines; line++) {
            for (col = 0; col < cols; col++) {
                screenPix = line * cols + col;
                if (screen[0][screenPix] != screen[1][screenPix]) {
                    p = screen[screenIdx][screenPix];
                    switch (p) {
                        case 'R':
                            color = RED;
                            break;
                        case 'Y':
                            color = YELLOW;
                            break;
                        case 'G':
                            color = GREEN;
                            break;
                        case 'B':
                            color = BLACK;
                            break;
                        default:
                            color = BLACK;
                            break;
                    }
                    term_gotoxy(&ctx->t, col + 1, line + 1);
                    if (color != old_color) {
                        term_putstr(&ctx->t, color, strlen(color));
                        color = old_color;
                    }
                    term_putstr(&ctx->t, outchar, strlen(outchar));
                }
            }
        }

        /* Toggle screens */
        screenIdx = (screenIdx + 1) & 1;

        /* Park the cursor */
        term_gotoxy(&ctx->t, cols, lines);

        /* Check for character */
        c = term_getch(&ctx->t, TERM_INPUT_DONT_WAIT);
        if (c < 0) {
            vTaskDelay(pdMS_TO_TICKS(VU_UPDATE_DELAY_MS));
        }

    } while (c < 0);

    /* Reset the character mode */
    term_putstr(&ctx->t, RESET_MODE, strlen(RESET_MODE));
    term_putstr(&ctx->t, SHOW_CURSOR, strlen(SHOW_CURSOR));

    /* Clear the screen */
    term_clrscr(&ctx->t);

    /* Go back home */
    term_gotoxy(&ctx->t, 0, 0);

    /* Free screen buffers */
    if (screen[0]) {
        SHELL_FREE(screen[0]);
    }
    if (screen[1]) {
        SHELL_FREE(screen[1]);
    }
}


/***********************************************************************
 * CMD: reset
 **********************************************************************/
#include "init.h"

const char shell_help_reset[] = "\n";
const char shell_help_summary_reset[] = "Resets the system";

void shell_reset(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    printf("Resetting...\n");
    delay(100);
    system_reset(context);
}

/***********************************************************************
 * CMD: route
 **********************************************************************/
const char shell_help_route[] =
    "[ <idx> <src> <src offset> <dst> <dst offset> <channels> [attenuation] [mix|set] ]\n"
    "  idx         - Routing index\n"
    "  src         - Source stream\n"
    "  src offset  - Source stream offset\n"
    "  dst         - Destination stream\n"
    "  dst offset  - Destination stream offset\n"
    "  channels    - Number of channels\n"
    "  attenuation - Source attenuation in dB (0dB default)\n"
    "  mix         - Mix or set source into destination (set default)\n"
    " Valid Streams\n"
    "  usb        - USB Audio\n"
    "  a2b        - A2B Audio\n"
    "  codec      - Analog TRS line in/out\n"
    "  spdif      - Optical SPDIF in/out\n"
#ifdef SHARC_AUDIO_ENABLE
    "  sharc0     - SHARC0 Audio\n"
    "  sharc1     - SHARC1 Audio\n"
#endif
    "  wav        - WAV file src/sink\n"
    "  rtp        - RTP network audio tx\n"
    "  vban       - VBAN network audio tx\n"
    "  vu         - VU Meter sink\n"
    "  off        - Turn off the stream\n"
    " No arguments\n"
    "  Show routing table\n"
    " Single 'clear' argument\n"
    "  Clear routing table\n";
const char shell_help_summary_route[] = "Configures the audio routing table";

#include "route.h"

static char *stream2str(int streamID)
{
    char *str = "NONE";

    switch (streamID) {
        case STREAM_ID_UNKNOWN:
            str = "NONE";
            break;
        case STREAM_ID_CODEC_IN:
            str = "CODEC_IN";
            break;
        case STREAM_ID_CODEC_OUT:
            str = "CODEC_OUT";
            break;
        case STREAM_ID_SPDIF_IN:
            str = "SPDIF_IN";
            break;
        case STREAM_ID_SPDIF_OUT:
            str = "SPDIF_OUT";
            break;
        case STREAM_ID_A2B_IN:
            str = "A2B_IN";
            break;
        case STREAM_ID_A2B_OUT:
            str = "A2B_OUT";
            break;
        case STREAM_ID_USB_RX:
            str = "USB_RX";
            break;
        case STREAM_ID_USB_TX:
            str = "USB_TX";
            break;
        case STREAM_ID_WAV_SRC:
            str = "WAV_SRC";
            break;
        case STREAM_ID_WAV_SINK:
            str = "WAV_SINK";
            break;
        case STREAM_ID_SHARC0_IN:
            str = "SHARC0_IN";
            break;
        case STREAM_ID_SHARC0_OUT:
            str = "SHARC0_OUT";
            break;
        case STREAM_ID_SHARC1_IN:
            str = "SHARC1_IN";
            break;
        case STREAM_ID_SHARC1_OUT:
            str = "SHARC1_OUT";
            break;
        case STREAM_ID_VU_IN:
            str = "VU_IN";
            break;
        case STREAM_ID_RTP_RX:
            str = "RTP_RX";
            break;
        case STREAM_ID_RTP_TX:
            str = "RTP_TX";
            break;
        case STREAM_ID_VBAN_RX:
            str = "VBAN_RX";
            break;
        case STREAM_ID_VBAN_TX:
            str = "VBAN_TX";
            break;
        default:
            str = "UNKNOWN";
            break;
    }

    return(str);
}

STREAM_ID str2stream(char *stream, bool src)
{
    if (strcmp(stream, "usb") == 0) {
        return(src ? STREAM_ID_USB_RX : STREAM_ID_USB_TX);
    } else if (strcmp(stream, "codec") == 0) {
        return(src ? STREAM_ID_CODEC_IN : STREAM_ID_CODEC_OUT);
    } else if (strcmp(stream, "spdif") == 0) {
        return(src ? STREAM_ID_SPDIF_IN : STREAM_ID_SPDIF_OUT);
    } else if (strcmp(stream, "a2b") == 0) {
        return(src ? STREAM_ID_A2B_IN : STREAM_ID_A2B_OUT);
    } else if (strcmp(stream, "wav") == 0) {
        return(src ? STREAM_ID_WAV_SRC : STREAM_ID_WAV_SINK);
    } else if (strcmp(stream, "sharc0") == 0) {
        return(src ? STREAM_ID_SHARC0_OUT : STREAM_ID_SHARC0_IN);
    } else if (strcmp(stream, "sharc1") == 0) {
        return(src ? STREAM_ID_SHARC1_OUT : STREAM_ID_SHARC1_IN);
    } else if (strcmp(stream, "vu") == 0) {
        return(src ? STREAM_ID_MAX : STREAM_ID_VU_IN);
    } else if (strcmp(stream, "rtp") == 0) {
        return(src ? STREAM_ID_RTP_RX : STREAM_ID_RTP_TX);
    } else if (strcmp(stream, "vban") == 0) {
        return(src ? STREAM_ID_VBAN_RX : STREAM_ID_VBAN_TX);
    } else if (strcmp(stream, "a2b2") == 0) {
        return(src ? STREAM_ID_A2B2_IN : STREAM_ID_A2B2_OUT);
    } else if (strcmp(stream, "off") == 0) {
        return(STREAM_ID_UNKNOWN);
    }

    return(STREAM_ID_MAX);
}

void shell_route(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    ROUTE_INFO *route;
    unsigned idx, srcOffset, sinkOffset, channels, attenuation, mix;
    STREAM_ID srcID, sinkID;
    unsigned i;

    if (argc == 1) {
        printf("Audio Routing\n");
        for (i = 0; i < MAX_AUDIO_ROUTES; i++) {
            route = context->routingTable + i;
            printf(" [%02d]: %s[%u] -> %s[%u], CHANNELS: %u, %s%udB, %s\n",
                i,
                stream2str(route->srcID), route->srcOffset,
                stream2str(route->sinkID), route->sinkOffset,
                route->channels,
                route->attenuation == 0 ? "" : "-",
                (unsigned)route->attenuation, route->mix ? "mix" : "set"
            );
        }
        return;
    } else if (argc == 2) {
        if (strcmp(argv[1], "clear") == 0) {
            for (i = 0; i < MAX_AUDIO_ROUTES; i++) {
                route = context->routingTable + i;
                /* Configure the route atomically in a critical section */
                taskENTER_CRITICAL();
                route->srcID = STREAM_ID_UNKNOWN;
                route->srcOffset = 0;
                route->sinkID = STREAM_ID_UNKNOWN;
                route->sinkOffset = 0;
                route->channels = 0;
                route->attenuation = 0;
                route->mix = 0;
                taskEXIT_CRITICAL();
            }
        } else {
            printf("Invalid input. Type 'help route' for more details.\n");
            return;
        }
    }

    /* Confirm a valid route index */
    idx = atoi(argv[1]);
    if (idx > MAX_AUDIO_ROUTES) {
        printf("Invalid idx\n");
        return;
    }
    route = context->routingTable + idx;
    srcID = route->srcID;
    srcOffset = route->srcOffset;
    sinkID = route->sinkID;
    sinkOffset = route->sinkOffset;
    channels = route->channels;
    mix = route->mix;

    /* Gather the source info */
    if (argc >= 3) {
        srcID = str2stream(argv[2], true);
        if (srcID == STREAM_ID_MAX) {
            printf("Invalid src\n");
            return;
        }
    }
    if (argc >= 4) {
        srcOffset = atoi(argv[3]);
    }

    /* Gather the sink info */
    if (argc >= 5) {
        sinkID = str2stream(argv[4], false);
        if (sinkID == STREAM_ID_MAX) {
            printf("Invalid sink\n");
            return;
        }
    }
    if (argc >= 6) {
        sinkOffset = atoi(argv[5]);
    }

    /* Get the number of channels */
    if (argc >= 7) {
        channels = atoi(argv[6]);
        if (channels > SYSTEM_MAX_CHANNELS) {
            printf("Invalid channels\n");
            return;
        }
    }

    /* Get the attenuation */
    if (argc >= 8) {
        attenuation = abs(atoi(argv[7]));
        if (attenuation > 120) {
            attenuation = 120;
        }
    } else {
        attenuation = 0;
    }

    /* Get the 'mix' arg */
    if (argc >= 9) {
        if (strcmp(argv[8], "mix") == 0) {
            mix = 1;
        } else {
            mix = 0;
        }
    } else {
        mix = 0;
    }

    /* Configure the route atomically in a critical section */
    taskENTER_CRITICAL();
    route->srcID = srcID;
    route->srcOffset = srcOffset;
    route->sinkID = sinkID;
    route->sinkOffset = sinkOffset;
    route->channels = channels;
    route->attenuation = attenuation;
    route->mix = mix;
    taskEXIT_CRITICAL();
}

/***********************************************************************
 * CMD: echo
 **********************************************************************/
const char shell_help_echo[] = "[arg_1] ... [arg_n]\n";
const char shell_help_summary_echo[] = "Echos arguments";

void shell_echo(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    int i;
    for (i = 1; i < argc; i++) {
        printf("%s ", argv[i]);
    }
    printf("\n");
}

/***********************************************************************
 * CMD: wav
 **********************************************************************/
const char shell_help_wav[] =
    "[options] <src|sink> <on|off> [file] [channels] [bits]\n"
    "  Valid options\n"
    "    -b <bits>       - Set bit format\n"
    "                      Values: 16, 32\n"
    "    -c <channels>   - Set number of channels\n"
    "                      Values: 1 - 64\n"
    "    -d <domain>     - Set clock domain\n"
    "                      Values: \"a2b\", \"system\"\n"
    "    -f <file>       - Set src/sink file name\n"
    "                      Values: String\n"
    "    -i              - Enable sink file name count auto-increase\n"
    "    -l <loop count> - Plays source file \"loop count\" times\n"
    "                      Values: Greater than 0\n";
const char shell_help_summary_wav[] = "Manages wave file source/sink";

#include "wav_file.h"
#include "clock_domain.h"
#include "task_cfg.h"

static void wav_state(SHELL_CONTEXT *ctx, int clockDomainMask, WAV_FILE *wf, bool isSrc)
{
    char loopHelp[30];

    if(isSrc) {
        if(wf->loopCount > 0) {
            sprintf(loopHelp, ", %d/%d loops", wf->loopCount, wf->loopCountTotal);
        } else {
            sprintf(loopHelp, ", N/A");
        }
    }

    printf(
        "%s: %s, %s, %d-bit, %d ch, %s%s\n",
        isSrc ? "Src" : "Sink",
        wf->enabled ? "ON" : "OFF",
        wf->fname ? wf->fname : "N/A",
        wf->wordSizeBytes == 2 ? 16 : 32,
        wf->channels,
        clock_domain_str(clock_domain_get(context, clockDomainMask)),
        isSrc ? loopHelp : ""
    );
}

void shell_wav( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    WAV_FILE *wf = NULL;
    int channels = 2;
    int wordSizeBytes = sizeof(int16_t);
    int bits = 16;
    char *fname = NULL;
    bool on;
    bool isSrc;
    bool inc = false;
    bool ok = true;
    CLOCK_DOMAIN domain = CLOCK_DOMAIN_MAX;
    int clockDomainMask = 0;
    bool channelsSpecified = false;
    int loopCount = -1; /* Loop indefinitely */

    struct option_data opt_data = OPT_DATA_INIT;
    int opt;
    int opt_offset;

    /* Parse options. */
    while ((opt = _getopt(argc, argv, ":b:c:d:f:il:", &opt_data)) != -1) {
        switch(opt) {
            case 'b':
                bits = atoi(opt_data.optarg);
                break;
            case 'c':
                channels = atoi(opt_data.optarg);
                channelsSpecified = true;
                break;
            case 'd':
                if (strcmp(opt_data.optarg, "a2b") == 0) {
                    domain = CLOCK_DOMAIN_A2B;
                } else if (strcmp(opt_data.optarg, "system") == 0) {
                    domain = CLOCK_DOMAIN_SYSTEM;
                } else {
                    ok = false;
                    printf("Bad domain\n");
                }
                break;
            case 'f':
                fname = opt_data.optarg;
                break;
            case 'i':
                inc = true;
                break;
            case 'l':
                loopCount = strtol(opt_data.optarg, NULL, 0);
                if (loopCount <= 0) {
                    ok = false;
                    printf("Value must be a positive integer\n");
                }
                break;
            case '?':
                ok = false;
                printf("Option '%c' is not a valid option\n", opt_data.optopt);
                break;
            case ':':
                ok = false;
                printf("Option '%c' requires an argument\n", opt_data.optopt);
                break;
            default:
                ok = false;
                printf("Error parsing options\n");
                break;
        }
    }
    if (!ok) {
        return;
    }

    opt_offset = opt_data.optind - 1;

    if (argc == opt_data.optind) {
        wav_state(ctx, CLOCK_DOMAIN_BITM_WAV_SRC, &context->wavSrc, true);
        wav_state(ctx, CLOCK_DOMAIN_BITM_WAV_SINK, &context->wavSink, false);
        return;
    }

    if (argc >= (opt_offset + 2)) {
        if (strcmp(argv[opt_offset + 1], "src") == 0) {
            wf = &context->wavSrc;
            isSrc = true;
            clockDomainMask = CLOCK_DOMAIN_BITM_WAV_SRC;
        } else if (strcmp(argv[opt_offset + 1], "sink") == 0) {
            wf = &context->wavSink;
            isSrc = false;
            clockDomainMask = CLOCK_DOMAIN_BITM_WAV_SINK;
        } else {
            ok = false;
        }
    } else {
        ok = false;
    }
    if (!ok) {
        printf("Invalid src/sink\n");
        return;
    }

    if (argc >= (opt_offset + 3)) {
        if (strcmp(argv[opt_offset + 2], "on") == 0) {
            if (wf->enabled) {
                printf("Already on\n");
                return;
            }
            on = true;
        } else if (strcmp(argv[opt_offset + 2], "off") == 0) {
            if (!wf->enabled) {
                printf("Already off\n");
                return;
            }
            on = false;
        } else {
            ok = false;
        }
    } else {
        ok = false;
    }
    if (!ok) {
        printf("Invalid on/off\n");
        return;
    }

    if (argc >= (opt_offset + 4)) {
        fname = argv[opt_offset + 3];
    }
    if ((wf->fname == NULL) && (fname == NULL)) {
        if(isSrc) {
            fname = "src.wav";
        } else {
            fname = "sink.wav";
        }
    }

    if (argc >= (opt_offset + 5)) {
        channels = atoi(argv[opt_offset + 4]);
        channelsSpecified = true;
    }
    if (channels > WAV_MAX_CHANNELS) {
        channels = WAV_MAX_CHANNELS;
    }

    if (argc >= (opt_offset + 6)) {
        bits = atoi(argv[opt_offset + 5]);
    }
    if (bits == 32) {
        wordSizeBytes = 4;
    }

    if((domain != CLOCK_DOMAIN_MAX) && (clockDomainMask != 0)) {
        clock_domain_set(context, domain, clockDomainMask);
    }

    UBaseType_t oldPrio = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, WAV_TASK_PRIORITY + 1);
    xSemaphoreTake((SemaphoreHandle_t)wf->lock, portMAX_DELAY);
    if (on) {
        if (isSrc) {
            wf->loopCount = loopCount;
            wf->loopCountTotal = loopCount;
            memset(&context->wavSrcStats, 0, sizeof(context->wavSrcStats));
            copyFileName(&wf->fname, fname);
        } else {
            wf->channels = channels;
            wf->sampleRate = SYSTEM_SAMPLE_RATE;
            wf->wordSizeBytes = wordSizeBytes;
            wf->frameSizeBytes = wf->channels * wf->wordSizeBytes;
            memset(&context->wavSinkStats, 0, sizeof(context->wavSinkStats));
            if(fname) {
                wf->fileAutoIncrease = inc;
                if(inc) {
                    copyFileName(&wf->fnameInput, fname);
                    wf->fcount = 0;
                } else {
                    copyFileName(&wf->fname, fname);
                }
            }
            if(wf->fileAutoIncrease) {
                insertCountToFileName(&wf->fname, wf->fnameInput, wf->fcount);
            }
        }
        wf->isSrc = isSrc;
        ok = openWave(wf);
        if (!ok) {
            printf("Failed to open %s\n", wf->fname);
        } else {
            if (isSrc) {
                if (wf->waveInfo.numChannels > WAV_MAX_CHANNELS) {
                    printf("Must be less than %d channels\n", WAV_MAX_CHANNELS);
                    closeWave(wf);
                }
                if ((wf->waveInfo.waveFmt != WAVE_FMT_SIGNED_32BIT_LE) &&
                    (wf->waveInfo.waveFmt != WAVE_FMT_SIGNED_16BIT_LE)) {
                    printf("Must be S16_LE or S32_LE format\n");
                    closeWave(wf);
                }
                if (wf->waveInfo.sampleRate != SYSTEM_SAMPLE_RATE) {
                    syslog_printf("WAV file sample rate mismatch: %d\n",
                        wf->waveInfo.sampleRate);
                    if (channelsSpecified) {
                        overrideWave(wf, channels);
                        syslog_printf("WAV file channel override: %d\n",
                            wf->channels);
                    }
                }
            }
        }
    } else {
        if (isSrc) {
            if (context->wavSrcStats.slowReads) {
                printf("WARNING: %u slow read%s\n", context->wavSrcStats.slowReads,
                    context->wavSrcStats.slowReads == 1 ? "" : "s");
            }
            if (context->wavSrcStats.underrun) {
                printf("WARNING: %u audio underrund\n", context->wavSrcStats.underrun);
            }
            memset(&context->wavSrcStats, 0, sizeof(context->wavSrcStats));
        } else {
            if (context->wavSinkStats.slowWrites) {
                printf("WARNING: %u slow write%s\n", context->wavSinkStats.slowWrites,
                    context->wavSinkStats.slowWrites == 1 ? "" : "s");
            }
            if (context->wavSinkStats.overrun) {
                printf("WARNING: %u audio overruns\n", context->wavSinkStats.overrun);
            }
            memset(&context->wavSrcStats, 0, sizeof(context->wavSrcStats));
            wf->fcount++;
        }
        closeWave(wf);
    }
    xSemaphoreGive((SemaphoreHandle_t)wf->lock);
    vTaskPrioritySet(NULL, oldPrio);
}

/***********************************************************************
 * CMD: rtp
 **********************************************************************/
const char shell_help_rtp[] =
    "<rx|tx> <on|off> <ip> [port] [channels] [bits]\n"
    "  ip - Source IP address for rx or dest IP address for tx\n"
    "  port - IP port number (Default 6970)\n"
    "  channels - Routable channels (Default 2)\n"
    "  bits - Audio bit depth.  16 and 32 supported (Default 16)\n";
const char shell_help_summary_rtp[] = "Manages RTP stream Rx/Tx";

#include "rtp_stream.h"
#include "clock_domain.h"

static void rtp_state(SHELL_CONTEXT *ctx, char *name, int clockDomainMask, RTP_STREAM *rs)
{
    printf(
        "%s: %s, %s:%d, %d-bit, %d ch, %s\n",
        name,
        rs->enabled ? "ON" : "OFF",
        rs->ipStr ? rs->ipStr : "N/A",
        rs->port,
        rs->wordSizeBytes == 2 ? 16 : 32,
        rs->channels,
        clock_domain_str(clock_domain_get(context, clockDomainMask))
    );
}

void shell_rtp( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    RTP_STREAM *rs = NULL;
    int channels;
    int wordSizeBytes;
    int bits;
    bool on;
    bool isRx;
    bool ok = true;
    int clockDomainMask;
    int port;
    char *ipStr = NULL;

    if (argc == 1) {
        rtp_state(ctx, "Rx", CLOCK_DOMAIN_BITM_RTP_RX, &context->rtpRx);
        rtp_state(ctx, "Tx", CLOCK_DOMAIN_BITM_RTP_TX, &context->rtpTx);
        return;
    }

    if (argc >= 2) {
        if (strcmp(argv[1], "rx") == 0) {
            rs = &context->rtpRx;
            isRx = true;
            clockDomainMask = CLOCK_DOMAIN_BITM_RTP_RX;
        } else if (strcmp(argv[1], "tx") == 0) {
            rs = &context->rtpTx;
            isRx = false;
            clockDomainMask = CLOCK_DOMAIN_BITM_RTP_TX;
        } else {
            ok = false;
        }
    } else {
        ok = false;
    }
    if (!ok) {
        printf("Invalid rx/tx\n");
        return;
    }

    if (argc >= 3) {
        if (strcmp(argv[2], "on") == 0) {
            if (rs->enabled) {
                printf("Already on\n");
                return;
            }
            on = true;
        } else if (strcmp(argv[2], "off") == 0) {
            if (!rs->enabled) {
                printf("Already off\n");
                return;
            }
            on = false;
        } else if (strcmp(argv[2], "domain") == 0) {
            if (argc >= 4) {
                if (strcmp(argv[3], "a2b") == 0) {
                    clock_domain_set(context, CLOCK_DOMAIN_A2B, clockDomainMask);
                } else if (strcmp(argv[3], "system") == 0) {
                    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, clockDomainMask);
                } else if (strcmp(argv[3], "rtp") == 0) {
                    clock_domain_set(context, CLOCK_DOMAIN_RTP, clockDomainMask);
                } else {
                    printf("Bad domain\n");
                }
                return;
            }
            return;
        } else {
            ok = false;
        }
    } else {
        ok = false;
    }
    if (!ok) {
        printf("Invalid on/off/domain\n");
        return;
    }

    if (argc >= 4) {
        ipStr = argv[3];
    } else {
        if (rs->ipStr) {
            ipStr = NULL;
        } else {
            ok = false;
        }
    }
    if (!ok) {
        printf("Invalid ip address\n");
        return;
    }

    if (argc >= 5) {
        port = atoi(argv[4]);
    } else {
        port = rs->port;
    }

    if (argc >= 6) {
        channels = atoi(argv[5]);
        if (channels > SYSTEM_MAX_CHANNELS) {
            channels = SYSTEM_MAX_CHANNELS;
        }
    } else {
        channels = rs->channels;
    }

    if (argc >= 7) {
        bits = atoi(argv[6]);
        if (bits > 16) {
            wordSizeBytes = 4;
        } else {
            wordSizeBytes = 2;
        }
    } else {
        wordSizeBytes = rs->wordSizeBytes;
    }

    xSemaphoreTake((SemaphoreHandle_t)rs->lock, portMAX_DELAY);
    if (on) {
        if (ipStr) {
            if (rs->ipStr) {
                SHELL_FREE(rs->ipStr); rs->ipStr = NULL;
            }
            rs->ipStr = SHELL_MALLOC(strlen(ipStr) + 1);
            strcpy(rs->ipStr, ipStr);
        }
        rs->channels = channels;
        rs->wordSizeBytes = wordSizeBytes;
        rs->port = port;
        rs->isRx = isRx;
        ok = openRtpStream(rs);
        if (!ok) {
            printf("Failed to open port %d\n", rs->port);
        }
    } else {
        closeRtpStream(rs);
    }
    xSemaphoreGive((SemaphoreHandle_t)rs->lock);
}

/***********************************************************************
 * CMD: vban
 **********************************************************************/
const char shell_help_vban[] =
    "<rx|tx> <on|off> <ip> [port] [channels] [bits]\n"
    "  ip - Source IP address for rx or dest IP address for tx\n"
    "  port - IP port number (Default 6980)\n"
    "  channels - Routable channels (Default 2)\n"
    "  bits - Audio bit depth.  16 and 32 supported (Default 16)\n";
const char shell_help_summary_vban[] = "Manages VBAN stream Rx/Tx";

#include "vban_stream.h"
#include "clock_domain.h"

static void vban_state(SHELL_CONTEXT *ctx, char *name, int clockDomainMask, VBAN_STREAM *rs)
{
    printf(
        "%s: %s, %s:%d, %d-bit, %d ch, %s\n",
        name,
        rs->enabled ? "ON" : "OFF",
        rs->ipStr ? rs->ipStr : "N/A",
        rs->port,
        rs->wordSizeBytes == 2 ? 16 : 32,
        rs->channels,
        clock_domain_str(clock_domain_get(context, clockDomainMask))
    );
}

void shell_vban( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    VBAN_STREAM *rs = NULL;
    int channels;
    int wordSizeBytes;
    int bits;
    bool on;
    bool isRx;
    bool ok = true;
    int clockDomainMask;
    int port;
    char *ipStr = NULL;

    if (argc == 1) {
        vban_state(ctx, "Rx", CLOCK_DOMAIN_BITM_VBAN_RX, &context->vbanRx);
        vban_state(ctx, "Tx", CLOCK_DOMAIN_BITM_VBAN_TX, &context->vbanTx);
        return;
    }

    if (argc >= 2) {
        if (strcmp(argv[1], "rx") == 0) {
            rs = &context->vbanRx;
            isRx = true;
            clockDomainMask = CLOCK_DOMAIN_BITM_VBAN_RX;
        } else if (strcmp(argv[1], "tx") == 0) {
            rs = &context->vbanTx;
            isRx = false;
            clockDomainMask = CLOCK_DOMAIN_BITM_VBAN_TX;
        } else {
            ok = false;
        }
    } else {
        ok = false;
    }
    if (!ok) {
        printf("Invalid rx/tx\n");
        return;
    }

    if (argc >= 3) {
        if (strcmp(argv[2], "on") == 0) {
            if (rs->enabled) {
                printf("Already on\n");
                return;
            }
            on = true;
        } else if (strcmp(argv[2], "off") == 0) {
            if (!rs->enabled) {
                printf("Already off\n");
                return;
            }
            on = false;
        } else if (strcmp(argv[2], "domain") == 0) {
            if (argc >= 4) {
                if (strcmp(argv[3], "a2b") == 0) {
                    clock_domain_set(context, CLOCK_DOMAIN_A2B, clockDomainMask);
                } else if (strcmp(argv[3], "system") == 0) {
                    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, clockDomainMask);
                } else if (strcmp(argv[3], "vban") == 0) {
                    clock_domain_set(context, CLOCK_DOMAIN_VBAN, clockDomainMask);
                } else {
                    printf("Bad domain\n");
                }
                return;
            }
            return;
        } else {
            ok = false;
        }
    } else {
        ok = false;
    }
    if (!ok) {
        printf("Invalid on/off/domain\n");
        return;
    }

    if (argc >= 4) {
        ipStr = argv[3];
    } else {
        if (rs->ipStr) {
            ipStr = NULL;
        } else {
            ok = false;
        }
    }
    if (!ok) {
        printf("Invalid ip address\n");
        return;
    }

    if (argc >= 5) {
        port = atoi(argv[4]);
    } else {
        port = rs->port;
    }

    if (argc >= 6) {
        channels = atoi(argv[5]);
        if (channels > SYSTEM_MAX_CHANNELS) {
            channels = SYSTEM_MAX_CHANNELS;
        }
    } else {
        channels = rs->channels;
    }

    if (argc >= 7) {
        bits = atoi(argv[6]);
        if (bits > 16) {
            wordSizeBytes = 4;
        } else {
            wordSizeBytes = 2;
        }
    } else {
        wordSizeBytes = rs->wordSizeBytes;
    }

    xSemaphoreTake((SemaphoreHandle_t)rs->lock, portMAX_DELAY);
    if (on) {
        if (ipStr) {
            if (rs->ipStr) {
                SHELL_FREE(rs->ipStr); rs->ipStr = NULL;
            }
            rs->ipStr = SHELL_MALLOC(strlen(ipStr) + 1);
            strcpy(rs->ipStr, ipStr);
        }
        rs->channels = channels;
        rs->wordSizeBytes = wordSizeBytes;
        rs->port = port;
        rs->isRx = isRx;
        rs->systemSampleRate = SYSTEM_SAMPLE_RATE;
        ok = vbanOpenStream(rs);
        if (!ok) {
            printf("Failed to open port %d\n", rs->port);
        }
    } else {
        vbanCloseStream(rs);
    }
    xSemaphoreGive((SemaphoreHandle_t)rs->lock);
}

/***********************************************************************
 * CMD: cmp (file compare)
 **********************************************************************/
const char shell_help_cmp[] = "<file1> <file2>\n";
const char shell_help_summary_cmp[] = "Compare <file1> to <file2>";

#define SHELL_CMP_PROGRESS_DOT  10000

void shell_cmp( SHELL_CONTEXT *ctx, int argc, char **argv )
{
   FILE *fp1 = NULL, *fp2 = NULL;
   unsigned long bytecount = 0;
   unsigned int printcount = 0;
   unsigned char byte1, byte2;
   bool match = false;
   bool eof1 = false;
   bool eof2 = false;

   if( argc != 3 ) {
      printf( "Usage: cmp <file1> <file2>\n" );
      return;
   }

   if( ( fp1 = fopen( argv[ 1 ], "r" ) ) == NULL ) {
      printf( "Unable to open %s\n", argv[ 1 ] );
   } else {
        if( ( fp2 = fopen( argv[ 2 ], "r" ) ) == NULL ) {
             printf( "Unable to open %s\n", argv[ 2 ] );
        } else {
            /* file compare */
            while (1) {
                byte1 = fgetc(fp1);
                if (feof(fp1)) {
                    eof1 = true;
                }
                byte2 = fgetc(fp2);
                if (feof(fp2)) {
                    eof2 = true;
                }
                if (byte1 == byte2) {
                    match = true;
                    bytecount++;
                    printcount++;
                    if (printcount > SHELL_CMP_PROGRESS_DOT) {
                        printf(".");  /* only updates screen when buffer fills */
                        printcount = 0;
                    }
                } else {
                    fseek(fp1, -1, SEEK_CUR);
                    match = false;
                    break;
                }
                if (eof1 || eof2) {
                    break;
                }
            }
            if (match) {
                printf("\nFiles match (%ld bytes)\n", bytecount);
            } else {
                printf("\nFiles don't match, first mismatch at byte: %ld (0x%02X : 0x%02X)\n", ftell(fp1), byte1, byte2);
            }
            if (eof1) {
                printf("EOF: %s\n", argv[1]);
            }
            if (eof2) {
                printf("EOF: %s\n", argv[2]);
            }
        }
    }

    if( fp1 ) {
        fclose( fp1 );
    }
    if( fp2 ) {
        fclose( fp2 );
    }
}

/***********************************************************************
 * CMD: a2b
 **********************************************************************/
const char shell_help_a2b[] = "[cmd]\n"
  "  mode [main|sub]   - Bus mode 'main node' or 'sub node'\n"
  "                      (default 'main')\n"
  "  No arguments, show current bus mode\n";

const char shell_help_summary_a2b[] = "Set A2B Parameters";

void shell_a2b( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    A2B_BUS_MODE mode;
    bool ok;

    if (argc == 1) {
        printf("A2B Mode: %s\n",
            context->a2bmode == A2B_BUS_MODE_MAIN ? "main node" : "sub node");
        return;
    }

    if (argc >= 2) {
        if (strcmp(argv[1], "mode") == 0) {
            mode = A2B_BUS_MODE_MAIN;
            if (argc >= 3) {
                if (strcmp(argv[2], "sub") == 0) {
                    mode = A2B_BUS_MODE_SUB;
                } else {
                    mode = A2B_BUS_MODE_MAIN;
                }
            }
            printf("Setting A2B mode to %s\n",
                mode == A2B_BUS_MODE_MAIN ? "main" : "sub");
            ok = a2b_set_mode(context, mode);
            if (!ok) {
                printf("Error setting A2B mode!\n");
            }
        } else {
            printf("Invalid subcommand\n");
        }
    }
}

/***********************************************************************
 * CMD: edit
 **********************************************************************/
const char shell_help_edit[] = "<filename>\n";
const char shell_help_summary_edit[] = "Edit a text file";

#include "util.h"

int kilo_main(int argc, char **argv, void *usr);

ssize_t shell_edit_write(void *usr, const void *buf, size_t len)
{
    SHELL_CONTEXT *ctx = (SHELL_CONTEXT *)usr;
    term_putstr(&ctx->t, buf, len);
    return(len);
}

ssize_t shell_edit_read(void *usr, const void *buf, size_t len)
{
    SHELL_CONTEXT *ctx = (SHELL_CONTEXT *)usr;
    int ch = ctx->t.term_in(100000, ctx->t.usr);
    if (ch == -1) {
        return(0);
    }
    ((char *)buf)[0] = ch;
    return(1);
}

void shell_edit(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    int ok;
    if (argc != 2) {
        printf("Usage: %s <filename>\n", argv[0]);
        return;
    }
    ok = kilo_main(argc, argv, ctx);
    if (ok < 0) {
        printf("Error editing file\n");
    }
}

/***********************************************************************
 * CMD: delay
 **********************************************************************/
const char shell_help_delay[] = "<ms>\n";
const char shell_help_summary_delay[] = "Delay";

#include "util.h"

void shell_delay(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    unsigned delaymS = 0;

    if (argc > 1) {
        delaymS = strtoul(argv[1], NULL, 0);
    }

    if (delaymS > 0) {
        delay(delaymS);
    }
}

/***********************************************************************
 * CMD: sdtest
 **********************************************************************/
const char shell_help_sdtest[] =
    "[ms] [raw]\n"
    "   ms - Number of milliseconds to write/read (default 5000)\n"
    "  raw - Test normal raw speed, not optimized WAV file speed (default WAV)\n";

const char shell_help_summary_sdtest[] =
    "Test the write/read speed of the SD card";

#define SDTEST_MS      (5000)
#define RAW_RWBUFSIZE  (1024)

size_t shell_sdtest_wav(SHELL_CONTEXT *ctx, char *fname,
    bool read, uint32_t ms, uint32_t *b, uint32_t *e)
{
    uint32_t begin, end;
    char *buf = NULL;
    size_t words, bsize, rwsize, samples;
    bool ok;

    WAV_FILE wr = {
        .fname = fname,
        .isSrc = true
    };
    WAV_FILE ww = {
        .fname = fname,
        .channels = WAV_MAX_CHANNELS,
        .sampleRate = SYSTEM_SAMPLE_RATE,
        .wordSizeBytes = sizeof(SYSTEM_AUDIO_TYPE),
        .frameSizeBytes = WAV_MAX_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE),
        .isSrc = false
    };
    WAV_FILE *w = read ? &wr : &ww;

    words = 0; begin = 0; end = 1;
    ok = openWave(w);
    if (ok) {
        bsize = w->frameSizeBytes * SYSTEM_BLOCK_SIZE;
        samples = bsize / w->wordSizeBytes;
        buf = umm_calloc(1, bsize);
    }
    if (ok && buf) {
        begin = end = rtosTimeMs();
        while ((end - begin) < ms) {
            if (read) {
                ok = readWave(w, buf, samples, &rwsize);
            } else {
                ok = writeWave(w, buf, samples, &rwsize);
            }

            if(ok) {
                words += rwsize;
                end = rtosTimeMs();
            } else {
                printf("Error %s file\n", (read ? "reading" : "writing"));
                break;
            }
        }

    }
    closeWave(w);

    if (buf) { umm_free(buf); }
    if (b) { *b = begin; }
    if (e) { *e = end;  }

    return(words * w->wordSizeBytes);
}

size_t shell_sdtest_raw(SHELL_CONTEXT *ctx, char *fname,
    bool read, uint32_t ms, uint32_t *b, uint32_t *e)
{
    FILE *f = NULL;
    uint32_t begin, end;
    size_t sz, rwsz;

    char *fbuf = NULL;

    fbuf = (char *)umm_calloc(RAW_RWBUFSIZE, 1);
    sz = 0; begin = 0; end = 1;

    f = fopen(fname, read ? "rb" : "wb");
    if (f && fbuf) {
        begin = end = rtosTimeMs();
        while ((end - begin) < ms) {
            if (read) {
                rwsz = fread(fbuf, 1, RAW_RWBUFSIZE, f);
                if (rwsz == 0) {
                    fseek(f, 0, SEEK_SET);
                }
            } else {
                rwsz = fwrite(fbuf, 1, RAW_RWBUFSIZE, f);
            }
            sz += rwsz;
            end = rtosTimeMs();
        }
    }
    if (f) { fclose(f); }

    if (fbuf) { umm_free(fbuf); }
    if (b) { *b = begin; }
    if (e) { *e = end; }

    return(sz);
}

void shell_sdtest(SHELL_CONTEXT *ctx, int argc, char **argv )
{
#if defined(SDCARD_VOL_NAME) && !defined(SDCARD_USE_EMMC)
    char *fname = "sd:jo11IEjP6i.bin";
    uint32_t begin, end;
    unsigned long kBps;
    size_t sz;
    uint32_t ms;
    bool raw = false;

    ms = SDTEST_MS;
    if (argc > 1) {
        ms = atoi(argv[1]);
        if (ms < 1000) {
            ms = 1000;
        }
    }

    if (argc > 2) {
        raw = strcasecmp(argv[2], "raw") == 0;
    }

    printf("Testing SD card %s write/read for %u mS...\n",
        raw ? "RAW" : "WAV", (unsigned)ms);

    /* Write Test */
    if (raw) {
        sz = shell_sdtest_raw(ctx, fname, false, ms, &begin, &end);
    } else {
        sz = shell_sdtest_wav(ctx, fname, false, ms, &begin, &end);
    }
    kBps = (unsigned long)sz / (unsigned long)(end - begin);
    printf("Write: %lu kB/s (%u Bytes, %u ms)\n",
        kBps, (unsigned)sz, (unsigned)(end - begin));

    /* Read Test */
    if (raw) {
        sz = shell_sdtest_raw(ctx, fname, true, ms, &begin, &end);
    } else {
        sz = shell_sdtest_wav(ctx, fname, true, ms, &begin, &end);
    }
    kBps = (unsigned long)sz / (unsigned long)(end - begin);
    printf("Read: %lu kB/s (%u Bytes, %u ms)\n",
        kBps, (unsigned)sz, (unsigned)(end - begin));

    unlink(fname);
#else
    printf("SDCARD not available.\n");
#endif
}

/***********************************************************************
 * CMD: date
 **********************************************************************/
const char shell_help_date[] =
  "<now>\n"
  "  now - RFC3339 date/time string\n";
const char shell_help_summary_date[] = "Get/Set current date and time";

#include "util.h"
#include "timestamp.h"

void shell_date(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    struct timeval tv;
    timestamp_t ts;
    char tmbuf[64] = { 0 };
    int err, len;

    if (argc > 1) {
        strncpy(tmbuf, argv[1], sizeof(tmbuf));
        len = strlen(tmbuf);
        if (len == 19) {
            strcat(tmbuf, "Z"); len++;
        }
        err = (len != 20);
        if (err == 0) {
            err = timestamp_parse(tmbuf, strlen(tmbuf), &ts);
        }
        if (err) {
            printf("Timestamp format error!\n");
            return;
        }
        tv.tv_sec = ts.sec;
        tv.tv_usec = ts.nsec / 1000;
        util_set_time_unix(&tv);
    }

    util_gettimeofday(&tv, NULL);
    ts.sec = tv.tv_sec;
    ts.nsec = tv.tv_usec * 10000;
    ts.offset = 0;
    timestamp_format(tmbuf, sizeof(tmbuf), &ts, 'T');
    printf("%s\n", tmbuf);
}

/***********************************************************************
 * CMD: eth
 **********************************************************************/
#include "lwip/netif.h"

const char shell_help_eth[] = "\n";
const char shell_help_summary_eth[] = "Report ethernet status";

static void eth_print_addr(SHELL_CONTEXT *ctx, char *name, ip_addr_t *net_addr)
{
    printf("%s: %d.%d.%d.%d\n", name,
        ip4_addr1(net_addr), ip4_addr2(net_addr),
        ip4_addr3(net_addr), ip4_addr4(net_addr));
}

void shell_eth(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    struct netif *n;
    int netifs = 1;
    int i;

#ifdef EMAC1_ENABLE
    netifs += 1;
#endif

    for (i = 0; i < netifs; i++) {
        n = &context->eth[i].netif;
        printf("%c%c%u:\n", n->name[0], n->name[1], n->num);
        printf(" Hostname: %s\n", n->hostname);
        eth_print_addr(ctx, " IP Address", &n->ip_addr);
        eth_print_addr(ctx, " Gateway", &n->gw);
        eth_print_addr(ctx, " Netmask", &n->netmask);
        printf(" MAC Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
            n->hwaddr[0], n->hwaddr[1], n->hwaddr[2],
            n->hwaddr[3], n->hwaddr[4], n->hwaddr[5]
        );
        printf(" Link: %s\n",
            n->flags & NETIF_FLAG_LINK_UP ? "Up" : "Down"
        );
    }
}

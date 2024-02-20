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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "syslog.h"
#include "twi_simple.h"
#include "shell.h"
#include "shell_printf.h"
#include "term.h"
#include "xmodem.h"
#include "util.h"

#include "FreeRTOS.h"
#include "task.h"

#include "context.h"

#ifdef printf
#undef printf
#endif

#define printf(...) shell_printf(ctx, __VA_ARGS__)

/***********************************************************************
 * Main application context
 **********************************************************************/
static APP_CONTEXT *context = &mainAppContext;

/***********************************************************************
 * Misc helper functions
 **********************************************************************/
enum {
    INVALID_FS = -1,
    SPIFFS_FS = 0
};

static int fsVolOK(SHELL_CONTEXT *ctx, APP_CONTEXT *context, char *fs)
{
    if (strcmp(fs, SPIFFS_VOL_NAME) == 0) {
        if (context->spiffsHandle) {
            return SPIFFS_FS;
        } else {
            printf("SPIFFS has not been configured for this project!\n");
            return INVALID_FS;
        }
    }
    printf("Invalid drive name specified.\n");
    return INVALID_FS;
}

/***********************************************************************
 * XMODEM helper functions
 **********************************************************************/
typedef struct XMODEM_STATE {
    SHELL_CONTEXT *ctx;
} XMODEM_STATE;

static void shell_xmodem_putchar(unsigned char c, void *usr)
{
    XMODEM_STATE *x = (XMODEM_STATE *)usr;
    SHELL_CONTEXT *ctx = x->ctx;
    TERM_STATE *t = &ctx->t;

    t->term_out(c, ctx->usr);
}

static int shell_xmodem_getchar(int timeout, void *usr)
{
    XMODEM_STATE *x = (XMODEM_STATE *)usr;
    SHELL_CONTEXT *ctx = x->ctx;
    TERM_STATE *t = &ctx->t;
    int c;

    c = t->term_in(timeout, ctx->usr);

    return(c);

}

typedef struct FLASH_WRITE_STATE {
    XMODEM_STATE xmodem;
    const FLASH_INFO *flash;
    unsigned addr;
    unsigned maxAddr;
    unsigned eraseBlockSize;
} FLASH_WRITE_STATE;

int flashDataWrite(unsigned char *data, unsigned size, bool final, void *usr)
{
   FLASH_WRITE_STATE *state = (FLASH_WRITE_STATE *)usr;
   int err;

   if (size > 0) {
      if ((state->addr % state->eraseBlockSize) == 0) {
         err = flash_erase(state->flash, state->addr, state->eraseBlockSize);
         if (err != FLASH_OK) {
            return(XMODEM_ERROR_GENERIC);
         }
      }
      if ((state->addr + size) <= state->maxAddr) {
         err = flash_program(state->flash, state->addr, (const unsigned char *)data, size);
         if (err != FLASH_OK) {
            return(XMODEM_ERROR_GENERIC);
         }
         state->addr += size;
      }
   }
   return(XMODEM_ERROR_NONE);
}

typedef struct FILE_WRITE_STATE {
    XMODEM_STATE xmodem;
    FILE *f;
} FILE_WRITE_STATE;

int fileDataWrite(unsigned char *data, unsigned size, bool final, void *usr)
{
    FILE_WRITE_STATE *state = (FILE_WRITE_STATE *)usr;
    size_t wsize;

    if (size > 0) {
        wsize = fwrite(data, sizeof(*data), size, state->f);
        if (wsize != size) {
            return(XMODEM_ERROR_CALLBACK);
        }
    }
    return(XMODEM_ERROR_NONE);
}

int confirmDanger(SHELL_CONTEXT *ctx, char *warnStr)
{
    char c;

    printf( "%s\n", warnStr );
    printf( "Are you sure you want to continue? [y/n]\n" );

    c = term_getch( &ctx->t, TERM_INPUT_WAIT );
    printf( "%c\n", isprint( c ) ? c : ' ' );

    if( tolower(c) == 'y' ) {
        return(1);
    }

    return(0);
}

/***********************************************************************
 * CMD: recv
 **********************************************************************/
const char shell_help_recv[] = "<file>\n"
    "  Transfer and save to file\n";
const char shell_help_summary_recv[] = "Receive a file via XMODEM";

void shell_recv( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    FILE_WRITE_STATE fileState;
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
    printf( "Waiting for file ... " );
    size = xmodem_receive(fileDataWrite, &fileState,
        shell_xmodem_putchar, shell_xmodem_getchar);
    if (size < 0) {
        printf( "XMODEM Error: %ld\n", size);
    } else {
        printf( "received and saved as %s\n", argv[ 1 ] );
    }
    fclose( fileState.f );
}

/***********************************************************************
 * CMD: i2c
 **********************************************************************/
const char shell_help_i2c[] = "<i2c_port> <i2c_addr> <reg_addr> <length>\n"
  "  i2c_port - I2C port to probe\n"
  "  i2c_addr - I2C device address\n"
  "  reg_addr - Starting address register dump\n"
  "  length - Number of bytes to dump\n";
const char shell_help_summary_i2c[] = "Executes an I2C write/read transaction";

void shell_i2c(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    sTWI *twiHandle;
    TWI_SIMPLE_PORT twiPort;
    bool useLocalTwiHandle;
    uint16_t  reg_addr;
    uint8_t  i2c_addr;
    uint16_t  length;
    uint16_t  addrLength;
    uint8_t twiWrBuffer[2];
    uint8_t *twiRdBuffer;
    TWI_SIMPLE_RESULT result;
    int i;

    if (argc != 5) {
        printf( "Invalid arguments. Type help [<command>] for usage.\n" );
        return;
    }

    twiPort = (TWI_SIMPLE_PORT)strtol(argv[1], NULL, 0);
    twiHandle = NULL;

    if ((twiPort < TWI0) || (twiPort >= TWI_END)) {
        printf("Invalid I2C port!\n");
        return;
    }

    i2c_addr = strtol(argv[2], NULL, 0);
    reg_addr = strtol(argv[3], NULL, 0);
    length = strtol(argv[4], NULL, 0);

    if (length <= 0) {
        length = 1;
    }

    /* Allocate a buffer to read into */
    twiRdBuffer = SHELL_MALLOC(length);

    printf ( "I2C Device (0x%02x): addr 0x%04x, bytes %d (0x%02x)",
        i2c_addr, reg_addr, length, length);

    /* See if requested TWI port is already open globally */
    if ((twiPort == TWI0) && context->twi0Handle) {
        useLocalTwiHandle = false;
        twiHandle = context->twi0Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI2) && context->twi2Handle) {
        useLocalTwiHandle = false;
        twiHandle = context->twi2Handle;
        result = TWI_SIMPLE_SUCCESS;
    }
    if (twiHandle == NULL) {
        useLocalTwiHandle = true;
        result = twi_open(twiPort, &twiHandle);
    }

    /* Do write/read of peripheral at the specified address */
    if (result == TWI_SIMPLE_SUCCESS) {
        if (reg_addr > 255) {
            twiWrBuffer[0] = (reg_addr >> 8) & 0xFF;
            twiWrBuffer[1] = (reg_addr >> 0) & 0xFF;
            addrLength = 2;
        } else {
            twiWrBuffer[0] = reg_addr;
            addrLength = 1;
        }
        result = twi_writeRead(twiHandle, i2c_addr, twiWrBuffer, addrLength, twiRdBuffer, length);
        if (result == TWI_SIMPLE_SUCCESS) {
            for (i = 0; i < length; i++) {
                if ((i % 16) == 0) {
                    printf("\n");
                    printf("%02x: ", i + reg_addr);
                }
                printf("%02x ", twiRdBuffer[i]);
            }
            printf("\n");
        } else {
            printf("\n");
            printf("twi twi_writeRead() error %d\n", result);
        }
    } else {
        printf("\n");
        printf ("twi open error %d\n", result);
    }

    /* Free the read buffer */
    if (twiRdBuffer) {
        SHELL_FREE(twiRdBuffer);
    }

    if (useLocalTwiHandle && twiHandle) {
        twi_close(&twiHandle);
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
    bool useLocalTwiHandle;
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

    if ((twiPort < TWI0) || (twiPort >= TWI_END)) {
        printf("Invalid I2C port!\n");
        return;
    }

    printf ( "Probing I2C port %d:\n", twiPort);

    /* See if requested TWI port is already open globally */
    if ((twiPort == TWI0) && context->twi0Handle) {
        useLocalTwiHandle = false;
        twiHandle = context->twi0Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI2) && context->twi2Handle) {
        useLocalTwiHandle = false;
        twiHandle = context->twi2Handle;
        result = TWI_SIMPLE_SUCCESS;
    }
    if (twiHandle == NULL) {
        useLocalTwiHandle = true;
        result = twi_open(twiPort, &twiHandle);
    }

    if (result == TWI_SIMPLE_SUCCESS) {
        for (i = 0; i < 128; i++) {
            result = twi_write(twiHandle, i, NULL, 0);
            if (result == TWI_SIMPLE_SUCCESS) {
                printf(" Found device 0x%02x\n", i);
            }
        }
    } else {
        printf ("twi open error %d\n", result);
    }

    if (useLocalTwiHandle && twiHandle) {
        twi_close(&twiHandle);
    }
}

/***********************************************************************
 * CMD: syslog
 **********************************************************************/
const char shell_help_syslog[] = "\n";
const char shell_help_summary_syslog[] = "Show the live system log";

void shell_syslog(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    int c;

    c = 0;
    do {
#ifdef FREE_RTOS
        if (c < 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
#endif
        syslog_dump(SYSLOG_MAX_LINES);
        c = term_getch(&ctx->t, TERM_INPUT_DONT_WAIT);
    } while (c < 0);
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
      if (ent->fdate) {
          year = ((ent->fdate >> 9) & 0x7F) + 1980;
          month = (ent->fdate >> 5) & 0xF;
          day = (ent->fdate >> 0) & 0x1F;
          printf("%04u-%02u-%02u ", year, month, day);
    }
      if (ent->ftime) {
          hour = (ent->ftime >> 11) & 0x1F;
          min = (ent->ftime >> 5) & 0x3F;
          sec = ((ent->ftime >> 0) & 0x1F) * 2;
          printf("%02u:%02u:%02u ", hour, min, sec);
      }
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
            device = SPIFFS_VOL_NAME;
        }
    } else {
        device = argv[1];
    }

    shell_ls_helper( ctx, device, 0, &phasdirs );
}

/***********************************************************************
 * CMD: format
 **********************************************************************/
const char shell_help_format[] = "["SPIFFS_VOL_NAME"]\n";
const char shell_help_summary_format[] = "Formats the internal flash filesystem";

#include "spiffs_fs.h"

void shell_format(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    int fs = INVALID_FS;

    fs = fsVolOK(ctx, context, (argc > 1) ? argv[1] : "");
    if (fs == INVALID_FS) {
        return;
    }

    printf("Be patient, this may take a while.\n");
    printf("Formatting %s...\n", argv[1]);

    if (fs == SPIFFS_FS) {
        spiffs_format(context->spiffsHandle);
    }

    printf("Done. Please reset the board to mount the filesystem.\n");
}

/***********************************************************************
 * CMD: discover
 **********************************************************************/
#include "init.h"
#include "a2b_xml.h"
#include "adi_a2b_cmdlist.h"

const char shell_help_discover[] = "<a2b.xml> <verbose> <i2c_port> <i2c_addr> <reset>\n"
  "  a2b.xml  - A SigmaStudio A2B XML config export file\n"
  "             default 'a2b.xml'\n"
  "  verbose  - Print out results to 0:none, 1:stdout, 2:syslog\n"
  "             default: 1\n"
  "  i2c_port - Set the AD242x transceiver I2C port.\n"
  "             default: 0 (TWI0)\n"
  "  i2c_addr - Set the AD2425 transceiver I2C address\n"
  "             default: 0x68\n"
  "  reset    - Reset the A2B transceiver(s) 'y':yes, 'n':no\n"
  "             default: 'y'\n";
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

typedef int (*PF)(const char *restrict format, ...);

void shell_discover(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    const char *fileName = "a2b.xml";
    ADI_A2B_CMDLIST_RESULT cmdListResult;
    void *a2bInitSequence;
    uint32_t a2bIinitLength;
    int verbose;
    bool useLocalTwiHandle;
    sTWI *twiHandle;
    TWI_SIMPLE_RESULT result;
    TWI_SIMPLE_PORT twiPort;
    uint8_t ad2425I2CAddr;
    bool reset;
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
        .usr = context
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
        pf = printf;
    } else if (verbose == 2) {
        pf = (PF)syslog_printf;
    } else {
        pf = NULL;
    }

    /* Determine TWI port */
    if (argc >= 4) {
        twiPort = strtol(argv[3], NULL, 0);
    } else {
        twiPort = 0;
        }
    if ((twiPort < TWI0) || (twiPort >= TWI_END)) {
        if (pf) {
            pf("Invalid I2C port!\n");
        }
        return;
    }

    /* See if requested TWI port is already open globally */
    if ((twiPort == TWI0) && context->twi0Handle) {
        useLocalTwiHandle = false;
        twiHandle = context->twi0Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI2) && context->twi2Handle) {
        useLocalTwiHandle = false;
        twiHandle = context->twi2Handle;
        result = TWI_SIMPLE_SUCCESS;
    }
    if (twiHandle == NULL) {
        useLocalTwiHandle = true;
        result = twi_open(twiPort, &twiHandle);
    }

    /* Save the TWI handle to use */
    cfg.handle = (void *)twiHandle;

    /* Determine AD2425 I2C address */
    if (argc >= 5) {
        ad2425I2CAddr = strtol(argv[4], NULL, 0);
    } else {
        ad2425I2CAddr = AD2425W_SAM_I2C_ADDR;
    }

    /* Reset the transceiver */
    if (argc >= 6) {
        reset = toupper(argv[5][0]) == 'Y';
    } else {
        reset = true;
    }

    /* Reset the AD2425 and A2B network */
    if (reset) {
        ad2425_reset(context);
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
                pf("WARNING: I2SGCFG mismatch (expected %02x, got %02x)\n",
                    SYSTEM_I2SGCFG, scanInfo.I2SGCFG);
                pf("         Overriding...\n");
            }
            overrideInfo.I2SGCFG_override = true;
            overrideInfo.I2SGCFG = SYSTEM_I2SGCFG;
        }
        if (scanInfo.I2SCFG_valid && (scanInfo.I2SCFG != SYSTEM_I2SCFG)) {
            if (pf) {
                pf("WARNING: I2SCFG mismatch (expected %02x, got %02x)\n",
                    SYSTEM_I2SCFG, scanInfo.I2SCFG);
                pf("         Overriding...\n");
            }
            overrideInfo.I2SCFG_override = true;
            overrideInfo.I2SCFG = SYSTEM_I2SCFG;
        }

        /* Process any overrides */
        cmdListResult = adi_a2b_cmdlist_override(list, &overrideInfo);

        /* Run the command list */
        cmdListResult = adi_a2b_cmdlist_execute(list, &execInfo);

        if (pf) {
            pf("A2B config lines processed: %lu\n", execInfo.linesProcessed);
            pf("A2B discovery result: %s\n", execInfo.resultStr);
            pf("A2B nodes discovered: %d\n", execInfo.nodesDiscovered);
        }

        /* Close the command list */
        cmdListResult = adi_a2b_cmdlist_close(&list);

        /* Free the network config */
        a2b_xml_free(a2bInitSequence, a2bIinitLength, a2bCmdType);

    } else {
        if (pf) {
            pf("Error loading '%s' A2B init XML file", fileName);
        }
    }

    if (useLocalTwiHandle && twiHandle) {
        twi_close(&twiHandle);
    }

}

/***********************************************************************
 * CMD: df
 **********************************************************************/
const char shell_help_df[] = "["SPIFFS_VOL_NAME"]\n";
const char shell_help_summary_df[] = "Shows internal filesystem disk full status";

#include "spiffs.h"

void shell_df( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    u32_t size, used;
    s32_t serr;
    int fs = INVALID_FS;

    fs = fsVolOK(ctx, context, (argc > 1) ? argv[1] : "");
    if (fs == INVALID_FS) {
        return;
    }

    printf("%-10s %10s %10s %10s %5s\n", "Filesystem", "Size", "Used", "Available", "Use %");

    if (fs == SPIFFS_FS) {
        serr = SPIFFS_info(context->spiffsHandle, &size, &used);
        if (serr == SPIFFS_OK) {
          printf("%-10s %10u %10u %10u %5u\n", SPIFFS_VOL_NAME,
            (unsigned)size, (unsigned)used, (unsigned)(size - used),
            (unsigned)((100 * used) / size));
        }
    }
}

/***********************************************************************
 * CMD: rm/del
 **********************************************************************/
#include <unistd.h>

const char shell_help_rm[] = "<file1> [<file2> ...]\n";
const char shell_help_summary_rm[] = "Removes a file";

void shell_rm( SHELL_CONTEXT *ctx, int argc, char **argv )
{
  int i;

  if (argc < 2) {
    printf( "Usage: rm <file1> [<file2> ...]\n" );
    return;
  }

  for (i = 1; i < argc; i++) {
    if (unlink(argv[i]) != 0) {
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
const char shell_help_run[] = "<file1> [<file2> ...]\n";
const char shell_help_summary_run[] = "Runs a command file";

void shell_run( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    FILE *f = NULL;
    char *cmd = NULL;
    char *ok = NULL;
    int i;

    if (argc < 2) {
        printf( "Usage: run <file1> [<file2> ...]\n" );
        return;
    }

    cmd = SHELL_MALLOC(SHELL_MAX_LINE_LEN);

    for (i = 1; i < argc; i++) {
        f = fopen(argv[i], "r");
        if (f) {
            ok = NULL;
            do {
                ok = fgets(cmd, SHELL_MAX_LINE_LEN, f);
                if (ok == cmd) {
                    if (cmd[0] == ';' || cmd[0] == '#') {
                        continue;
                    }
                    shell_exec(ctx, cmd);
                }
            } while (ok);
            fclose(f);
        } else {
            printf("Failed to open '%s'\n", argv[i]);
        }
    }

    SHELL_FREE(cmd);
}

/***********************************************************************
 * CMD: stacks
 **********************************************************************/
const char shell_help_stacks[] = "\n";
const char shell_help_summary_stacks[] = "Report task stack usage";

void shell_stacks( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    APP_CONTEXT *context = &mainAppContext;

    printf("High Water Marks are in 32-bit words (zero is bad).\n");
    printf("Task Stask High Water Marks:\n");
    if (context->startupTaskHandle) {
        printf(" %s: %u\n",
            pcTaskGetName(context->startupTaskHandle),
            (unsigned)uxTaskGetStackHighWaterMark(context->startupTaskHandle));
    }
    if (context->houseKeepingTaskHandle) {
        printf(" %s: %u\n",
            pcTaskGetName(context->houseKeepingTaskHandle),
            (unsigned)uxTaskGetStackHighWaterMark(context->houseKeepingTaskHandle));
    }
    if (context->uac2TaskHandle) {
        printf(" %s: %u\n",
            pcTaskGetName(context->uac2TaskHandle),
            (unsigned)uxTaskGetStackHighWaterMark(context->uac2TaskHandle));
    }
    if (context->wavSrcTaskHandle) {
        printf(" %s: %u\n",
            pcTaskGetName(context->wavSrcTaskHandle),
            (unsigned)uxTaskGetStackHighWaterMark(context->wavSrcTaskHandle));
    }
    if (context->wavSinkTaskHandle) {
        printf(" %s: %u\n",
            pcTaskGetName(context->wavSinkTaskHandle),
            (unsigned)uxTaskGetStackHighWaterMark(context->wavSinkTaskHandle));
    }
    if (context->pollStorageTaskHandle) {
        printf(" %s: %u\n",
            pcTaskGetName(context->pollStorageTaskHandle),
            (unsigned)uxTaskGetStackHighWaterMark(context->pollStorageTaskHandle));
    }
    if (context->a2bSlaveTaskHandle) {
        printf(" %s: %u\n",
            pcTaskGetName(context->a2bSlaveTaskHandle),
            (unsigned)uxTaskGetStackHighWaterMark(context->a2bSlaveTaskHandle));
    }
}

/***********************************************************************
 * CMD: cpu
 **********************************************************************/
#include "cpu_load.h"
#include "clock_domain.h"

const char shell_help_cpu[] = "\n";
const char shell_help_summary_cpu[] = "Report cpu usage";

void shell_cpu( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    uint32_t percentCpuLoad, maxCpuLoad;
    int i;

    percentCpuLoad = cpuLoadGetLoad(&maxCpuLoad, true);
    printf("ARM CPU Load: %u%% (%u%% peak)\n",
        (unsigned)percentCpuLoad, (unsigned)maxCpuLoad);
    printf("SHARC0 Load:\n");
    for (i = 0; i < CLOCK_DOMAIN_MAX; i++) {
        printf(" %s: %lu\n", clock_domain_str(i), context->sharc0Cycles[i]);
    }
    printf("SHARC1 Load:\n");
    for (i = 0; i < CLOCK_DOMAIN_MAX; i++) {
        printf(" %s: %lu\n", clock_domain_str(i), context->sharc1Cycles[i]);
    }
}

/***********************************************************************
 * CMD: usb
 **********************************************************************/
#include "buffer_track.h"
#include "cpu_load.h"
#include "clocks.h"
#include "clock_domain.h"

const char shell_help_usb[] = "[in|out|domain] [a2b|system]\n";
const char shell_help_summary_usb[] = "Displays USB performance tracking metrics";

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

    fs = fsVolOK(ctx, context, (argc > 1) ? argv[1] : "");
    if (fs == INVALID_FS) {
        return;
    }

    printf("Be patient, this may take a while.\n");

    if (fs == SPIFFS_FS) {
        ok = SPIFFS_check(context->spiffsHandle);
        if (ok == SPIFFS_OK) {
            printf(SPIFFS_VOL_NAME " OK\n");
        } else {
            printf(SPIFFS_VOL_NAME " CORRUPT: %d\n", (int)ok);
        }
    }
}

/***********************************************************************
 * CMD: update
 **********************************************************************/
#include "flash_map.h"

const char shell_help_update[] = "<app,fs>\n";
const char shell_help_summary_update[] = "Updates the firmware via xmodem";

void shell_update(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    char *warn = "Updating the firmware is DANGEROUS - DO NOT REMOVE POWER!";
    uint32_t flashBaseAddr;
    uint32_t flashSize;
    long size;
    FLASH_WRITE_STATE state;
    const FLASH_INFO *flash;
    bool checkFs;
    p_xm_data_func dataWriteFunc;

    checkFs = false;
    dataWriteFunc = flashDataWrite;

    if (argc > 1) {
        if ((argc == 2) && (strcmp(argv[1], "app") == 0)) {
           flashBaseAddr = APP_OFFSET;
           flashSize = APP_SIZE;
        } else if ((argc == 2) && (strcmp(argv[1], "fs") == 0)) {
           flashBaseAddr = SPIFFS_OFFSET;
           flashSize = SPIFFS_SIZE;
           checkFs = true;
        }  else {
           printf( "Usage: %s %s", argv[0], shell_help_update);
           return;
        }
    } else {
        flashBaseAddr = APP_OFFSET;
        flashSize = APP_SIZE;
    }

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
    state.addr = flashBaseAddr;
    state.maxAddr = flashBaseAddr + flashSize;
    state.eraseBlockSize = ERASE_BLOCK_SIZE;
    state.xmodem.ctx = ctx;

    /* Start the update */
    printf( "Start XMODEM transfer now... ");
    size = xmodem_receive(dataWriteFunc, &state,
        shell_xmodem_putchar, shell_xmodem_getchar);

    /* Wait a bit */
    delay(100);

    /* Display results and erase any remaining space */
    if (size < 0) {
       printf( "XMODEM Error: %ld\n", size);
    } else {
       printf("Received %ld bytes.\n", size);
    }

    if (checkFs) {
       shell_fsck(ctx, 0, NULL);
    }

    printf("Done.\n");
}

/***********************************************************************
 * CMD: meminfo
 **********************************************************************/
const char shell_help_meminfo[] = "\n";
const char shell_help_summary_meminfo[] = "Displays UMM_MALLOC heap statistics";

#include "sae.h"
#include "umm_malloc_cfg.h"
#include "umm_malloc_heaps.h"
const static char *heapNames[] = UMM_HEAP_NAMES;

void shell_meminfo(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    UMM_HEAP_INFO ummHeapInfo;
    SAE_HEAP_INFO saeHeapInfo;
    SAE_RESULT saeOk;
    int i;
    int ok;

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

    saeOk = sae_heapInfo(context->saeContext, &saeHeapInfo);
    if (saeOk) {
        printf("SAE Heap Info:\n");
        printf("   Blocks: Total  %8u, Allocated %8u, Free %8u\n",
            saeHeapInfo.totalBlocks,
            saeHeapInfo.allocBlocks,
            saeHeapInfo.freeBlocks
        );
        printf("    Bytes: Contig %8u, Allocated %8u, Free %8u\n",
            saeHeapInfo.maxContigFreeSize,
            saeHeapInfo.allocSize,
            saeHeapInfo.freeSize
        );
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
    bool useLocalTwiHandle;
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
        .delay = shell_discover_delay,
        .getTime = shell_discover_get_time,
        .getBuffer = shell_discover_get_buffer,
        .freeBuffer = shell_discover_free_buffer,
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
        pf = printf;
    } else if (verbose == 2) {
        pf = (PF)syslog_printf;
    } else {
        pf = NULL;
    }

    /* Determine TWI port */
    if (argc >= 4) {
        twiPort = strtol(argv[3], NULL, 0);
    } else {
        twiPort = 0;
        }
    if ((twiPort < TWI0) || (twiPort >= TWI_END)) {
        if (pf) {
            pf("Invalid I2C port!\n");
        }
        return;
    }

    /* See if requested TWI port is already open globally */
    if ((twiPort == TWI0) && context->twi0Handle) {
        useLocalTwiHandle = false;
        twiHandle = context->twi0Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI2) && context->twi2Handle) {
        useLocalTwiHandle = false;
        twiHandle = context->twi2Handle;
        result = TWI_SIMPLE_SUCCESS;
    }
    if (twiHandle == NULL) {
        useLocalTwiHandle = true;
        result = twi_open(twiPort, &twiHandle);
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
                pf("Error processing command list\n");
            }
        }

        /* Close the command list */
        adi_a2b_cmdlist_close(&list);

        /* Free the network config */
        a2b_xml_free(a2bInitSequence, a2bIinitLength, a2bCmdType);

    } else {
        if (pf) {
            pf("Error loading '%s' A2B init XML file\n", fileName);
        }
    }

    if (useLocalTwiHandle && twiHandle) {
        twi_close(&twiHandle);
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
 * CMD: sdtest
 **********************************************************************/
const char shell_help_sdtest[] =
    "[ms]\n"
    "  ms - Number of milliseconds to read/write (default 5000)\n";

const char shell_help_summary_sdtest[] =
    "Test the write/read speed of the SD card";

#include "wav_file_cfg.h"

#define SDTEST_MS   (5000)
#define IO_BUF_SIZE (WAVE_FILE_BUF_SIZE)
#define RWBUFSIZE   (IO_BUF_SIZE/4)

void shell_sdtest(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    char *fname = "sd:jo11IEjP6i.bin";
    char *fbuf = NULL;
    char *vbuf = NULL;
    FILE *f = NULL;
    uint32_t begin, end;
    unsigned long KBPS;
    size_t sz, rwsz;
    uint32_t ms;

    ms = SDTEST_MS;
    if (argc > 1) {
        ms = atoi(argv[1]);
    }

    printf("Testing SD card read/write for %u mS...\n", (unsigned)ms);

    vbuf = (char *)WAVE_FILE_CALLOC(IO_BUF_SIZE, 1);
    fbuf = (char *)WAVE_FILE_CALLOC(RWBUFSIZE, 1);

    /* Write Test */
    f = fopen(fname, "wb");
    if (f && vbuf && fbuf) {
        setvbuf(f, vbuf, _IOFBF, IO_BUF_SIZE);

        memset(fbuf, 0xAA, RWBUFSIZE);
        sz = 0;
        begin = end = rtosTimeMs();
        while ((end - begin) < ms) {
            rwsz = fwrite(fbuf, 1, RWBUFSIZE, f);
            sz += rwsz;
            end = rtosTimeMs();
        }

        KBPS = (unsigned long)sz / (unsigned long)(end - begin);

        printf("Write: %lu KB/s (%u Bytes, %u ms)\n",
            KBPS, (unsigned)sz, (unsigned)(end - begin));
    }
    if (f) {
        fclose(f); f = NULL;
    }

    /* Read Test */
    f = fopen(fname, "rb");
    if (f && vbuf && fbuf) {
        setvbuf(f, vbuf, _IOFBF, IO_BUF_SIZE);

        sz = 0;
        begin = end = rtosTimeMs();
        while ((end - begin) < ms) {
            rwsz = fread(fbuf, 1, RWBUFSIZE, f);
            if (rwsz == 0) {
                fseek(f, 0, SEEK_SET);
            }
            sz += rwsz;
            end = rtosTimeMs();
        }

        KBPS = (unsigned long)sz / (unsigned long)(end - begin);

        printf("Read: %lu KB/s (%u Bytes, %u ms)\n",
            KBPS, (unsigned)sz, (unsigned)(end - begin));

        fclose(f); f = NULL;
    }
    if (f) {
        fclose(f); f = NULL;
    }

    if (f) {
        fclose(f);
    }
    if (vbuf) {
        WAVE_FILE_FREE(vbuf);
    }
    if (fbuf) {
        WAVE_FILE_FREE(fbuf);
    }

    unlink(fname);
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
    "  wav        - WAV file src/sink\n"
    "  off        - Turn off the stream\n"
    " No arguments\n"
    "  Show routing table\n"
    " Single 'clear' argument\n"
    "  Clear routing table\n";
const char shell_help_summary_route[] = "Configures the audio routing table";

static char *stream2str(int streamID)
{
    char *str = "NONE";

    switch (streamID) {
        case IPC_STREAMID_UNKNOWN:
            str = "NONE";
            break;
        case IPC_STREAMID_CODEC_IN:
            str = "CODEC_IN";
            break;
        case IPC_STREAMID_CODEC_OUT:
            str = "CODEC_OUT";
            break;
        case IPC_STREAMID_SPDIF_IN:
            str = "SPDIF_IN";
            break;
        case IPC_STREAMID_SPDIF_OUT:
            str = "SPDIF_OUT";
            break;
        case IPC_STREAMID_A2B_IN:
            str = "A2B_IN";
            break;
        case IPC_STREAMID_A2B_OUT:
            str = "A2B_OUT";
            break;
        case IPC_STREAMID_USB_RX:
            str = "USB_RX";
            break;
        case IPC_STREAMID_USB_TX:
            str = "USB_TX";
            break;
        case IPC_STREAM_ID_WAV_SRC:
            str = "WAV_SRC";
            break;
        case IPC_STREAM_ID_WAV_SINK:
            str = "WAV_SINK";
            break;
        default:
            str = "UNKNOWN";
            break;
    }

    return(str);
}

int str2stream(char *stream, bool src)
{
    if (strcmp(stream, "usb") == 0) {
        return(src ? IPC_STREAMID_USB_RX : IPC_STREAMID_USB_TX);
    } else if (strcmp(stream, "codec") == 0) {
        return(src ? IPC_STREAMID_CODEC_IN : IPC_STREAMID_CODEC_OUT);
    } else if (strcmp(stream, "spdif") == 0) {
        return(src ? IPC_STREAMID_SPDIF_IN : IPC_STREAMID_SPDIF_OUT);
    } else if (strcmp(stream, "a2b") == 0) {
        return(src ? IPC_STREAMID_A2B_IN : IPC_STREAMID_A2B_OUT);
    } else if (strcmp(stream, "wav") == 0) {
        return(src ? IPC_STREAM_ID_WAV_SRC : IPC_STREAM_ID_WAV_SINK);
    } else if (strcmp(stream, "off") == 0) {
        return(IPC_STREAMID_UNKNOWN);
    }

    return(IPC_STREAM_ID_MAX);
}

void shell_route(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    IPC_MSG_ROUTING *routeInfo = (IPC_MSG_ROUTING *)&context->routingMsg->routes;
    ROUTE_INFO *route;
    unsigned i;
    unsigned idx, srcOffset, sinkOffset, channels, attenuation, mix;
    int srcID, sinkID;

    if (argc == 1) {
        printf("Audio Routing\n");
        for (i = 0; i < routeInfo->numRoutes; i++) {
            route = &routeInfo->routes[i];
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
            for (i = 0; i < routeInfo->numRoutes; i++) {
                route = &routeInfo->routes[i];
                /* Configure the route atomically in a critical section */
                taskENTER_CRITICAL();
                route->srcID = IPC_STREAMID_UNKNOWN;
                route->srcOffset = 0;
                route->sinkID = IPC_STREAMID_UNKNOWN;
                route->sinkOffset = 0;
                route->channels = 0;
                route->attenuation = 0;
                route->mix = 0;
                taskEXIT_CRITICAL();
            }
        }
    }

    /* Confirm a valid route index */
    idx = atoi(argv[1]);
    if (idx > routeInfo->numRoutes) {
        printf("Invalid idx\n");
        return;
    }

    route = &routeInfo->routes[idx];
    srcID = route->srcID;
    srcOffset = route->srcOffset;
    sinkID = route->sinkID;
    sinkOffset = route->sinkOffset;
    channels = route->channels;
    mix = route->mix;

    /* Gather the source info */
    if (argc >= 3) {
        srcID = str2stream(argv[2], true);
        if (srcID == IPC_STREAM_ID_MAX) {
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
        if (sinkID == IPC_STREAM_ID_MAX) {
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
 * CMD: wav
 **********************************************************************/
const char shell_help_wav[] = "<src|sink> <on|off> [file] [channels] [bits]\n";
const char shell_help_summary_wav[] = "Manages wave file source/sink";

#include "wav_file.h"
#include "clock_domain.h"

static void wav_state(SHELL_CONTEXT *ctx, char *name, int clockDomainMask, WAV_FILE *wf)
{
    printf(
        "%s: %s, %s, %d-bit, %d ch, %s\n",
        name,
        wf->enabled ? "ON" : "OFF",
        wf->fname ? wf->fname : "N/A",
        wf->wordSizeBytes == 2 ? 16 : 32,
        wf->channels,
        clock_domain_str(clock_domain_get(context, clockDomainMask))
    );
}

void shell_wav( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    WAV_FILE *wf = NULL;
    int channels;
    int wordSizeBytes;
    int bits;
    char *fname = NULL;
    bool on;
    bool isSrc;
    bool ok = true;
    int clockDomainMask;
    bool channelsSpecified = false;

    if (argc == 1) {
        wav_state(ctx, "Src", CLOCK_DOMAIN_BITM_WAV_SRC, &context->wavSrc);
        wav_state(ctx, "Sink", CLOCK_DOMAIN_BITM_WAV_SINK, &context->wavSink);
        return;
    }

    if (argc >= 2) {
        if (strcmp(argv[1], "src") == 0) {
            wf = &context->wavSrc;
            fname = "src.wav";
            isSrc = true;
            clockDomainMask = CLOCK_DOMAIN_BITM_WAV_SRC;
        } else if (strcmp(argv[1], "sink") == 0) {
            wf = &context->wavSink;
            fname = "sink.wav";
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

    if (argc >= 3) {
        if (strcmp(argv[2], "on") == 0) {
            if (wf->enabled) {
                printf("Already on\n");
                return;
            }
            on = true;
        } else if (strcmp(argv[2], "off") == 0) {
            if (!wf->enabled) {
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
        fname = argv[3];
    } else {
        if (wf->fname) {
            fname = NULL;
        }
    }

    channels = 2;
    if (argc >= 5) {
        channels = atoi(argv[4]);
        if (channels > WAV_MAX_CHANNELS) {
            channels = WAV_MAX_CHANNELS;
        }
        channelsSpecified = true;
    }

    wordSizeBytes = sizeof(int16_t);
    if (argc >= 6) {
        bits = atoi(argv[5]);
        if (bits == 32) {
            wordSizeBytes = 4;
        }
    }

    xSemaphoreTake((SemaphoreHandle_t)wf->lock, portMAX_DELAY);
    if (on) {
        if (!isSrc) {
            wf->channels = channels;
            wf->sampleRate = SYSTEM_SAMPLE_RATE;
            wf->wordSizeBytes = wordSizeBytes;
            wf->frameSizeBytes = SYSTEM_BLOCK_SIZE * wf->wordSizeBytes;
        }
        if (fname) {
            if (wf->fname) {
                SHELL_FREE(wf->fname); wf->fname = NULL;
            }
            wf->fname = SHELL_MALLOC(strlen(fname) + 1);
            strcpy(wf->fname, fname);
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
        closeWave(wf);
    }
    xSemaphoreGive((SemaphoreHandle_t)wf->lock);
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
  "  mode [main | sub]   - Bus mode 'main node' or 'sub node'\n"
  "                           (default 'main')\n"
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
            ok = ad2425_set_mode(context, mode);
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


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

/*!
 * @brief  Simple system logger
 *
 * This logger supports FreeRTOS and bare-metal projects.
 *
 * @file      syslog.h
 * @version   1.0.1
 * @copyright 2023 Analog Devices, Inc.  All rights reserved.
 *
*/
#include <stdarg.h>

#ifndef SYSLOG_H_
#define SYSLOG_H_

/*!****************************************************************
 * @brief  System log init
 *
 * This function initializes the system logger.
 *
 * This function can be called before or after the FreeRTOS is
 * started.  The logger will allocate the log buffer using
 * the function defined by SYSLOG_MALLOC
 *
 ******************************************************************/
void syslog_init(void);

/*!****************************************************************
 * @brief  System log print
 *
 * This function prints a fixed string to the system log.  Strings
 * which do not fit within the log line length will be truncated.
 *
 * This function is thread safe.
 *
 * @param [in]   msg     Null-terminated string to save in the log
 *
 ******************************************************************/
void syslog_print(char *msg);

/*!****************************************************************
 * @brief  System log printf
 *
 * This function prints a variable argument string to the system
 * log.  Strings which do not fit within the log line length
 * will be truncated.
 *
 * This function is thread safe.
 *
 * @param [in]  fmt   Null-terminated format string
 * @param [in]  ...   Remaining arguments
 *
 ******************************************************************/
void syslog_printf(char *fmt, ...);

/*!****************************************************************
 * @brief  System log vprintf
 *
 * This function prints a variable argument list to the system
 * log.  Strings which do not fit within the log line length
 * will be truncated.
 *
 * This function is thread safe.
 *
 * @param [in]  fmt   Null-terminated format string
 * @param [in]  args  Variable argument list
 *
 ******************************************************************/
void syslog_vprintf(char *fmt, va_list args);

/*!****************************************************************
 * @brief  System log next line
 *
 * This function returns the next line in the system log or NULL
 * if the log is empty.
 *
 * This function is thread safe.
 *
 ******************************************************************/
char *syslog_next(char *ts, size_t tsMax, char *line, size_t lineMax);

/*!****************************************************************
 * @brief  System log dump
 *
 * This function dumps the contents of the system log to stdout.
 *
 * This function is thread safe.
 *
 ******************************************************************/
void syslog_dump(unsigned max);

#endif

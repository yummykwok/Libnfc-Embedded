/*-
 * Free/Libre Near Field Communication (NFC) library
 *
 * Libnfc historical contributors:
 * Copyright (C) 2009      Roel Verdult
 * Copyright (C) 2009-2013 Romuald Conty
 * Copyright (C) 2010-2012 Romain Tartière
 * Copyright (C) 2010-2013 Philippe Teuwen
 * Copyright (C) 2012-2013 Ludovic Rousseau
 * See AUTHORS file for a more comprehensive list of contributors.
 * Additional contributors of this file:
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#ifndef __LOG_H__
#define __LOG_H__
#ifndef _DLIB_FILE_DESCRIPTOR
#define _DLIB_FILE_DESCRIPTOR 1
#endif

#include "nfc-internal.h"
#include <stdio.h>
#include <stdlib.h>

#define NFC_LOG_PRIORITY_NONE   0
#define NFC_LOG_PRIORITY_ERROR  1
#define NFC_LOG_PRIORITY_INFO   2
#define NFC_LOG_PRIORITY_DEBUG  3

#define NFC_LOG_GROUP_GENERAL   1
#define NFC_LOG_GROUP_CONFIG    2
#define NFC_LOG_GROUP_CHIP      3
#define NFC_LOG_GROUP_DRIVER    4
#define NFC_LOG_GROUP_COM       5
#define NFC_LOG_GROUP_LIBUSB    6

/*
  To enable log only for one (or more) group, you can use this formula:
    log_level = NFC_LOG_PRIORITY(main) + NFC_LOG_PRIORITY(group) * 2 ^ (NFC_LOG_GROUP(group) * 2)

  Examples:
   * Main log level is NONE and only communication group log is set to DEBUG verbosity (for rx/tx trace):
       LIBNFC_LOG_LEVEL=3072  // 0+3072
   * Main log level is ERROR and driver layer log is set to DEBUG level:
       LIBNFC_LOG_LEVEL=769   // 1+768
   * Main log level is ERROR, driver layer is set to INFO and communication is set to DEBUG:
       LIBNFC_LOG_LEVEL=3585  // 1+512+3072
*/

//int log_priority_to_int(const char* priority);
const char *log_priority_to_str(const int priority);
#define LOG
#if defined LOG

#ifndef __has_attribute
#  define __has_attribute(x) 0
#endif

#if __has_attribute(format) || defined(__GNUC__)
#  define __has_attribute_format 1
#endif

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  //#define PRINTF_PROTOTYPE signed int printf(const char *pFormat, ...)
#endif /* __GNUC__ */

void log_enable(bool enabled);
void log_exit(void);
void log_put(const uint8_t group, const char *category, const uint8_t priority, const char *format, ...)
#if __has_attribute_format
__attribute__((format(printf, 4, 5)))
#endif
;
#else
// No logging
#define log_enable(b) ((void) 0)
#define log_exit() ((void) 0)
#define log_put(group, category, priority, format, ...) do {} while (0)

#endif // LOG

/**
 * @macro LOG_HEX
 * @brief Log a byte-array in hexadecimal format
 * Max values:  pcTag of 121 bytes + ": " + 300 bytes of data+ "\0" => acBuf of 1024 bytes
 */
#ifdef LOG
void LOG_HEX(const uint8_t group,const char * pcTag,const u8* pbtData,const u16 szBytes);

#else
#  define LOG_HEX(group, pcTag, pbtData, szBytes) do { \
    (void) group; \
    (void) pcTag; \
    (void) pbtData; \
    (void) szBytes; \
  } while (0);
#endif

#endif // __LOG_H__

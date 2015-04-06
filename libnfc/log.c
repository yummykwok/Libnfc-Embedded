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

#include "log.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stm32f10x.h>
#include <buses/stm32_f107_uart.h>
const char *
log_priority_to_str(const int priority)
{
  switch (priority) {
    case NFC_LOG_PRIORITY_ERROR:
      return  "error";
    case NFC_LOG_PRIORITY_INFO:
      return  "info";
    case NFC_LOG_PRIORITY_DEBUG:
      return  "debug";
    default:
      break;
  }
  return "unknown";
}


#ifdef LOG
//static USART_TypeDef* debug_port = NULL;
static bool logEnabled = false;
extern u8 TxBuffer1[1024];
extern u32 TxFront;
extern u32 TxRear;

PUTCHAR_PROTOTYPE
{
  if(!logEnabled) return 0;
  PutToLogBuffer(ch);
  return ch;
}

/*
PRINTF_PROTOTYPE
{
  va_list ap;
  size_t	 __szPos; 
  char	 __acBuf[512];
  // Forward call to vprintf
  va_start(ap, pFormat);
  sprintf(__acBuf,pFormat, ap);
  va_end(ap);
  __szPos = strlen(__acBuf);
  Uart_InterruptsendY(debug_port,__acBuf,__szPos);
  return __szPos;
}
*/

void
log_enable(bool enabled)
{
  logEnabled = enabled;
}

void
log_exit(void)
{
}

void LOG_HEX(const uint8_t group,const char * pcTag,const u8* pbtData,const u16 szBytes){
  /*
    size_t	 __szPos; 
    char	 __acBuf[512]; 
    size_t	 __szBuf = 0; 
    
    for (__szPos=0; (__szPos < (size_t)(szBytes) && __szPos<sizeof(__acBuf)); __szPos++) {
      sprintf(__acBuf+__szBuf,"%02X ",pbtData[__szPos]);
      __szBuf += 3;
    } 
    printf ("[Debug][uart]%s: %s\r\n", pcTag, __acBuf);
    */
      size_t	 __szPos;
    char	 __acBuf[512];
    size_t	 __szBuf = 0;
    sprintf (__acBuf + __szBuf, "%s: ", pcTag);
    __szBuf += strlen (pcTag) + 2;
    for (__szPos=0; (__szPos < (size_t)(szBytes)) && (__szBuf < sizeof(__acBuf)); __szPos++) {
      sprintf (__acBuf + __szBuf, "%02x ",((uint8_t *)(pbtData))[__szPos]);
      __szBuf += 3;
    }
    printf("[Debug][UART]%s\r\n", __acBuf);
}

void
log_put(const uint8_t group, const char *category, const uint8_t priority, const char *format, ...)
{
  //  printf("log_level = %"PRIu32" group = %"PRIu8" priority = %"PRIu8"\n", log_level, group, priority);
  /*if (log_level) { // If log is not disabled by log_level=none
    if (((log_level & 0x00000003) >= priority) ||   // Global log level
        (((log_level >> (group * 2)) & 0x00000003) >= priority)) { // Group log level

      va_list va;
      va_start(va, format);
      printf("[%s][%s]", log_priority_to_str(priority), category);
      printf(format, va);
      printf("\n");
      va_end(va);
    }
  }*/
}

#endif // LOG

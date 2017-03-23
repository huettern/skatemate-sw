/**
 * @file       util.h
 * @brief      Miscellaneous Utility methods
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup UTIL
 * @brief Utility finctions
 * @{
 */

#include "util.h"

#include "ch.h"
#include "hal.h"
#include "defs.h"

#include "chprintf.h"
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Dump a memory block in a readable hex format to the output
 * stream
 *
 * @param      bss   output stream
 * @param      mem   start memory
 * @param[in]  len   memory length
 */
void hexdump(BaseSequentialStream * bss, void *mem, unsigned int len)
{
    unsigned int i, j;
    if (bss)
    {
        for(i = 0; i < len + ((len % 16) ? (16 - len % 16) : 0); i++)
        {
            /* print offset */
            if(i % 16 == 0)
                chprintf(bss, "0x%06x: ", i);

            /* print hex data */
            if(i < len)
                chprintf(bss,"%02x ", 0xFF & ((char*)mem)[i]);
            else /* end of block, just aligning for ASCII dump */
                chprintf(bss, "   ");

            /* print ASCII dump */
            if(i % 16 == (16 - 1))
            {
                for(j = i - (16 - 1); j <= i; j++)
                {
                    if(j >= len) /* end of block, not really printing */
                        chprintf(bss," ");
                    else if( ((((char*)mem)[j]) > 0x20) && ((((char*)mem)[j]) < 0x7F)) /* printable char */
                        chprintf(bss, "%c", 0xFF & ((char*)mem)[j]);        
                    else /* other char */
                        chprintf(bss, ".");
                }
                chprintf(bss, "\r\n");
            }
        }
        chprintf(bss,"\r\n");
    }
}

/**
 * @brief      strip trailing stripchar from string
 *
 * @param      str       string
 * @param[in]  trimchar  character to trim
 *
 * @return     charpointer to a new string from beginning to stripchar
 */
char * rtrim(char * str, char trimchar)
{
  char *pos;
  pos=strrchr(str, trimchar);
  if (pos != NULL)
      *pos = '\0';

  return str;
}

/**
 * @brief      Tries to lock a mutex for the given time and exits
 *
 * @param      mtx   The mtx
 * param[in] t timeout in ms to wait
 *
 * @return     true if mutex got locked succesfully, false if mutex is already locked
 */
bool lockMtx(mutex_t* mtx, uint32_t t) 
{
  uint32_t ctr = 0;
  while(ctr <= t)
  {
    if(chMtxTryLock(mtx))
    {
      return true;
    }
    chThdSleepMilliseconds(1);
    ctr++;
  }
  return false;
}

/**
 * @brief      Returns the WA Size of the given thread
 *
 * @param      tp    thread pointer
 *
 * @return     The thd wa size.
 */
size_t getThdWaSize(thread_t *tp)
{
  const char* nm = chRegGetThreadNameX(tp);
  if(strcmp(nm,DEFS_THD_IDLE_NAME) == 0) return (size_t)DEFS_THD_IDLE_WA_SIZE;
  if(strcmp(nm,DEFS_THD_SHELL_NAME) == 0) return (size_t)DEFS_THD_SHELL_WA_SIZE;
  return 0;
}



/** @} */


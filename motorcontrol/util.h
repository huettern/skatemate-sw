/**
 * @file       util.h
 * @brief      Miscellaneous Utility methods
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup UTIL
 * @{
 */

#include "ch.h"


void hexdump(BaseSequentialStream * bss, void *mem, unsigned int len);
char * rtrim(char * str, char trimchar);
bool lockMtx(mutex_t* mtx, uint32_t t);
size_t getThdWaSize(thread_t *tp);


/** @} */
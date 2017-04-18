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

/*===========================================================================*/
/* constants                                                                 */
/*===========================================================================*/
#define ONE_BY_SQRT_3 (0.5773503f)
#define TWO_BY_SQRT_3 (1.1547005f)
#define SQRT_3_BY_2   (0.8660254f)
#define SQRT_3        (1.7320508f)
#define PI (3.1415927f)


/*===========================================================================*/
/* UTIL public functions.                                                    */
/*===========================================================================*/
void hexdump(BaseSequentialStream * bss, void *mem, unsigned int len);
char * rtrim(char * str, char trimchar);
bool lockMtx(mutex_t* mtx, uint32_t t);
size_t getThdWaSize(thread_t *tp);




/** @} */
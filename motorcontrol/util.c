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
#include <math.h>
#include "arm_math.h"

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

/**
 * @brief      fast atan2 calculation
 * @note       See http://www.dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization
 *
 * @param[in]  y     y
 * @param[in]  x     x
 *
 * @return     atan2(y,x)
 */
float utilFastAtan2(float y, float x) {
  float abs_y = fabsf(y) + 1e-10; // kludge to prevent 0/0 condition
  float angle, r, rsq;

  if (x >= 0) {
    r = (x - abs_y) / (x + abs_y);
    rsq = r * r;
    angle = ((0.1963 * rsq) - 0.9817) * r + (PI / 4.0);
  } else {
    r = (x + abs_y) / (abs_y - x);
    rsq = r * r;
    angle = ((0.1963 * rsq) - 0.9817) * r + (3.0 * PI / 4.0);
  }

  if (y < 0) {
    return(-angle);
  } else {
    return(angle);
  }
}

/**
 * @brief      Truncate the magnitude of a vector.
 * @note       Duration 7.325us
 *
 * @param x   The first component.
 * @param y   The second component.
 * @param max The maximum magnitude.
 *
 * @return True if saturation happened, false otherwise
 */
bool utils_saturate_vector_2d(float *x, float *y, float max) 
{
  bool retval = false;
  float mag = sqrtf(*x * *x + *y * *y);
  // arm_sqrt_f32(*x * *x + *y * *y, &mag);
  max = fabsf(max);

  if (mag < 1e-10) 
  {
    mag = 1e-10;
  }

  if (mag > max) 
  {
    const float f = max / mag;
    *x *= f;
    *y *= f;
    retval = true;
  }

  return retval;
}

/**
 * @brief      Make sure that -pi <= angle < pi,
 *
 * @param      angle  The angle
 */
void utils_norm_angle_rad(float *angle) 
{
  while (*angle < -PI) 
  {
    *angle += 2.0 * PI;
  }

  while (*angle >  PI) 
  {
    *angle -= 2.0 * PI;
  }
}

/**
 * @brief      Fast sine and cosine implementation.
 * @note       See http://lab.polygonal.de/?p=205
 * @note       WARNING: Don't use too large angles.
 *
 * @param[in]  angle  The angle in radians
 * @param      sin    The sine in radians
 * @param      cos    The cosine in radians
 */
void sincos_fast(float angle, float *sin, float *cos)
{
  //always wrap input angle to -PI..PI
  while (angle < -PI) 
  {
    angle += 2.0 * PI;
  }
  while (angle >  PI) 
  {
    angle -= 2.0 * PI;
  }

  //compute sine
  if (angle < 0.0) 
  {
    *sin = 1.27323954 * angle + 0.405284735 * angle * angle;

    if (*sin < 0.0) 
    {
      *sin = 0.225 * (*sin * -*sin - *sin) + *sin;
    } else 
    {
      *sin = 0.225 * (*sin * *sin - *sin) + *sin;
    }
  } 
  else 
  {
    *sin = 1.27323954 * angle - 0.405284735 * angle * angle;

    if (*sin < 0.0) 
    {
      *sin = 0.225 * (*sin * -*sin - *sin) + *sin;
    } 
    else 
    {
      *sin = 0.225 * (*sin * *sin - *sin) + *sin;
    }
  }

  // compute cosine: sin(x + PI/2) = cos(x)
  angle += 0.5 * PI;
  if (angle >  PI) 
  {
    angle -= 2.0 * PI;
  }

  if (angle < 0.0) 
  {
    *cos = 1.27323954 * angle + 0.405284735 * angle * angle;

    if (*cos < 0.0) 
    {
      *cos = 0.225 * (*cos * -*cos - *cos) + *cos;
    } 
    else 
    {
      *cos = 0.225 * (*cos * *cos - *cos) + *cos;
    }
  } else 
  {
    *cos = 1.27323954 * angle - 0.405284735 * angle * angle;

    if (*cos < 0.0) 
    {
      *cos = 0.225 * (*cos * -*cos - *cos) + *cos;
    } 
    else 
    {
      *cos = 0.225 * (*cos * *cos - *cos) + *cos;
    }
  }
}

/** @} */


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
// #define PI (3.1415927f) // defined in arm_math

/*===========================================================================*/
/* Macros 	                                                                 */
/*===========================================================================*/
/**
 * A simple low pass filter.
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */

/**
 * @brief      Simple low pass filter
 *
 * @param      value            filtered value
 * @param      sample           input sample
 * @param      filter_constant  The filter constant: lower equals more filtering
 *
 * @return     Filtered value. filter_constant is a factor between 0.0 to 1.0
 */
#define UTIL_LP_FAST(value, sample, filter_constant) (value -= (filter_constant) * (value - (sample)))

#define SIGN(x) ((x<0)?-1:1)
/*===========================================================================*/
/* UTIL public functions.                                                    */
/*===========================================================================*/
void hexdump(BaseSequentialStream * bss, void *mem, unsigned int len);
char * rtrim(char * str, char trimchar);
bool lockMtx(mutex_t* mtx, uint32_t t);
size_t getThdWaSize(thread_t *tp);

float utilFastAtan2(float y, float x);
bool utils_saturate_vector_2d(float *x, float *y, float max);
void utils_norm_angle_rad(float *angle);
void sincos_fast(float angle, float *sin, float *cos);
float stof(const char* s);
float wrapAngle(float in);

/** @} */
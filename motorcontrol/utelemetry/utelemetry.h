/**
 * @file       utelemetry.h
 * @brief      Micro telemetry implementation for C
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       24 March 2017
 * 
 *
 * @addtogroup UTELEMETRY
 * @{
 */
#include <stdint.h>
#include <stdbool.h>

/*===========================================================================*/
/* Datatypes                                                                 */
/*===========================================================================*/
/**
 * utlm data types
 */
typedef enum 
{
	utlm_uint8 = 0,
	utlm_int8 = 1,
	utlm_uint16 = 2,
	utlm_int16 = 3,
	utlm_uint32 = 4,
	utlm_int32 = 5,
	utlm_uint64 = 6,
	utlm_int64 = 7,
	utlm_float = 8,
	utlm_double = 9
} utlmDatatype_t;

/**
 * Measurement setting typedef
 */
typedef struct 
{
	uint16_t mid;
	const char* name;
	const char* unit;
	utlmDatatype_t xtype;
	utlmDatatype_t ytype;
} utlmMeasurement_t;

/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/
#define UTLM_MAJOR_VERSION 0
#define UTLM_MINOR_VERSION 1

/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/


/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* callbacks                                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
void utlmEnable(bool en);
void utlmSend(uint16_t mid, uint16_t num, void* xdata, void* ydata);

/** @} */

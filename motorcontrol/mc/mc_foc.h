/**
 * @file       MC_FOC.h
 * @brief      
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       
 * 
 *
 * @addtogroup MC
 * @brief Motor control
 * @{
 */
#ifndef _MC_FOC_H
#define _MC_FOC_H

#include "ch.h"
#include "hal.h"

/*===========================================================================*/
/* Macros				                                                             */
/*===========================================================================*/

/*===========================================================================*/
/* Datatypes                                                                 */
/*===========================================================================*/
typedef struct 
{
  float psi;  // flux linkage
  float p;    // pole pairs
  float Ls;   // series stator inductance
  float Rs;   // series stator resistance
  float J;    // inhertia
} mcfMotorParameter_t;

typedef struct
{
  float obsGain;
  float curr_d_kp;
  float curr_d_ki;
  float curr_d_kd;
  float curr_q_kp;
  float curr_q_ki;
  float curr_q_kd;
  float speed_kp;
  float speed_ki;
  float speed_kd;
} mcfFOCParameter_t;

typedef struct
{
  float x[2];
  float eta[2];
  float y[2];
  float theta;
} mcfObs_t;

/*===========================================================================*/
/* MC_FOC public functions.                                                  */
/*===========================================================================*/
void mcfInit(void);
void mcfSetDuty (uint16_t a, uint16_t b, uint16_t c);


#endif
/** @} */

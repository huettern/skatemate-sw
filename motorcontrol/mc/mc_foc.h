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
#include "arm_math.h"

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
  float obsSpeed_kp;
  float obsSpeed_ki;
  float obsSpeed_kd;
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
  // position observer
  float x[2];
  float eta[2];
  float y[2];
  float theta;  // the estimated position
  // speed observer
  float theta_var;
  float omega_e;  // the estimated electrical speed
  float omega_m;  // the estimated mechanical speed [rpm]
  arm_pid_instance_f32 speedPID;
} mcfObs_t;

typedef struct
{
  float w_set;  // [rpm]
  float w_is;
  float id_set;
  float id_is;
  float iq_set;
  float iq_is;
  float vd_set;
  float vq_set;
  float ia_is;
  float ib_is;
  float ia_set;
  float ib_set;
  float va_set;
  float vb_set;
  float ipa_is;
  float ipb_is;
  float ipc_is;
  arm_pid_instance_f32 speedPID;
  arm_pid_instance_f32 idPID;
  arm_pid_instance_f32 iqPID;
} mcfController_t;

typedef enum {
  MC_HALT,
  MC_OPEN_LOOP,
  MC_CLOSED_LOOP
} mcState_t;

/*===========================================================================*/
/* MC_FOC public functions.                                                  */
/*===========================================================================*/
void mcfInit(void);
void mcfSetDuty (uint16_t a, uint16_t b, uint16_t c);
void mcfDumpData(void);


#endif
/** @} */

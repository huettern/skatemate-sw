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
  float obsSpeed_ceil;
  float obsSpeed_floor;
  float curr_d_kp;
  float curr_d_ki;
  float curr_q_kp;
  float curr_q_ki;
  float speed_kp;
  float speed_ki;
  float iTermCeil;
  float iTermFloor;
  float dutyMax;
  float minErpm;
  float currMin;
  float currMax;
  float currInMin;
  float currInMax;
  float duty_dowmramp_kp;
  float duty_dowmramp_ki;
  float dCurrInjDuty;
  float dCurrInjFactor;
  float openloop_rpm;
  float openloop_hyst;
  float openloop_time;
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
  float duty_set;
  float id_set;
  float id_is;
  float id_is_filter;
  float iq_set;
  float iq_is;
  float iq_is_filter;
  float vd_set;
  float vq_set;
  float vd_is;
  float vq_is;
  float ia_is;
  float ib_is;
  float va_is;
  float vb_is;
  float ia_set;
  float ib_set;
  float va_set;
  float vb_set;
  float ipa_is;
  float ipb_is;
  float ipc_is;
  float vsupply;
  float isupply;
  float vpa_is;
  float vpb_is;
  float vpc_is;
  float fc_f;
  float fc_vd;
  float fc_vq;
  float mod_d;
  float mod_q;
  float phase;
  float duty_now;
  float max_duty;
  float i_abs;
  float i_abs_filter;
} mcfController_t;

typedef enum {
   MC_STATE_OFF = 0,
   MC_STATE_DETECTING,
   MC_STATE_RUNNING,
   MC_STATE_FULL_BRAKE,
} mcState_t;

typedef enum {
  MC_HALT = 0,
  MC_OPEN_LOOP = 1,
  MC_CLOSED_LOOP_CURRENT = 2,
  MC_CLOSED_LOOP_SPEED = 3,
  MC_CONTROL_MODE_DUTY,
  MC_CONTROL_MODE_CURRENT_BRAKE,
  MC_CONTROL_MODE_CURRENT,
  MC_CONTROL_MODE_SPEED
} mcMode_t;

/*===========================================================================*/
/* MC_FOC public functions.                                                  */
/*===========================================================================*/
void mcfInit(void);
void mcfSetDuty (uint16_t a, uint16_t b, uint16_t c);
void mcfDumpData(void);
void mcfStartSample(void);

#endif
/** @} */

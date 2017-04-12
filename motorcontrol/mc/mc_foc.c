/**
 * @file       mc_foc.c
 * @brief      FOC implementation
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       12 April 2017
 * 
 *
 * @addtogroup MC
 * @brief Motor control
 * @{
 */
#include "mc_foc.h"

#include "ch.h"
#include "hal.h"
#include "defs.h"
#include "chprintf.h"
#include "usbcfg.h"

#include "stm32f30x_conf.h"

#define DBG(X, ...) chprintf(bssusb, X, ##__VA_ARGS__ )

/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/
// FOC PWM switching frequencz
#define FOC_F_SW 20000 //Hz

/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/

/*===========================================================================*/
/* prototypes                                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Init the mc FOC peripherals, data and threads
 */
void mcfInit(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

  // TIM1 clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  // Time Base configuration
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned2; // compare flag when upcounting
  TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / FOC_F_SW;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 1; // Only generate update event on underflow

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

}


/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/

/** @} */

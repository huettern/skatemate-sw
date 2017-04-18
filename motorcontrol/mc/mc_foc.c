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

/**
 * PERIPHERAL USAGE OVERVIEW
 * 
 * TIM1
 * - Channel 1
 *    PWM Gate phase C, complementary configuration
 * - Channel 2
 *    PWM Gate phase B, complementary configuration
 * - Channel 3
 *    PWM Gate phase A, complementary configuration
 * - Channel 4
 *    Used for triggering the injected conversion of ADC3, current sensing and
 *    ADC1 3-phase and supply voltage sensing
 */

#include "mc_foc.h"

#include "ch.h"
#include "hal.h"
#include "defs.h"
#include "util.h"

#include "arm_math.h"

#include "stm32f30x_conf.h"



/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/
/**
 * PWM switching frequency
 */
#define FOC_F_SW 20000 //Hz
/**
 * FOC PWM output deadtime cycles. refer to p.592 of reference manual
 * allowed value 0..255
 */
#define FOC_PWM_DEADTIME_CYCLES 0 // cycles
/**
 * Duty cycle for TIM1 Channel 4 set to maximum to get a falling edge in the
 * middle of a Gate low output sequence for current and voltage sensing
 */
#define FOC_TIM1_OC4_VALUE (SYSTEM_CORE_CLOCK / FOC_F_SW) - 1
/**
 * How fast the control thread should run
 */
#define FOC_THREAD_INTERVAL 1000 // us

/*===========================================================================*/
/* macros                                                                    */
/*===========================================================================*/
/**
 * @brief      synchronous update of the PWM duty cycles
 *
 * @param      duty1  dutycycle channel 1
 * @param      duty2  dutycycle channel 2
 * @param      duty3  dutycycle channel 3
 *
 * @return     none
 */
#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
    TIM1->CR1 |= TIM_CR1_UDIS; \
    TIM1->CCR1 = duty1; \
    TIM1->CCR2 = duty2; \
    TIM1->CCR3 = duty3; \
    TIM1->CR1 &= ~TIM_CR1_UDIS;

/**
 * Period, the timer should run on. Calculated by Core clock and FOC_F_SW
 */
#define FOC_TIM_PERIOD SYSTEM_CORE_CLOCK / FOC_F_SW

/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/
/*
 * Working area for the State machine thread
 */
static THD_WORKING_AREA(mcfWA, DEFS_THD_MCFOC_WA_SIZE);
static THD_FUNCTION(mcfocThread, arg);

/*===========================================================================*/
/* prototypes                                                                */
/*===========================================================================*/
static void svm (float* a, float* b, uint16_t* da, uint16_t* db, uint16_t* dc);

static void clark (float* va, float* vb, float* vc, float* a, float* b);
static void park (float* a, float* b, float* theta, float* d, float* q );
static void invclark (float* a, float* b, float* va, float* vb, float* vc);
static void invpark (float* d, float* q, float* theta, float* a, float* b);


/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Init the mc FOC peripherals, data and threads
 */
void mcfInit(void)
{
  TIM_TimeBaseInitTypeDef  tim_tbs;
  TIM_OCInitTypeDef  tim_ocis;
  TIM_BDTRInitTypeDef tim_bdtris;
  ADC_CommonInitTypeDef adc_cis;
  ADC_InitTypeDef adc_is;
  DMA_InitTypeDef dma_is;

  // TIM1 clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /**
   * TIM1 Timebase init
   * 
   * Prescaler: Clock prescaler. Set to 0 for maximum input clock
   * Counter Mode: Center aligned 2, counter counts up to TIM_Period-1 and down
   *  to zero. The Output compare interrupt flag of channels configured in output 
   *  is set when the counter counts up
   * Period: Auto reload register for the counter to wrap around
   * Clock Division: Divide by 1
   * Repetition Counter: Update event only if counter reaches zero (preload 
   *  registers are transfered and update interrupt generated)
   */
  tim_tbs.TIM_Prescaler = 0;
  tim_tbs.TIM_CounterMode = TIM_CounterMode_CenterAligned2;
  tim_tbs.TIM_Period = FOC_TIM_PERIOD;
  tim_tbs.TIM_ClockDivision = TIM_CKD_DIV1;
  tim_tbs.TIM_RepetitionCounter = 1;
  TIM_TimeBaseInit(TIM1, &tim_tbs);

  /**
   * TIM1 Output Compare channels configured as PWM
   * 
   * OCMode: PWM mode 1 - In upcounting, channel n is active as long as 
   *  TIMx_CNT<TIMx_CCRn else inactive. In downcounting, channel 1 is 
   *  inactive (OC1REF=‘0’) as long as TIMx_CNT>TIMx_CCRn else active (OC1REF=’1’).
   * TIM_OutputState: Enable the compare output
   * TIM_OutputNState: Enable the negative compare output
   * TIM_Pulse: Compare register value set to 50% duty
   * TIM_OCPolarity: output polarity active High
   * TIM_OCNPolarity: inverting output polarity active High
   * Idle state: output set in idle state
   */
  tim_ocis.TIM_OCMode = TIM_OCMode_PWM1;
  tim_ocis.TIM_OutputState = TIM_OutputState_Enable;
  tim_ocis.TIM_OutputNState = TIM_OutputNState_Enable;
  tim_ocis.TIM_Pulse = TIM1->ARR / 2;
  tim_ocis.TIM_OCPolarity = TIM_OCPolarity_High;
  tim_ocis.TIM_OCNPolarity = TIM_OCNPolarity_High;
  tim_ocis.TIM_OCIdleState = TIM_OCIdleState_Set;
  tim_ocis.TIM_OCNIdleState = TIM_OCNIdleState_Set;
  TIM_OC1Init(TIM1, &tim_ocis);
  TIM_OC2Init(TIM1, &tim_ocis);
  TIM_OC3Init(TIM1, &tim_ocis);
  tim_ocis.TIM_Pulse = FOC_TIM1_OC4_VALUE;
  TIM_OC4Init(TIM1, &tim_ocis); // used as ADC trigger
  
  /**
   * Preload register on TIMx_CCRn enabled. Read/Write operations access 
   * the preload register. TIMx_CCR1 preload value is loaded in the active 
   * register at each update event.
   */
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
  // TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
  
  /**
   * Configures the Break feature, dead time, Lock level, OSSI/OSSR State
   * and the AOE(automatic output enable).
   * 
   * TIM_OSSRState: Off-state selection for Run mode: outputs are enabled 
   *  with their inactive level as soon as CCxE=1 or CCxNE=1 (the output 
   *  is still controlled by the timer).
   * TIM_OSSIState: Off-state selection for Idle mode: OC/OCN outputs are 
   *  first forced with their inactive level then forced to their idle 
   *  level after the deadtime.
   * TIM_LOCKLevel: Diable any locking of config bits
   * TIM_DeadTime: Dead time cycles for the complementary outputs
   * TIM_Break: Disable break inputs
   * TIM_AutomaticOutput: Master output enable can only be set by SW
   */
  tim_bdtris.TIM_OSSRState = TIM_OSSRState_Enable;
  tim_bdtris.TIM_OSSIState = TIM_OSSIState_Enable;
  tim_bdtris.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
  tim_bdtris.TIM_DeadTime = FOC_PWM_DEADTIME_CYCLES;
  tim_bdtris.TIM_Break = TIM_Break_Disable;
  tim_bdtris.TIM_BreakPolarity = TIM_BreakPolarity_High;
  tim_bdtris.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
  TIM_BDTRConfig(TIM1, &tim_bdtris);
  /**
   * Enable preload register for latched transfer of the CC and AAR registers
   */
  TIM_CCPreloadControl(TIM1, ENABLE);
  TIM_ARRPreloadConfig(TIM1, ENABLE);

  /******* ADC *******/
 //  adc_cis.ADC_Mode
 //  adc_cis.ADC_Clock
 //  adc_cis.ADC_DMAAccessMode
 //  adc_cis.ADC_DMAModeingDelay


 //  ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;
 //  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
 //  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
 //  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
 //  ADC_CommonInit(&ADC_CommonInitStructure);

 // ADC_ExternalTrigInjecConv_T1_CC4

  /**
   * Enable timer
   */
  TIM_Cmd(TIM1, ENABLE);

  /**
   * Main output enable
   */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);

  /**
   * Create thread
   */
  (void)chThdCreateStatic(mcfWA, sizeof(mcfWA), HIGHPRIO, mcfocThread, NULL);
}



/**
 * @brief      Sets the duty time of the three PWM outputs in raw timer ticks
 *
 * @param[in]  a     Phase a duty
 * @param[in]  b     Phase b duty
 * @param[in]  c     Phase c duty
 */
void mcfSetDuty (uint16_t a, uint16_t b, uint16_t c)
{
  TIMER_UPDATE_DUTY(c,b,a);
  DBG("max duty: %d\r\n",FOC_TIM_PERIOD);
}

/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/
static THD_FUNCTION(mcfocThread, arg) {
  (void)arg;
  chRegSetThreadName(DEFS_THD_MCFOC_NAME);


  /**
   * @brief      SVM test
   *
   * @param[in]  <unnamed>  { parameter_description }
   */
  float t = 0;
  float freq = 100;
  float alpha, beta, d, q, theta;
  uint16_t da, db, dc;
  d = 0;
  q = 0.9;

  while (true) {
  palSetPad(GPIOE,14);
    theta = 2*PI*freq*(t/1000000); //800ns
  palTogglePad(GPIOE,14);
    invpark(&d, &q, &theta, &alpha, &beta); // 5.5us
  palTogglePad(GPIOE,14);
    svm(&alpha, &beta, &da, &db, &dc); // 4.3us
  palTogglePad(GPIOE,14);  
    TIMER_UPDATE_DUTY(da, db, dc);
    chThdSleepMicroseconds(FOC_THREAD_INTERVAL);
    t += FOC_THREAD_INTERVAL;
  }
}
/**
 * @brief      Calculates the duty cycles based on the input vectors in the
 * clark reference frame
 *
 * @param      a     in: clark alpha component
 * @param      b     in: clark beta component
 * @param      da    out: phase a duty cycle
 * @param      db    out: phase b duty cycle
 * @param      dc    out: phase c duty cycle
 */
static void svm (float* a, float* b, uint16_t* da, uint16_t* db, uint16_t* dc)
{
  uint8_t sector;
  float ton1, ton2;

  /**
   * First, decide which sector we're in
   */
  if(*b >= 0.0f)
  {
    // top half
    if(*a >= 0.0f)
    {
      // Quadrant I
      if( *a < (*b * ONE_BY_SQRT_3) )
        sector = 2;
      else
        sector = 1;
    }
    else
    {
      // Quadrant II
      if( ( -(*a) ) > (*b * ONE_BY_SQRT_3) )
        sector = 3;
      else
        sector = 2;
    }
  }
  else
  {
    // bottom half
    if(*a >= 0.0f)
    {
      // Quadrant IV
      if( *a < ( (-*b) * ONE_BY_SQRT_3) )
        sector = 5;
      else
        sector = 6;
    }
    else
    {
      // Quadrant III
      if( *a < (*b * ONE_BY_SQRT_3) )
        sector = 5;
      else
        sector = 4;
    }
  }

  /**
   * Now the sector dependant calculations
   */
  // Switch-case is approx. 0.1us faster than if statements
  switch(sector)
  {
    case 1:
      // Vector on times
      ton1 = (*a - ONE_BY_SQRT_3 * (*b) ) * FOC_TIM_PERIOD;
      ton2 = (TWO_BY_SQRT_3 * (*b)) * FOC_TIM_PERIOD;
      // PWM timings
      *da = (uint16_t)( (FOC_TIM_PERIOD - ton1 - ton2) / 2);
      *db = (uint16_t)(*da + ton1);
      *dc = (uint16_t)(*db + ton2);
      break;
    case 2:
      // Vector on times
      ton1 = ((*a) + ONE_BY_SQRT_3 * (*b)) * FOC_TIM_PERIOD;
      ton2 = (-(*a) + ONE_BY_SQRT_3 * (*b)) * FOC_TIM_PERIOD;
      // PWM timings
      *db = (uint16_t)((FOC_TIM_PERIOD - ton1 - ton2) / 2);
      *da = (uint16_t)((*db) + ton2);
      *dc = (uint16_t)((*da) + ton1);
      break;
    case 3:
      // Vector on times
      ton1 = (TWO_BY_SQRT_3 * (*b)) * FOC_TIM_PERIOD;
      ton2 = (-(*a) - ONE_BY_SQRT_3 * (*b)) * FOC_TIM_PERIOD;
      // PWM timings
      *db = (uint16_t)((FOC_TIM_PERIOD - ton1 - ton2) / 2);
      *dc = (uint16_t)((*db) + ton1);
      *da = (uint16_t)((*dc) + ton2);
      break;
    case 4:
      // Vector on times
      ton1 = (-(*a) + ONE_BY_SQRT_3 * (*b)) * FOC_TIM_PERIOD;
      ton2 = (-TWO_BY_SQRT_3 * (*b)) * FOC_TIM_PERIOD;
      // PWM timings
      *dc = (uint16_t)((FOC_TIM_PERIOD - ton1 - ton2) / 2);
      *db = (uint16_t)((*dc) + ton2);
      *da = (uint16_t)((*db) + ton1);
      break;
    case 5:
      // Vector on times
      ton1 = (-(*a) - ONE_BY_SQRT_3 * (*b)) * FOC_TIM_PERIOD;
      ton2 = ((*a) - ONE_BY_SQRT_3 * (*b)) * FOC_TIM_PERIOD;
      // PWM timings
      *dc = (uint16_t)((FOC_TIM_PERIOD - ton1 - ton2) / 2);
      *da = (uint16_t)(*dc + ton1);
      *db = (uint16_t)(*da + ton2);
      break;
    case 6:
      // Vector on times
      ton1 = (-TWO_BY_SQRT_3 * (*b)) * FOC_TIM_PERIOD;
      ton2 = ((*a) + ONE_BY_SQRT_3 * (*b)) * FOC_TIM_PERIOD;
      // PWM timings
      *da = (uint16_t)((FOC_TIM_PERIOD - ton1 - ton2) / 2);
      *dc = (uint16_t)(*da + ton2);
      *db = (uint16_t)(*dc + ton1);
      break;
  }
}

/**
 * @brief      Forward clark transformation
 *
 * @param      va    in: a component
 * @param      vb    in: b component
 * @param      vc    in: c component
 * @param      a     out: alpha component
 * @param      b     out: beta component
 */
static void clark (float* va, float* vb, float* vc, float* a, float* b)
{
  *va = 2.0f / 3.0f * (*va - 0.5f*(*vb) - 0.5f*(*vc));
  *vb = 2.0f / 3.0f * (SQRT_3_BY_2*(*vb) - SQRT_3_BY_2*(*vc));
}
/**
 * @brief      Forward park transformation
 *
 * @param      a     in: alpha component
 * @param      b     in: beta component
 * @param      theta in: angle
 * @param      d     out: d component
 * @param      q     out: q component
 */
static void park (float* a, float* b, float* theta, float* d, float* q )
{
  float sin, cos;
  sin = arm_sin_f32(*theta);
  cos = arm_cos_f32(*theta);
  (*d) =  (*a)*cos + (*b)*sin;
  (*q) = -(*a)*sin + (*b)*cos;
}
/**
 * @brief      Inverse clark transformation
 *
 * @param      a     in: alpha component
 * @param      b     in: beta component
 * @param      va    out: a component
 * @param      vb    out: b component
 * @param      vc    out: c component
 */
static void invclark (float* a, float* b, float* va, float* vb, float* vc)
{
  *va = *a;
  *vb = 1.0f / 2.0f * (-(*a) + SQRT_3*(*b));
  *vc = 1.0f / 2.0f * (-(*a) + SQRT_3*(*b));
}
/**
 * @brief      Inverse park transformation
 *
 * @param      d     in: d component
 * @param      q     in: q component
 * @param      theta in: angle
 * @param      a     out: alpha component
 * @param      b     out: beta component
 */
static void invpark (float* d, float* q, float* theta, float* a, float* b)
{
  float sin, cos;
  sin = arm_sin_f32(*theta);
  cos = arm_cos_f32(*theta);
  (*a) = (*d)*cos - (*q)*sin;
  (*b) = (*q)*cos + (*d)*sin;
}

/** @} */

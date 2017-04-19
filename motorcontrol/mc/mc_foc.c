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

#include "utelemetry.h"
#include "drv8301.h"

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

/**
 * Motor default parameters
 */
#define FOC_MOTOR_DEFAULT_PSI 0.003
#define FOC_MOTOR_DEFAULT_P   7
#define FOC_MOTOR_DEFAULT_LS  10e-6
#define FOC_MOTOR_DEFAULT_RS  15e-3
#define FOC_MOTOR_DEFAULT_J   150e-6

/**
 * FOC defautl parameters
 */
#define FOC_PARAM_DEFAULT_OBS_GAIN    100e6
#define FOC_PARAM_DEFAULT_CURR_D_KP   0.097
#define FOC_PARAM_DEFAULT_CURR_D_KI   0
#define FOC_PARAM_DEFAULT_CURR_D_KD   0
#define FOC_PARAM_DEFAULT_CURR_Q_KP   0.097
#define FOC_PARAM_DEFAULT_CURR_Q_KI   0
#define FOC_PARAM_DEFAULT_CURR_Q_KD   0
#define FOC_PARAM_DEFAULT_SPEED_KP    0.1
#define FOC_PARAM_DEFAULT_SPEED_KI    0
#define FOC_PARAM_DEFAULT_SPEED_KD    0
/**
 * @brief      Resistor divider for voltage measurements
 */
#define BOARD_ADC_PIN_TO_VOLT (float)((2.2f + 39.0f)/2.2f)
/**
 * @brief      Shunt resistor for current measurement
 */
#define BOARD_ADC_PIN_TO_AMP (float)(1000.0/DRV_CURRENT_GAIN)

#define ADC_CH_PH_A    2
#define ADC_CH_PH_B    1
#define ADC_CH_PH_C    0
#define ADC_CH_SUPPL   3
#define ADC_CH_CURR_A  5
#define ADC_CH_CURR_B  4
#define ADC_CH_TEMP    6
#define ADC_CH_REF     7

/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/
/*
 * Working area for the State machine thread
 */
static THD_WORKING_AREA(mcfMainWA, DEFS_THD_MCFOCMAIN_WA_SIZE);
static THD_FUNCTION(mcfocMainThread, arg);
static THD_WORKING_AREA(mcfSecondWA, DEFS_THD_MCFOCSECOND_WA_SIZE);
static THD_FUNCTION(mcfocSecondaryThread, arg);

static mcfMotorParameter_t mMotParms;
static mcfFOCParameter_t mFOCParms;

/**
 * @brief      Observer working set
 */
static mcfObs_t mObs;

static volatile uint16_t mADCValue[8]; // raw converted values
static float mADCtoPinFactor, mADCtoVoltsFactor, mADCtoAmpsFactor;

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
/**
 * @brief      Returns the ADC voltage on the pin
 */
#define ADC_PIN(ch) ( (float)mADCValue[ch] * mADCtoPinFactor)
/**
 * @brief      Returns the ADC voltage after the resistor divider
 */
#define ADC_VOLT(ch) ( (float)mADCValue[ch] * mADCtoVoltsFactor)
/**
 * @brief      Returns the current in the shunt resister
 */
#define ADC_AMP(ch) ( (float)(mADCValue[ch]-2047) * mADCtoAmpsFactor)
/**
 * @brief      Returns the current temperature
 */
#define ADC_TEMP(ch) ((((1.43 - ADC_PIN(ch) )) / 0.0043F) + 25.0f)

/*===========================================================================*/
/* prototypes                                                                */
/*===========================================================================*/
static void dataInit(void);
static void analogCalibrate(void);

static void svm (float* a, float* b, uint16_t* da, uint16_t* db, uint16_t* dc);

static void clark (float* va, float* vb, float* vc, float* a, float* b);
static void park (float* a, float* b, float* theta, float* d, float* q );
static void invclark (float* a, float* b, float* va, float* vb, float* vc);
static void invpark (float* d, float* q, float* theta, float* a, float* b);

static void runObserver(float *ua, float *ub, float *ia, float *ib, float *dt);

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
  dataInit();

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
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
  
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

  /******* DMA *******/  
  /**
   * DMA to transfer the regular channels to memory
   * 
   * DMA_PeripheralBaseAddr: ADC data register
   * DMA_MemoryBaseAddr: Memory address to start
   * DMA_DIR: Copy from peripheral to memory
   * DMA_BufferSize: number of data to transfer: 4 (one for each channel)
   * DMA_PeripheralInc: Dont increment the peripheral data adress
   * DMA_MemoryInc: Enable memory increment
   * Size: Each register is 16 bit = half word
   * Mode: Circular, start at the beginning after 4 transfers
   * Priority: Should be highest
   * M2M: Dont copy memory to memory
   */
  dma_is.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
  dma_is.DMA_MemoryBaseAddr = (uint32_t)(&mADCValue[0]);
  dma_is.DMA_DIR = DMA_DIR_PeripheralSRC;
  dma_is.DMA_BufferSize = 4;
  dma_is.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dma_is.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dma_is.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  dma_is.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  dma_is.DMA_Mode = DMA_Mode_Circular;
  dma_is.DMA_Priority = DMA_Priority_VeryHigh;
  dma_is.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &dma_is);
  /**
   * Change source and destination address for ADC3
   */
  dma_is.DMA_PeripheralBaseAddr = (uint32_t)&(ADC3->DR);
  dma_is.DMA_MemoryBaseAddr = (uint32_t)(&mADCValue[4]);
  DMA_Init(DMA2_Channel5, &dma_is);

  // Enable
  DMA_Cmd(DMA1_Channel1, ENABLE);
  DMA_Cmd(DMA2_Channel5, ENABLE);

  /******* ADC *******/  
  // Clock
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, ENABLE); 

  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div10); 
  RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div10); 

  // GPIOs (SOx pins are done in the drv8301 module)
  palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);

  /**
   * ADC_Mode: ADCs 1/2 and 3/4 are working independant
   * ADC_Clock: Synchronous clock mode divided by 2 HCLK/2
   * ADC_DMAAccessMode: MDMA mode enabled for 12 and 10-bit resolution
   * ADC_DMAMode: DMA circular event generation for continuous data streams
   * ADC_TwoSamplingDelay: Not used in independant mode
   */
  adc_cis.ADC_Mode = ADC_Mode_Independent;
  adc_cis.ADC_Clock = ADC_Clock_SynClkModeDiv2;
  adc_cis.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
  adc_cis.ADC_DMAMode = ADC_DMAMode_Circular;
  adc_cis.ADC_TwoSamplingDelay = 0x00;
  ADC_CommonInit(ADC1, &adc_cis);
  ADC_CommonInit(ADC3, &adc_cis);

  /**
   * ADC specific settings
   * 
   * ADC_ContinuousConvMode: Single conversion on trigger
   * ADC_Resolution: maximum resolution
   * ADC_ExternalTrigConvEvent: TIM1 TRGO event
   * ADC_ExternalTrigEventEdge: on falling edge of tim1 channel4
   * ADC_DataAlign: ADC_DataAlign_Right
   * ADC_OverrunMode: on overrun keep the valid data
   * ADC_AutoInjMode: dont do injected channels after regular channels
   * ADC_NbrOfRegChannel: 4 channels on ADC 1 (Phase A B C, Supply) and 2 on ADC3 (currents)
   */
  adc_is.ADC_ContinuousConvMode = DISABLE;
  adc_is.ADC_Resolution = ADC_Resolution_12b;
  adc_is.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_9; // 9 for trigo
  adc_is.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_FallingEdge;
  // adc_is.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  adc_is.ADC_DataAlign = ADC_DataAlign_Right;
  adc_is.ADC_OverrunMode = DISABLE;
  adc_is.ADC_AutoInjMode = DISABLE;
  adc_is.ADC_NbrOfRegChannel = 4;
  ADC_Init(ADC1, &adc_is);
  ADC_Init(ADC3, &adc_is);

  // Enable voltage regulator
  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  ADC_VoltageRegulatorCmd(ADC3, ENABLE);
  ADC_TempSensorCmd(ADC1, ENABLE);
  chThdSleepMicroseconds(20);

  // Enable Vrefint channel
  ADC_VrefintCmd(ADC1, ENABLE);
  ADC_VrefintCmd(ADC3, ENABLE);

  //calibrate
  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC1);
  ADC_SelectCalibrationMode(ADC3, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC3);
  while(ADC_GetCalibrationStatus(ADC1) == SET);
  while(ADC_GetCalibrationStatus(ADC3) == SET);

  /**
   * Configure the regular channels
   * 
   * ADC1: 
   * - 1: IN1 Phase C
   * - 2: IN2 Phase B
   * - 3: IN3 Phase A
   * - 4: IN9 Supply
   */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_1Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_1Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_1Cycles5);
  ADC_RegularChannelSequencerLengthConfig(ADC1, 4);
  /** 
   * ADC3:
   * - 1: IN12 SOB
   * - 2: IN1  SOA
   * - 3: Temperatur
   * - 4: VrefInt
   */
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_1Cycles5);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 2, ADC_SampleTime_1Cycles5);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_TempSensor, 3, ADC_SampleTime_1Cycles5);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_Vrefint, 4, ADC_SampleTime_1Cycles5);
  ADC_RegularChannelSequencerLengthConfig(ADC3, 4);

  /**
   * Enable ADC
   */
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
  nvicEnableVector(ADC1_2_IRQn, 4);
  // nvicEnableVector(ADC3_IRQn, 4);

  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC3, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);
  ADC_DMACmd(ADC3, ENABLE);
  ADC_StartConversion(ADC1);
  ADC_StartConversion(ADC3);

  /**
   * Enable timer
   */
  TIM_Cmd(TIM1, ENABLE);

  /**
   * Main output enable
   */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);

  // Trigger for ADC
  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC4Ref);
  
  analogCalibrate();

  /**
   * Create thread
   */
  (void)chThdCreateStatic(mcfMainWA, sizeof(mcfMainWA), HIGHPRIO, mcfocMainThread, NULL);
  (void)chThdCreateStatic(mcfSecondWA, sizeof(mcfSecondWA), NORMALPRIO, mcfocSecondaryThread, NULL);
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

/**
 * @brief      Dumps the local data to the debug stream
 */
void mcfDumpData(void)
{
  DBG2("\r\n--- ADC ---\r\n");
  DBG2("          ph_C |     ph_B |     ph_A |     v_in |    cur_b |    cur_a |     temp |     vref\r\n");
  DBG2("raw       %04d |     %04d |     %04d |     %04d |     %04d |     %04d |     %04d |     %04d\r\n", mADCValue[0], mADCValue[1],
    mADCValue[2], mADCValue[3], mADCValue[4], mADCValue[5], mADCValue[6], mADCValue[7]);
  DBG2("pin    %7.3f |  %7.3f |  %7.3f |  %7.3f |  %7.3f |  %7.3f |  %7.3f |  %7.3f\r\n", ADC_PIN(0), ADC_PIN(1),
    ADC_PIN(2), ADC_PIN(3), ADC_PIN(4), ADC_PIN(5), ADC_PIN(6), ADC_PIN(7));
  DBG2("SI     %7.3f |  %7.3f |  %7.3f |  %7.3f |  %7.3f |  %7.3f |  %7.3f\r\n", ADC_VOLT(0), ADC_VOLT(1),
    ADC_VOLT(2), ADC_VOLT(3), ADC_AMP(4), ADC_AMP(5), ADC_TEMP(6));

  ADC_StartConversion(ADC1);
  ADC_StartConversion(ADC3);
}

/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/
static THD_FUNCTION(mcfocMainThread, arg) {
  (void)arg;
  chRegSetThreadName(DEFS_THD_MCFOC_MAIN_NAME);

  // while(true)
  // {
  //   // transform
    
  //   // run speed controller
    
  //   // run current controller
    
  //   // inverse transform
    
  //   // calculate duties
    
  //   // set output
  // }

  /**
   * @brief      SVM test
   *
   * @param[in]  <unnamed>  { parameter_description }
   */
  float t = 0;
  float freq = 50;
  float alpha, beta, d, q, theta;
  uint16_t da, db, dc;
  d = 0;
  q = 0.5;

  while (true) {
    theta = 2*PI*freq*(t/1000000); //800ns
    invpark(&d, &q, &theta, &alpha, &beta); // 5.5us
    svm(&alpha, &beta, &da, &db, &dc); // 4.3us
    TIMER_UPDATE_DUTY(da, db, dc);
    chThdSleepMicroseconds(FOC_THREAD_INTERVAL);
    t += FOC_THREAD_INTERVAL;
    freq += 0.002;
  }
}
static THD_FUNCTION(mcfocSecondaryThread, arg)
{
  (void)arg;
  chRegSetThreadName(DEFS_THD_MCFOC_SECOND_NAME);
  static float ph_a, ph_b, ph_c, suppl, curr_a, curr_b;
  static uint64_t x = 0;
  // utlmEnable(true);
  while(true)
  {
    ph_a = ADC_VOLT(ADC_CH_PH_A);
    ph_b = ADC_VOLT(ADC_CH_PH_B);
    ph_c = ADC_VOLT(ADC_CH_PH_C);
    suppl = ADC_VOLT(ADC_CH_SUPPL);
    curr_a = ADC_VOLT(ADC_CH_CURR_A);
    curr_b = ADC_VOLT(ADC_CH_CURR_B);
    // utlmSend(0, 1, &x, &ph_a);
    // utlmSend(1, 1, &x, &ph_b);
    // utlmSend(2, 1, &x, &ph_c);
    // utlmSend(3, 1, &x, &suppl);
    // utlmSend(4, 1, &x, &curr_a);
    // utlmSend(5, 1, &x, &curr_b);
    DBG3("%d %.3f %.3f %.3f %.3f %.3f %.3f ",
      x, ph_a, ph_b, ph_c, suppl, curr_a, curr_b);
    DBG3("\r\n");
    x+=10;
    chThdSleepMilliseconds(10);
  }
}
/**
 * @brief      Initializes the static data with default values
 */
static void dataInit(void)
{
  mMotParms.psi = FOC_MOTOR_DEFAULT_PSI;
  mMotParms.p = FOC_MOTOR_DEFAULT_P;
  mMotParms.Ls = FOC_MOTOR_DEFAULT_LS;
  mMotParms.Rs = FOC_MOTOR_DEFAULT_RS;
  mMotParms.J = FOC_MOTOR_DEFAULT_J;

  mFOCParms.obsGain = FOC_PARAM_DEFAULT_OBS_GAIN;
  mFOCParms.curr_d_kp = FOC_PARAM_DEFAULT_CURR_D_KP;
  mFOCParms.curr_d_ki = FOC_PARAM_DEFAULT_CURR_D_KI;
  mFOCParms.curr_d_kd = FOC_PARAM_DEFAULT_CURR_D_KD;
  mFOCParms.curr_q_kp = FOC_PARAM_DEFAULT_CURR_Q_KP;
  mFOCParms.curr_q_ki = FOC_PARAM_DEFAULT_CURR_Q_KI;
  mFOCParms.curr_q_kd = FOC_PARAM_DEFAULT_CURR_Q_KD;
  mFOCParms.speed_kp = FOC_PARAM_DEFAULT_SPEED_KP;
  mFOCParms.speed_ki = FOC_PARAM_DEFAULT_SPEED_KI;
  mFOCParms.speed_kd = FOC_PARAM_DEFAULT_SPEED_KD;
}
/**
 * @brief      Calibrates all analog signals
 */
static void analogCalibrate(void)
{
  const uint8_t* vrefint_cal_lsb = (uint8_t*)0x1FFFF7BA;
  const uint8_t* vrefint_cal_msb = (uint8_t*)0x1FFFF7BB;
  const uint16_t vrefint_cal = ((*vrefint_cal_msb)<<8) | (*vrefint_cal_lsb);

  // wait for data in vref input
  while(mADCValue[7] == 0);

  // reference man, page 375
  // mADCtoPinFactor = 3.3f * (float)vrefint_cal / ((float)mADCValue[7]*4095.0f);
  mADCtoPinFactor = 3.3f / 4095.0f;
  mADCtoVoltsFactor = mADCtoPinFactor * BOARD_ADC_PIN_TO_VOLT;
  mADCtoAmpsFactor =  mADCtoPinFactor * BOARD_ADC_PIN_TO_AMP;

  DBG2("vrefint_cal=%d\r\n",vrefint_cal);
  DBG2("mADCtoPinFactor=%f\r\n", mADCtoPinFactor);
  DBG2("mADCtoVoltsFactor=%f\r\n", mADCtoVoltsFactor);
  DBG2("mADCtoAmpsFactor=%f\r\n", mADCtoAmpsFactor);
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
/**
 * @brief      Run a non linear observer iteration
 * @note       Based on IEEE 2010 Position Estimator using a Nonlinear Observer
 *
 * @param      ua    in: measured v alpha
 * @param      ub    in: measured v beta
 * @param      ia    in: measured i alpha
 * @param      ib    in: measured i beta
 * @param      dt    in: time delta since last call
 */
static void runObserver(float *ua, float *ub, float *ia, float *ib, float *dt)
{
  float pos_error;
  float xp[2];

  mObs.eta[0] = mObs.x[0] - mMotParms.Ls*(*ia);
  mObs.eta[1] = mObs.x[1] - mMotParms.Ls*(*ib);

  mObs.y[0] = -mMotParms.Rs*(*ia) + (*ua);
  mObs.y[1] = -mMotParms.Rs*(*ib) + (*ub);

  pos_error =  mMotParms.psi*mMotParms.psi - 
    (mObs.eta[0]*mObs.eta[0] + mObs.eta[1]*mObs.eta[1]);

  xp[0] = mObs.y[0] + 0.5f*mFOCParms.obsGain*mObs.eta[0]*pos_error;
  xp[1] = mObs.y[1] + 0.5f*mFOCParms.obsGain*mObs.eta[1]*pos_error;
        
  mObs.x[0] += xp[0]*(*dt);
  mObs.x[1] += xp[1]*(*dt);

  mObs.theta = utilFastAtan2((mObs.x[1] - mMotParms.Ls*(*ib)), 
    (mObs.x[0] - mMotParms.Ls*(*ia) ));
}

/**
 * @brief      ADC1_2 IRQ handler
 */
CH_IRQ_HANDLER(Vector88) {
  CH_IRQ_PROLOGUE();
  palTogglePad(GPIOE,14);
  ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  // mc_interface_adc_inj_int_handler();
  CH_IRQ_EPILOGUE();
}

/**
 * @brief      ADC IRQ handler
 */
CH_IRQ_HANDLER(Vector47) {
  CH_IRQ_PROLOGUE();
  // ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
  // mc_interface_adc_inj_int_handler();
  CH_IRQ_EPILOGUE();
}

/** @} */

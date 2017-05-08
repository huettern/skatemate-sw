/*
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
#include "chbsem.h"

#include "defs.h"
#include "util.h"

#include "arm_math.h"

#include "stm32f30x_conf.h"

#include "utelemetry.h"
#include "drv8301.h"

/*===========================================================================*/
/* DEBUG                                                                     */
/*===========================================================================*/
// #define DEBUG_ADC
// #define DEBUG_SVM
//#define DEBUG_OBSERVER
#define DEBUG_CONTROLLERS

#define DEBUG_DOWNSAMPLE_FACTOR 10

/*===========================================================================*/
/* IMPLEMENTATION SETTINGS                                                   */
/*===========================================================================*/
// If defined, use the CMSIS implementation of the clark and park transform
// #define USE_CMSIS_CLARK_PARK

/*===========================================================================*/
/* settings                                                                  */
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
 * How fast the control thread should run
 */
#define FOC_THREAD_INTERVAL 100 // us
/**
 * How much slower the current control loop should run
 */
#define FOC_CURRENT_CONTROLLER_SLOWDOWN 10
/**
 * Forced commutation settings
 */
#define FOC_FORCED_COMM_FREQ -30.0
#define FOC_FORCED_COMM_VD 0.0
#define FOC_FORCED_COMM_VQ 0.07

/**
 * Motor default parameters
 */
#define FOC_MOTOR_DEFAULT_PSI 15.0e-3 //0.008
#define FOC_MOTOR_DEFAULT_P   7
#define FOC_MOTOR_DEFAULT_LS  35.0e-6 //18.0e-6 // Lumenier: 12e-6
#define FOC_MOTOR_DEFAULT_RS  20*0.04 // 1.2 // Lumenier: 0.06
#define FOC_MOTOR_DEFAULT_J   150e-6 // not used

/**
 * FOC defautl parameters
 */
#define FOC_PARAM_DEFAULT_OBS_GAIN    100e6
#define FOC_PARAM_DEFAULT_OBS_SPEED_KP  200.0  
#define FOC_PARAM_DEFAULT_OBS_SPEED_KI  20000.0
#define FOC_PARAM_DEFAULT_OBS_SPEED_KD  0.0
#define FOC_PARAM_DEFAULT_OBS_SPEED_CEIL     6.0
#define FOC_PARAM_DEFAULT_OBS_SPEED_FLOOR    -6.0

#define FOC_PARAM_DEFAULT_CURR_D_KP   0.03
#define FOC_PARAM_DEFAULT_CURR_D_KI   0.01
#define FOC_PARAM_DEFAULT_CURR_D_KD   0.0
#define FOC_PARAM_DEFAULT_CURR_D_I_CEIL     1.0
#define FOC_PARAM_DEFAULT_CURR_D_I_FLOOR    -1.0

#define FOC_PARAM_DEFAULT_CURR_Q_KP   0.03
#define FOC_PARAM_DEFAULT_CURR_Q_KI   0.01
#define FOC_PARAM_DEFAULT_CURR_Q_KD   0.0
#define FOC_PARAM_DEFAULT_CURR_Q_I_CEIL     1.0
#define FOC_PARAM_DEFAULT_CURR_Q_I_FLOOR    -1.0

#define FOC_PARAM_DEFAULT_SPEED_KP    0.1
#define FOC_PARAM_DEFAULT_SPEED_KI    0.0
#define FOC_PARAM_DEFAULT_SPEED_KD    0.0

#define FOC_LP_FAST_CONSTANT 0.1
/**
 * @brief      Resistor divider for voltage measurements
 */
// #define BOARD_ADC_PIN_TO_VOLT (float)((2.2f + 39.0f)/2.2f) // using resistor values
#define BOARD_ADC_PIN_TO_VOLT (17.1183) // using measured ratio
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
/* private datatypes                                                         */
/*===========================================================================*/
typedef struct {
  float kp;
  float ki;
  float istate;
  float iceil;
  float ifloor;
} piStruct_t;


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
static mcfController_t mCtrl;

/**
 * @brief      Observer working set
 */
static mcfObs_t mObs;

static volatile uint16_t mADCValue[8]; // raw converted values

static float mADCtoPinFactor, mADCtoVoltsFactor, mADCtoAmpsFactor;
static uint16_t mDrvOffA, mDrvOffB; 

static BSEMAPHORE_DECL(mIstSem, TRUE);

/**
 * Debug data
 */
#ifdef DEBUG_ADC
  #define ADC_STORE_DEPTH 500
#else
  #define ADC_STORE_DEPTH 1
#endif
static volatile uint16_t mADCValueStore[ADC_STORE_DEPTH][8]; // raw converted values
static volatile uint8_t mStoreADC1, mStoreADC3;
#ifdef DEBUG_OBSERVER
  #define OBS_STORE_DEPTH 1000
#else
  #define OBS_STORE_DEPTH 1
#endif
static volatile float mOBSValueStore[OBS_STORE_DEPTH][6];
static volatile uint8_t mStoreObserver;
static uint16_t mObsDebugCounter;
#ifdef DEBUG_CONTROLLERS
  #define CONT_STORE_DEPTH 800
#else
  #define CONT_STORE_DEPTH 1
#endif
static volatile float mContValueStore[CONT_STORE_DEPTH][8];
static volatile uint8_t mStoreController;
static uint16_t mControllerDebugCtr;

static uint8_t mForcedCommutationMode = 1;

static piStruct_t mpiId;
static piStruct_t mpiIq;
static piStruct_t mpiSpeed;

static mcState_t mState = MC_HALT;

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
#define FOC_TIM_PERIOD SYSTEM_CORE_CLOCK / FOC_F_SW / 2
/**
 * Duty cycle for TIM1 Channel 4 set to maximum to get a falling edge in the
 * middle of a Gate low output sequence for current and voltage sensing
 */
#define FOC_TIM1_OC4_VALUE FOC_TIM_PERIOD - 1
/**
 * @brief      Returns the ADC voltage on the pin
 */
#define ADC_PIN(ch) ( (float)mADCValue[ch] * mADCtoPinFactor)
/**
 * @brief      Returns the ADC voltage after the resistor divider
 */
#define ADC_VOLT(ch) ( (float)mADCValue[ch] * mADCtoVoltsFactor)
#define ADC_STORE_VOLT(i, ch) ( (float)mADCValueStore[i][ch] * mADCtoVoltsFactor)
/**
 * @brief      Returns the current in the shunt resister
 * @note       TODO: Remove minus for new revision!!!
 * @note       TODO: Removed the minus again on old hardware but why???
 */
#define ADC_CURR_A() ( ((float)mADCValue[ADC_CH_CURR_A]-mDrvOffA) * mADCtoAmpsFactor )
// TODO: This 2.2 is a measured difference between the 2 currnet sense outputs from the drv
#define ADC_CURR_B() ( ((float)mADCValue[ADC_CH_CURR_B]-mDrvOffB) * mADCtoAmpsFactor / 2.2) 
#define ADC_STORE_CURR_A(i) ( ((float)mADCValueStore[i][ADC_CH_CURR_A]-mDrvOffA) * -mADCtoAmpsFactor )
#define ADC_STORE_CURR_B(i) ( ((float)mADCValueStore[i][ADC_CH_CURR_B]-mDrvOffB) * -mADCtoAmpsFactor )
/**
 * @brief      Returns the current temperature
 */
#define ADC_TEMP(ch) ((((1.43 - ADC_PIN(ch) )) / 0.0043F) + 25.0f)

/*===========================================================================*/
/* prototypes                                                                */
/*===========================================================================*/
static void dataInit(void);
static void analogCalibrate(void);
static void drvDCCal(void);

static void svm (float* a, float* b, uint16_t* da, uint16_t* db, uint16_t* dc);

static void clark (float* va, float* vb, float* vc, float* a, float* b);
static void park (float* a, float* b, float* theta, float* d, float* q );
static void invclark (float* a, float* b, float* va, float* vb, float* vc);
static void invpark (float* d, float* q, float* theta, float* a, float* b);
static float piController(piStruct_t* s, float sample, float* dt);

static void runPositionObserver(float *dt);
static void runSpeedObserver (float *dt);
static void runSpeedController (void);
static void runCurrentController (float *dt);
static void runOutputs(void);
static void runOutputsWithoutObserver(float theta);
static void forcedCommutation (void);

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
   * Clock source is PCLK2 = APB2_CLOCK
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
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
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
  // ADC clock source is directly from the PLL output
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1); 
  RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div1); 
  // Clock
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, ENABLE); 

  // GPIOs (SOx pins are done in the drv8301 module)
  palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);

  /**
   * ADC_Mode: ADCs 1/2 and 3/4 are working independant
   * ADC_Clock: Clock source is PLL output
   * ADC_DMAAccessMode: MDMA mode enabled for 12 and 10-bit resolution
   * ADC_DMAMode: DMA circular event generation for continuous data streams
   * ADC_TwoSamplingDelay: Not used in independant mode
   */
  adc_cis.ADC_Mode = ADC_Mode_Independent;
  adc_cis.ADC_Clock = ADC_Clock_AsynClkMode;
  adc_cis.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
  adc_cis.ADC_DMAMode = ADC_DMAMode_Circular;
  adc_cis.ADC_TwoSamplingDelay = 0x00;
  ADC_CommonInit(ADC1, &adc_cis);
  ADC_CommonInit(ADC3, &adc_cis);

  /**
   * ADC specific settings
   * 
   * ADC_ContinuousConvMode: continuous conversion on trigger
   * ADC_Resolution: maximum resolution
   * ADC_ExternalTrigConvEvent: TIM1 TRGO event
   * ADC_ExternalTrigEventEdge: on falling edge of tim1 channel4
   * ADC_DataAlign: ADC_DataAlign_Right
   * ADC_OverrunMode: on overrun keep the valid data
   * ADC_AutoInjMode: dont do injected channels after regular channels
   * ADC_NbrOfRegChannel: 4 channels on ADC 1 (Phase A B C, Supply) and 2 on ADC3 (currents)
   */
  adc_is.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
  adc_is.ADC_Resolution = ADC_Resolution_12b;
  adc_is.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_9; // 9 for trgo
  adc_is.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_FallingEdge;
  // adc_is.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  adc_is.ADC_DataAlign = ADC_DataAlign_Right;
  adc_is.ADC_OverrunMode = DISABLE;
  adc_is.ADC_AutoInjMode = DISABLE;
  adc_is.ADC_NbrOfRegChannel = 4;
  ADC_Init(ADC1, &adc_is);
  ADC_Init(ADC3, &adc_is);
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC3, ENABLE);

  // Enable voltage regulator
  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  ADC_VoltageRegulatorCmd(ADC3, ENABLE);
  ADC_TempSensorCmd(ADC1, ENABLE);
  chThdSleepMicroseconds(20);

  // Enable Vrefint channel
  ADC_VrefintCmd(ADC1, ENABLE);
  ADC_VrefintCmd(ADC3, ENABLE);


  /**
   * Configure the regular channels
   * 
   * ADC1: 
   * - 1: IN1 Phase C
   * - 2: IN2 Phase B
   * - 3: IN3 Phase A
   * - 4: IN9 Supply
   */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_61Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_61Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_61Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_61Cycles5);
  ADC_RegularChannelSequencerLengthConfig(ADC1, 4);
  /** 
   * ADC3:
   * - 1: IN12 SOB
   * - 2: IN1  SOA
   * - 3: Temperatur
   * - 4: VrefInt
   */
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_61Cycles5);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 2, ADC_SampleTime_61Cycles5);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_TempSensor, 3, ADC_SampleTime_61Cycles5);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_Vrefint, 4, ADC_SampleTime_61Cycles5);
  ADC_RegularChannelSequencerLengthConfig(ADC3, 4);

  //calibrate
  // ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  // ADC_StartCalibration(ADC1);
  // while(ADC_GetCalibrationStatus(ADC1) == SET);
  // ADC_SelectCalibrationMode(ADC3, ADC_CalibrationMode_Single);
  // ADC_StartCalibration(ADC3);
  // while(ADC_GetCalibrationStatus(ADC3) == SET);
  
  /**
   * Enable ADC
   */
  ADC_ITConfig(ADC1, ADC_IT_EOS, ENABLE);
  ADC_ITConfig(ADC3, ADC_IT_EOS, ENABLE);
  // nvicEnableVector(ADC1_2_IRQn, 8);
  nvicEnableVector(ADC3_IRQn, 2); // Higher prio to current sampling

  // ADC_ExternalTriggerConfig(ADC1, ADC_ExternalTrigConvEvent_9, ADC_ExternalTrigEventEdge_FallingEdge);
  // ADC_ExternalTriggerConfig(ADC3, ADC_ExternalTrigConvEvent_9, ADC_ExternalTrigEventEdge_FallingEdge);
  //start
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
  drvDCCal();

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
    ADC_VOLT(2), ADC_VOLT(3), ADC_CURR_A(), ADC_CURR_B(), ADC_TEMP(6));

  // ADC_StartConversion(ADC1);
  // ADC_StartConversion(ADC3);
}

/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/
static THD_FUNCTION(mcfocMainThread, arg) {
  (void)arg;
  chRegSetThreadName(DEFS_THD_MCFOC_MAIN_NAME);
  // uint16_t dutya, dutyb, dutyc;

  // while(true)
  // {
  //   // transform
  //   park(&mCtrl.ia_is, &mCtrl.ib_is, &mObs.theta, &mCtrl.id_is, &mCtrl.iq_is);
  //   // run speed controller
  //   // runSpeedController();
  //   mCtrl.id_set = 0.0;
  //   mCtrl.iq_set = 0.01;
  //   // run current controller
  //   runCurrentController();
  //   // inverse transform
  //   invpark(&mCtrl.vd_set, &mCtrl.vq_set, &mObs.theta, &mCtrl.va_set, &mCtrl.vb_set);
  //   // calculate duties
  //   utils_saturate_vector_2d(&mCtrl.va_set, &mCtrl.vb_set, SQRT_3_BY_2);
  //   svm(&mCtrl.va_set, &mCtrl.vb_set, &dutya, &dutyb, &dutyc);
  //   // set output
  //   TIMER_UPDATE_DUTY(dutyc, dutyb, dutya);
  //   // delay
  //   chThdSleepMicroseconds(FOC_THREAD_INTERVAL);
  // }

  /**
   * @brief      SVM test
   *
   * @param[in]  <unnamed>  { parameter_description }
   */
  mCtrl.vd_set = 0;
  mCtrl.vq_set = 0.05;
  // TIMER_UPDATE_DUTY(1500, 0, 0);
    mState = MC_OPEN_LOOP;
    chThdSleepMilliseconds(3000);
    mState = MC_CLOSED_LOOP;
  while (true) {
    chThdSleepMilliseconds(3000);
  }
}
static THD_FUNCTION(mcfocSecondaryThread, arg)
{
  (void)arg;
  chRegSetThreadName(DEFS_THD_MCFOC_SECOND_NAME);
  static float ph_a, ph_b, ph_c, suppl, curr_a, curr_b, ref;
  uint16_t i;
  // utlmEnable(true);
  
  while(true)
  {
    #ifdef DEBUG_ADC
      chThdSleepMilliseconds(5000);
      mStoreADC1 = 1;
      mStoreADC3 = 1;
      chThdSleepMilliseconds(1);
      while(mStoreADC1 | mStoreADC3) chThdSleepMilliseconds(1);
      for(i = 0; i < ADC_STORE_DEPTH; i++)
      {
        ph_a = ADC_STORE_VOLT(i, ADC_CH_PH_A);
        ph_b = ADC_STORE_VOLT(i, ADC_CH_PH_B);
        ph_c = ADC_STORE_VOLT(i, ADC_CH_PH_C);
        suppl = ADC_STORE_VOLT(i, ADC_CH_SUPPL);
        curr_a = ADC_STORE_CURR_A(i);
        curr_b = ADC_STORE_CURR_B(i);
        ref = ADC_STORE_VOLT(i, ADC_CH_REF);

        DBG3("%.3f %.3f %.3f %.3f %.3f %.3f %.3f\r\n", ph_a, ph_b, ph_c, suppl, curr_a, curr_b, ref);
        // DBG3("%d %d %d %d %d %d %d\r\n", mADCValueStore[i][ADC_CH_PH_A], mADCValueStore[i][ADC_CH_PH_B], mADCValueStore[i][ADC_CH_PH_C], 
        //   mADCValueStore[i][ADC_CH_SUPPL], mADCValueStore[i][ADC_CH_CURR_A], mADCValueStore[i][ADC_CH_CURR_B], mADCValueStore[i][ADC_CH_REF]);
      }  
    #endif
    #ifdef DEBUG_SVM
      // svm debug
      DBG3("%.3f %.3f %d %d %d\r\n", mCtrl.va_set, mCtrl.vb_set, 
        TIM1->CCR2, TIM1->CCR2, TIM1->CCR1);
    #endif
    #ifdef DEBUG_OBSERVER
      chThdSleepMilliseconds(5000);
      mStoreObserver = 1;
      chThdSleepMilliseconds(1);
      while(mStoreObserver) chThdSleepMilliseconds(1);
      for(i = 0; i < OBS_STORE_DEPTH; i++)
      {
        DBG3("%.3f %.3f %.3f %.3f %.3f %.3f\r\n", mOBSValueStore[i][0], mOBSValueStore[i][1], 
          mOBSValueStore[i][2], mOBSValueStore[i][3], mOBSValueStore[i][4], mOBSValueStore[i][5]);
      } 
    // observer debug
      // DBG3("%.3f %.3f %.3f %.3f\r\n", mObs.omega_e, mObs.theta, mCtrl.va_set, mCtrl.vb_set);
    #endif
    #ifdef DEBUG_CONTROLLERS
      chThdSleepMilliseconds(3000);
      mStoreController = 1;
      chThdSleepMilliseconds(1);
      while(mStoreController) chThdSleepMilliseconds(1);
      for(i = 0; i < CONT_STORE_DEPTH; i++)
      {
        DBG3("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\r\n", mContValueStore[i][0], mContValueStore[i][1], 
          mContValueStore[i][2], mContValueStore[i][3], mContValueStore[i][4], mContValueStore[i][5], 
          mContValueStore[i][6], mContValueStore[i][7]);
      } 
    #endif
    chThdSleepMilliseconds(1);
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
  mFOCParms.obsSpeed_kp = FOC_PARAM_DEFAULT_OBS_SPEED_KP;
  mFOCParms.obsSpeed_ki = FOC_PARAM_DEFAULT_OBS_SPEED_KI;
  mFOCParms.obsSpeed_kd = FOC_PARAM_DEFAULT_OBS_SPEED_KD;
  mFOCParms.curr_d_kp = FOC_PARAM_DEFAULT_CURR_D_KP;
  mFOCParms.curr_d_ki = FOC_PARAM_DEFAULT_CURR_D_KI;
  mFOCParms.curr_d_kd = FOC_PARAM_DEFAULT_CURR_D_KD;
  mFOCParms.curr_q_kp = FOC_PARAM_DEFAULT_CURR_Q_KP;
  mFOCParms.curr_q_ki = FOC_PARAM_DEFAULT_CURR_Q_KI;
  mFOCParms.curr_q_kd = FOC_PARAM_DEFAULT_CURR_Q_KD;
  mFOCParms.speed_kp = FOC_PARAM_DEFAULT_SPEED_KP;
  mFOCParms.speed_ki = FOC_PARAM_DEFAULT_SPEED_KI;
  mFOCParms.speed_kd = FOC_PARAM_DEFAULT_SPEED_KD;

  mObs.x[0] = 0.0; mObs.x[1] = 0.0;
  mObs.eta[0] = 0.0; mObs.eta[1] = 0.0;
  mObs.y[0] = 0.0; mObs.y[1] = 0.0; 
  mObs.theta = 0.0;
  mObs.theta_var = 0.0;
  mObs.omega_m = 0.0;
  mObs.omega_e = 0.0;

  mCtrl.w_set = 200.0;
  mCtrl.w_is = 0.0;
  mCtrl.id_set = 0.0;
  mCtrl.id_is = 0.0;
  mCtrl.iq_set = 0.0;
  mCtrl.iq_is = 0.0;
  mCtrl.vd_set = 0.0;
  mCtrl.vq_set = 0.0;

  // PID controllers
  mObs.speedPID.Kp = mFOCParms.obsSpeed_kp;
  mObs.speedPID.Ki = mFOCParms.obsSpeed_ki;
  mObs.speedPID.Kd = mFOCParms.obsSpeed_kd;
  arm_pid_init_f32(&mObs.speedPID, true);
  mCtrl.speedPID.Kp = mFOCParms.speed_kp;
  mCtrl.speedPID.Ki = mFOCParms.speed_ki;
  mCtrl.speedPID.Kd = mFOCParms.speed_kd;
  arm_pid_init_f32(&mCtrl.speedPID, true);
  mCtrl.idPID.Kp = mFOCParms.curr_d_kp;
  mCtrl.idPID.Ki = mFOCParms.curr_d_ki;
  mCtrl.idPID.Kd = mFOCParms.curr_d_kd;
  arm_pid_init_f32(&mCtrl.idPID, true);
  mCtrl.iqPID.Kp = mFOCParms.curr_q_kp;
  mCtrl.iqPID.Ki = mFOCParms.curr_q_ki;
  mCtrl.iqPID.Kd = mFOCParms.curr_q_kd;
  arm_pid_init_f32(&mCtrl.iqPID, true);

  mpiId.kp = FOC_PARAM_DEFAULT_CURR_D_KP;
  mpiId.ki = FOC_PARAM_DEFAULT_CURR_D_KI;
  mpiId.istate = 0;
  mpiId.iceil = FOC_PARAM_DEFAULT_CURR_D_I_CEIL;
  mpiId.ifloor = FOC_PARAM_DEFAULT_CURR_D_I_FLOOR;

  mpiIq.kp = FOC_PARAM_DEFAULT_CURR_Q_KP;
  mpiIq.ki = FOC_PARAM_DEFAULT_CURR_Q_KI;
  mpiIq.istate = 0;
  mpiIq.iceil = FOC_PARAM_DEFAULT_CURR_Q_I_CEIL;
  mpiIq.ifloor = FOC_PARAM_DEFAULT_CURR_Q_I_FLOOR;

  mpiSpeed.kp = mFOCParms.obsSpeed_kp;
  mpiSpeed.ki = mFOCParms.obsSpeed_ki;
  mpiSpeed.istate = 0;
  mpiSpeed.iceil = FOC_PARAM_DEFAULT_OBS_SPEED_CEIL;
  mpiSpeed.ifloor = FOC_PARAM_DEFAULT_OBS_SPEED_FLOOR;
}
/**
 * @brief      Calibrates all analog signals
 */
static void analogCalibrate(void)
{
  uint16_t ctr = 0;
  float buf;
  const uint8_t* vrefint_cal_lsb = (uint8_t*)0x1FFFF7BA;
  const uint8_t* vrefint_cal_msb = (uint8_t*)0x1FFFF7BB;
  const uint16_t vrefint_cal = ((*vrefint_cal_msb)<<8) | (*vrefint_cal_lsb);

  // wait for data in vref input
  while(mADCValue[7] == 0);
  buf = 0;
  for(ctr = 0; ctr < 256; ctr++)
  {
    buf += (float)mADCValue[ADC_CH_REF];
    chThdSleepMicroseconds(100);
  }
  buf /= 256;

  // reference man, page 375
  mADCtoPinFactor = 3.3f * (float)vrefint_cal / (buf*4095.0f);
  // mADCtoPinFactor = 3.3f / 4095.0f;
  mADCtoVoltsFactor = mADCtoPinFactor * BOARD_ADC_PIN_TO_VOLT;
  mADCtoAmpsFactor =  mADCtoPinFactor * BOARD_ADC_PIN_TO_AMP;

  DBG2("vrefint_cal=%d\r\n",vrefint_cal);
  DBG2("mADCtoPinFactor=%f\r\n", mADCtoPinFactor);
  DBG2("mADCtoVoltsFactor=%f\r\n", mADCtoVoltsFactor);
  DBG2("mADCtoAmpsFactor=%f\r\n", mADCtoAmpsFactor);
}
/**
 * @brief      Performs the Current offset clibration of the drv8301
 */
static void drvDCCal(void)
{
  uint16_t ctr;
  uint32_t sum1, sum2;
  drvDCCalEnable();
  sum1 = 0; sum2 = 0;
  chThdSleepMilliseconds(10);
  while(drvIsFault()) chThdSleepMilliseconds(1);
  for(ctr = 0; ctr < 2000; ctr++)
  {
    sum1 += mADCValue[ADC_CH_CURR_A];
    sum2 += mADCValue[ADC_CH_CURR_B];
    chThdSleepMicroseconds(100);
  }
  mDrvOffA = sum1 / 2000;
  mDrvOffB = sum2 / 2000;
  drvDCCalDisable();
  DBG2("Current offset A/B: %04d / %04d\r\n", mDrvOffA, mDrvOffB);
}
/**
 * @brief      Calculates the duty cycles based on the input vectors in the
 * clark reference frame
 * @note       Magnitude must not be larger than sqrt(3)/2, or 0.866
 * @note       Source: https://ez.analog.com/community/motor-control-hardware-platforms2/blog/2015/08/07/matlab-script-for-space-vector-modulation-functions
 * @note       Duration: 3.738us
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
  float pwmHalfPeriod = TIM1->ARR;

  float Ta, Tb, T0;
  float ta, tb, tc;

  float va = *a;
  float vb = (*b)*ONE_BY_SQRT_3;

  if(fabsf(va) >= fabsf(vb))
  {
    Ta=fabsf(va)-fabsf(vb); // Segment 1,3,4,6
    Tb=fabsf(vb)*2;
    T0=-Ta-Tb; // assuming Ts = 1;
    if (vb >= 0)
    {
      if (va >= 0)
      {
        // sector 1
        sector = 1;
        tc = (T0/2);
        tb = (T0/2+Tb);
        ta = (T0/2+Tb+Ta);
      }
      else
      {
        // sector 3
        sector = 3;
        ta = (T0/2);
        tc = (T0/2+Ta);
        tb = (T0/2+Tb+Ta);
      }
    }
    else if (va >= 0)
    {
      // sector 6
      sector = 6;
      tb = (T0/2);
      tc = (T0/2+Tb);
      ta = (T0/2+Tb+Ta);
    }
    else
    {
      // sector 4
      sector = 4;
      ta = (T0/2);
      tb = (T0/2+Ta);
      tc = (T0/2+Tb+Ta);
    }
  }        
  else
  {
    // Segment 2,5
    Ta=fabsf(va+vb);
    Tb=fabsf(va-vb);
    T0=-Ta-Tb; // assuming Ts = 1;
    if (vb > 0)
    {
      // sector 2
      sector = 2;
      tc = (T0/2);
      ta = (T0/2+Ta);
      tb = (T0/2+Tb+Ta);
    }
    else
    {
      // sector 5
      sector = 5;
      tb = (T0/2);
      ta = (T0/2+Tb);
      tc = (T0/2+Tb+Ta);
    }
  }

  *da = (uint16_t)(pwmHalfPeriod*(ta+0.5));
  *db = (uint16_t)(pwmHalfPeriod*(tb+0.5));
  *dc = (uint16_t)(pwmHalfPeriod*(tc+0.5));        
}

/**
 * @brief      Forward clark transformation
 * @note       Duration: 1.729us, with USE_CMSIS_CLARK_PARK 1.326us
 *
 * @param      va    in: a component
 * @param      vb    in: b component
 * @param      vc    in: c component
 * @param      a     out: alpha component
 * @param      b     out: beta component
 */
static void clark (float* va, float* vb, float* vc, float* a, float* b)
{
  (void)vc;
  #ifdef USE_CMSIS_CLARK_PARK
    arm_clarke_f32(*va, *vb, a, b);
  #else
    arm_clarke_f32(*va, *vb, a, b);
    // *a = 2.0f / 3.0f * (*va - 0.5f*(*vb) - 0.5f*(*vc));
    // *b = 2.0f / 3.0f * (SQRT_3_BY_2*(*vb) - SQRT_3_BY_2*(*vc));
  #endif
}
/**
 * @brief      Forward park transformation
 * @note       Duration: 5.762us, with USE_CMSIS_CLARK_PARK 7.609us
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
  #ifdef USE_CMSIS_CLARK_PARK
    arm_sin_cos_f32(*theta, &sin, &cos);
    arm_park_f32(*a, *b, d, q, sin, cos);
  #else
    sin = arm_sin_f32(*theta);
    cos = arm_cos_f32(*theta);
    (*d) =  (*a)*cos + (*b)*sin;
    (*q) = -(*a)*sin + (*b)*cos;
  #endif
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
  #ifdef USE_CMSIS_CLARK_PARK
    arm_inv_clarke_f32(*a, *b, va, vb);
    *vc = *vb;
  #else
    *va = *a;
    *vb = 1.0f / 2.0f * (-(*a) + SQRT_3*(*b));
    *vc = 1.0f / 2.0f * (-(*a) + SQRT_3*(*b));
  #endif
}
/**
 * @brief      Inverse park transformation
 * @note       Duration: 5.637us, with USE_CMSIS_CLARK_PARK 7.483us
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
  #ifdef USE_CMSIS_CLARK_PARK
    arm_sin_cos_f32(*theta, &sin, &cos);
    arm_inv_park_f32(*d, *q, a, b, sin, cos);
  #else
    sin = arm_sin_f32(*theta);
    cos = arm_cos_f32(*theta);
    // sincos_fast(*theta, &sin, &cos);
    (*a) = (*d)*cos - (*q)*sin;
    (*b) = (*q)*cos + (*d)*sin;
  #endif
}
static float piController(piStruct_t* s, float sample, float* dt)
{
  s->istate += s->ki * sample * (*dt);
  if(s->istate > s->iceil) s->istate = s->iceil;
  if(s->istate < s->ifloor) s->istate = s->ifloor;
  return (s->kp*sample) + (s->istate);
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
static void runPositionObserver(float *dt)
{
  static float pos_error;
  static float xp[2];

  mObs.eta[0] = mObs.x[0] - mMotParms.Ls*(mCtrl.ia_is);
  mObs.eta[1] = mObs.x[1] - mMotParms.Ls*(mCtrl.ib_is);

  mObs.y[0] = -mMotParms.Rs*(mCtrl.ia_is) + (mCtrl.va_set);
  mObs.y[1] = -mMotParms.Rs*(mCtrl.ib_is) + (mCtrl.vb_set);

  pos_error =  mMotParms.psi*mMotParms.psi - 
    (mObs.eta[0]*mObs.eta[0] + mObs.eta[1]*mObs.eta[1]);

  xp[0] = mObs.y[0] + 0.5f*mFOCParms.obsGain*mObs.eta[0]*pos_error;
  xp[1] = mObs.y[1] + 0.5f*mFOCParms.obsGain*mObs.eta[1]*pos_error;
        
  mObs.x[0] += xp[0]*(*dt);
  mObs.x[1] += xp[1]*(*dt);

  mObs.theta = utilFastAtan2((mObs.x[1] - mMotParms.Ls*(mCtrl.ib_is)), 
    (mObs.x[0] - mMotParms.Ls*(mCtrl.ia_is) ));
}
/**
 * @brief      Estimates the rotor speed using a PLL
 * @note       Based on IEEE 2010 Position Estimator using a Nonlinear Observer
 *
 * @param      dt    in: time delta since last call
 */
static void runSpeedObserver (float *dt)
{
  static float err, wm;

  err = mObs.theta - mObs.theta_var;
  utils_norm_angle_rad(&err);
  // mObs.omega_e = arm_pid_f32(&mObs.speedPID, err);
  mObs.omega_e = piController(&mpiSpeed, err, dt);
  mObs.theta_var += mObs.omega_e * (*dt);
  utils_norm_angle_rad(&mObs.theta_var);
  wm = -mObs.omega_e * (60.0 / 2.0 / PI / mMotParms.p);
  UTIL_LP_FAST(mObs.omega_m, wm, 0.0005);

#ifdef DEBUG_OBSERVER
  static uint16_t downSampleCtr = 0;
  if(mStoreObserver)
  {
    if(++downSampleCtr == DEBUG_DOWNSAMPLE_FACTOR)
    {
      downSampleCtr = 0;
      // copy to store reg
      mObsDebugCounter %= OBS_STORE_DEPTH;
      mOBSValueStore[mObsDebugCounter][0] = mCtrl.ia_is;
      mOBSValueStore[mObsDebugCounter][1] = mCtrl.ib_is;
      mOBSValueStore[mObsDebugCounter][2] = mCtrl.va_set;
      mOBSValueStore[mObsDebugCounter][3] = mCtrl.vb_set;
      mOBSValueStore[mObsDebugCounter][4] = mObs.theta;
      mOBSValueStore[mObsDebugCounter][5] = mObs.omega_m;
      mObsDebugCounter++;
      if(mObsDebugCounter >= OBS_STORE_DEPTH) mStoreObserver = 0;
    }
  }
#endif
}
/**
 * @brief      Runs the speed controller, calculates id_set and iq_set
 */
static void runSpeedController (void)
{
  static float err;

  err = mCtrl.w_set - mObs.omega_m;
  mCtrl.id_set = 0.0;
  mCtrl.iq_set = arm_pid_f32(&mCtrl.speedPID, err);
}
/**
 * @brief      Runs the current controller. Calculates vd and vq
 */
static void runCurrentController (float* dt)
{
  static float d_err, q_err;

  d_err = mCtrl.id_set - mCtrl.id_is;
  q_err = mCtrl.iq_set - mCtrl.iq_is;

  // mCtrl.vd_set = arm_pid_f32(&mCtrl.idPID, d_err);
  // mCtrl.vq_set = arm_pid_f32(&mCtrl.iqPID, q_err);
  mCtrl.vd_set = piController(&mpiId, d_err, dt);
  mCtrl.vq_set = piController(&mpiIq, q_err, dt);

  // mCtrl.vd_set -= mObs.omega_e * mMotParms.Ls;
  // mCtrl.vq_set += mObs.omega_e * mMotParms.Ls;
  // mCtrl.vq_set += mObs.omega_e * mMotParms.psi;
  // 

#ifdef DEBUG_CONTROLLERS
  static uint16_t downSampleCtr = 0;
  if(mStoreController)
  {
    if(++downSampleCtr == DEBUG_DOWNSAMPLE_FACTOR)
    {
      downSampleCtr = 0;
      // copy to store reg
      mControllerDebugCtr %= CONT_STORE_DEPTH;
      mContValueStore[mControllerDebugCtr][0] = mObs.omega_m;
      mContValueStore[mControllerDebugCtr][1] = mCtrl.w_set;
      mContValueStore[mControllerDebugCtr][2] = mCtrl.id_is;
      mContValueStore[mControllerDebugCtr][3] = mCtrl.id_set;
      mContValueStore[mControllerDebugCtr][4] = mCtrl.iq_is;
      mContValueStore[mControllerDebugCtr][5] = mCtrl.iq_set;
      mContValueStore[mControllerDebugCtr][6] = mCtrl.vd_set;
      mContValueStore[mControllerDebugCtr++][7] = mCtrl.vq_set;
      if(mControllerDebugCtr >= CONT_STORE_DEPTH) mStoreController = 0;
    }
  }
#endif
}

/**
 * @brief      Calculate output vectors for the Timer and sets new dutycycle
 * @note       Needs
 *              mCtrl.vd_set
 *              mCtrl.vq_set
 *              mObs.theta
 *             Calculates
 *              mCtrl.va_set
 *              mCtrl.vb_set
 */ 
static void runOutputs(void)
{  
  uint16_t dutya, dutyb, dutyc;
  // inverse transform
  invpark(&mCtrl.vd_set, &mCtrl.vq_set, &mObs.theta, &mCtrl.va_set, &mCtrl.vb_set);
  // calculate duties
  utils_saturate_vector_2d(&mCtrl.va_set, &mCtrl.vb_set, SQRT_3_BY_2);
  svm(&mCtrl.va_set, &mCtrl.vb_set, &dutya, &dutyb, &dutyc);
  // set output
  TIMER_UPDATE_DUTY(dutyc, dutyb, dutya);
}
/**
 * @brief      Calculate output vectors for the Timer and sets new dutycycle
 * @note       Takes the theata as a parameter and ignores the observer theta
 * @note       Duration ~18us
 * @note       Needs
 *              mCtrl.vd_set
 *              mCtrl.vq_set
 *             Calculates
 *              mCtrl.va_set
 *              mCtrl.vb_set
 * param[in] theta Rotor position
 */ 
static void runOutputsWithoutObserver(float theta)
{  
  uint16_t dutya, dutyb, dutyc;
  // inverse transform
  invpark(&mCtrl.vd_set, &mCtrl.vq_set, &theta, &mCtrl.va_set, &mCtrl.vb_set); // 5.5us
  // calculate duties
  utils_saturate_vector_2d(&mCtrl.va_set, &mCtrl.vb_set, SQRT_3_BY_2); //7.325us
  svm(&mCtrl.va_set, &mCtrl.vb_set, &dutya, &dutyb, &dutyc); // 3.738us
  // set output
  TIMER_UPDATE_DUTY(dutyc, dutyb, dutya); // 0.993us
}
/**
 * @brief      Runs output in forced commutation mode
 */
static void forcedCommutation (void)
{
  static float t = 0.0;
  static const float freq = FOC_FORCED_COMM_FREQ;
  static float theta;

  theta = 2*PI*FOC_FORCED_COMM_FREQ*(t); //800ns
  mCtrl.vd_set = FOC_FORCED_COMM_VD;
  mCtrl.vq_set = FOC_FORCED_COMM_VQ;
  t += ((float)FOC_CURRENT_CONTROLLER_SLOWDOWN / FOC_F_SW);
  runOutputsWithoutObserver(theta);
}

/*===========================================================================*/
/* Interrupt handlers                                                        */
/*===========================================================================*/
/**
 * @brief      ADC1_2 IRQ handler
 */
CH_IRQ_HANDLER(Vector88) {
  static uint16_t ctr = 0;
  CH_IRQ_PROLOGUE();
  ADC_ClearITPendingBit(ADC1, ADC_IT_EOS);
  ADC1->CR |= ADC_CR_ADSTART;

  if(mStoreADC1)
  {
    // copy to store reg
    mADCValueStore[ctr][0] = mADCValue[0];
    mADCValueStore[ctr][1] = mADCValue[1];
    mADCValueStore[ctr][2] = mADCValue[2];
    mADCValueStore[ctr][3] = mADCValue[3];
    ctr++;
    if(ctr >= ADC_STORE_DEPTH) mStoreADC1 = 0;
    ctr %= ADC_STORE_DEPTH;
  }

  // mc_interface_adc_inj_int_handler();
  CH_IRQ_EPILOGUE();
}

/**
 * @brief      ADC3 IRQ handler
 */
CH_IRQ_HANDLER(VectorFC) {
  static uint16_t istCtr = 0;
  static float dt = 0;
  static float id, iq;

  CH_IRQ_PROLOGUE();
  chSysLockFromISR();
  palSetPad(GPIOE,14);

  ADC_ClearITPendingBit(ADC3, ADC_IT_EOS);
  ADC3->CR |= ADC_CR_ADSTART;

  dt = 1.0/((float)FOC_F_SW);

  // Current calculation time: 1.944us
  mCtrl.ipa_is = ADC_CURR_A();
  mCtrl.ipb_is = ADC_CURR_B();
  mCtrl.ipc_is = -mCtrl.ipa_is -mCtrl.ipb_is;
  clark(&mCtrl.ipa_is, &mCtrl.ipb_is, &mCtrl.ipc_is, &mCtrl.ia_is, &mCtrl.ib_is); //1.727us
  runPositionObserver(&dt); // 8.765us
  runSpeedObserver(&dt); // 2.895us


  if(++istCtr == FOC_CURRENT_CONTROLLER_SLOWDOWN)
  {
    istCtr = 0;
    // palSetPad(GPIOE,14);
    // chSysLockFromISR();
    // chBSemSignalI(&mIstSem);
    // chSysUnlockFromISR(); 

    park(&mCtrl.ia_is, &mCtrl.ib_is, &mObs.theta, &id, &iq);
    UTIL_LP_FAST(mCtrl.id_is, id, FOC_LP_FAST_CONSTANT);
    UTIL_LP_FAST(mCtrl.iq_is, iq, FOC_LP_FAST_CONSTANT);
    runSpeedController();

    // Force a step for the current controller
      // if((mControllerDebugCtr < (CONT_STORE_DEPTH/3)) || (!mStoreController))
      // // if((mObsDebugCounter < (OBS_STORE_DEPTH/3)) || (!mStoreObserver))
      // {
      //   mState = MC_OPEN_LOOP;
      // }
      // else
      // {
      //   mState = MC_CLOSED_LOOP;
      // }
    if(mState != MC_CLOSED_LOOP)
    {
      mCtrl.id_set = 0.0; // override
      mCtrl.iq_set = 5.0;
      mForcedCommutationMode = 1;
    }
    else
    {
      mForcedCommutationMode = 0;
    }
    runCurrentController(&dt);
    // mCtrl.vd_set = FOC_FORCED_COMM_VD; // override
    // mCtrl.vq_set = FOC_FORCED_COMM_VQ;
    if(mForcedCommutationMode)
      forcedCommutation();
    else
      runOutputs(); 
  }

#ifdef DEBUG_ADC
  if(mStoreADC3)
  {
    // copy to store reg
    mADCValueStore[ctr][4] = mADCValue[4];
    mADCValueStore[ctr][5] = mADCValue[5];
    mADCValueStore[ctr][6] = mADCValue[6];
    mADCValueStore[ctr][7] = mADCValue[7];
    ctr++;
    if(ctr >= ADC_STORE_DEPTH) mStoreADC3 = 0;
    ctr %= ADC_STORE_DEPTH;
  }
#endif

  palClearPad(GPIOE,14);
  chSysUnlockFromISR(); 
  CH_IRQ_EPILOGUE();
}



/** @} */
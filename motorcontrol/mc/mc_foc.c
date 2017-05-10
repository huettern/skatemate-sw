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
#include "usbcdc.h"

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
 * How much slower the speed control loop should run
 */
#define FOC_SPEED_CONTROLLER_SLOWDOWN 100
/**
 * How much to slow down the voltage measurements
 */
#define FOC_VOLT_MEAS_SLOWDOWN  10
/**
 * Forced commutation settings
 */
#define FOC_FORCED_COMM_FREQ -30.0
#define FOC_FORCED_COMM_VD 0.0
#define FOC_FORCED_COMM_VQ 0.07
/**
 * Maximum and minimum duty cycle
 */
#define FOC_MIN_DUTY 0.005
#define FOC_MAX_DUTY 0.95
/**
 * Minimum allowed RPM in speed control mode
 */
#define FOC_PARAM_DEFAULT_MIN_RPM 900.0
/**
 * Max and min current
 */
#define FOC_PARAM_DEFAULT_CURR_MAX 60.0
#define FOC_PARAM_DEFAULT_CURR_MIN -60.0
/**
 * Maximum battery currents
 */
#define FOC_PARAM_DEFAULT_CURR_IN_MAX 60.0
#define FOC_PARAM_DEFAULT_CURR_IN_MIN -20.0
/**
 * PI controller for duty control when decreasing the duty
 */
#define FOC_PARAM_DEFAULT_DUTY_DOWNRAMP_KP 10.0
#define FOC_PARAM_DEFAULT_DUTY_DOWNRAMP_KI 200.0
/**
 * Inject d-axis current below this duty cycle in sensorless more
 */
#define FOC_DEFAULT_PARAM_D_CURRENT_INJ_DUTY  0.0
/**
 * Maximum q-axis current factor
 */
#define FOC_DEFAULT_PARAM_D_CURRENT_INJ_FACTOR  0.0
/**
 * Openloop RPM (sensorless low speed or when finding index pulse)
 */
#define FOC_DEFAULT_PARAM_OPENLOOP_RPM 1200.0
/**
 * Time below min RPM to activate openloop (s)
 */
#define FOC_DEFAULT_PARAM_OPENLOOP_HYST 0.5
/**
 * Time to remain in openloop (s)
 */
#define FOC_DEFAULT_PARAM_OPENLOOP_TIME 0.5



/**
 * Motor default parameters
 */
#define FOC_MOTOR_DEFAULT_PSI 15.0e-3//0.00274 //15.0e-3 //0.008
#define FOC_MOTOR_DEFAULT_P   7
#define FOC_MOTOR_DEFAULT_LS  18.0e-6//1.248e-5 //35.0e-6 //18.0e-6 // Lumenier: 12e-6
#define FOC_MOTOR_DEFAULT_RS  1.2//0.01682 //20*0.04 // 1.2 // Lumenier: 0.06
#define FOC_MOTOR_DEFAULT_J   150e-6 // not used

/**
 * FOC defautl parameters
 */
#define FOC_PARAM_DEFAULT_OBS_GAIN    100e6
#define FOC_PARAM_DEFAULT_OBS_SPEED_KP  2000.0  
#define FOC_PARAM_DEFAULT_OBS_SPEED_KI  20000.0
#define FOC_PARAM_DEFAULT_OBS_SPEED_ITERM_MAX  10.0
#define FOC_PARAM_DEFAULT_OBS_SPEED_ITERM_MIN  -10.0

#define FOC_PARAM_DEFAULT_CURR_D_KP   0.03
#define FOC_PARAM_DEFAULT_CURR_D_KI   0.0

#define FOC_PARAM_DEFAULT_CURR_Q_KP   0.03
#define FOC_PARAM_DEFAULT_CURR_Q_KI   0.0

#define FOC_PARAM_DEFAULT_ITERM_CEIL     10.0
#define FOC_PARAM_DEFAULT_ITERM_FLOOR    -10.0

#define FOC_PARAM_DEFAULT_SPEED_KP    0.2
#define FOC_PARAM_DEFAULT_SPEED_KI    1.0

#define FOC_LP_FAST_CONSTANT 0.1

#define FOC_PARAPM_DEFAULT_O_CURRENT_MAX 1.0
/**
 * @brief      Resistor divider for voltage measurements
 */
// #define BOARD_ADC_PIN_TO_VOLT (float)((2.2f + 39.0f)/2.2f) // using resistor values
#define BOARD_ADC_PIN_TO_VOLT (17.1183) // using measured ratio
/**
 * @brief      Shunt resistor for current measurement
 */
#define BOARD_ADC_PIN_TO_AMP (float)(1000.0/DRV_CURRENT_GAIN)

#define FOC_MEASURE_RES_NUM_SAMPLES 1000

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

typedef struct {
  int nCurrSamples;
  int nVoltSamples;
  float curr_sum;
  float volt_sum;
  bool measure_inductance;
} sample_t;

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
  #define CONT_STORE_DEPTH 200
#else
  #define CONT_STORE_DEPTH 1
#endif
static volatile float mContValueStore[CONT_STORE_DEPTH][8];
static volatile uint8_t mStoreController;
static uint16_t mControllerDebugCtr;
static uint8_t mStartStore = 0;

static piStruct_t mpiSpeed;
static piStruct_t mpiId;
static piStruct_t mpiIq;
static piStruct_t mpiSpeedObs;
static piStruct_t mpiSpeed;

static mcMode_t mMode = MC_HALT;
static mcState_t mState = MC_STATE_OFF;
static sample_t mSample;
static volatile bool mPhaseOverride = false;
static volatile float mPhaseNowObserverOverride = 0.0;
static volatile float mPhaseNowOverride = 0.0;
static volatile float mTachometer = 0.0;
static volatile float mTachometerAbs = 0.0;
static volatile float mLastISRDuration = 0.0;

static volatile bool mPhaseObserverOverride = false;

static float mMeasuredResistance = 0;

/*===========================================================================*/
/* SHELL settings                                                            */
/*===========================================================================*/
static const usbcdcParameterStruct_t mShellSpeedObsKp = {"speed_obs_kp", &mpiSpeedObs.kp};
static const usbcdcParameterStruct_t mShellSpeedObsKi = {"speed_obs_ki", &mpiSpeedObs.ki};
static const usbcdcParameterStruct_t mShellcurr_dKp = {"curr_d_kp", &mpiId.kp};
static const usbcdcParameterStruct_t mShellcurr_dKi = {"curr_d_ki", &mpiId.ki};
static const usbcdcParameterStruct_t mShellcurr_qKp = {"curr_q_kp", &mpiIq.kp};
static const usbcdcParameterStruct_t mShellcurr_qKi = {"curr_q_ki", &mpiIq.ki};
static const usbcdcParameterStruct_t mShellSpeedKp = {"speed_kp", &mpiSpeed.kp};
static const usbcdcParameterStruct_t mShellSpeedKi = {"speed_ki", &mpiSpeed.ki};
static const usbcdcParameterStruct_t mShellObsGain = {"obs_gain", &mFOCParms.obsGain};

static const usbcdcParameterStruct_t mShellwSet = {"w_set", &mCtrl.w_set};
static const usbcdcParameterStruct_t mShellIdSet = {"id_set", &mCtrl.id_set};
static const usbcdcParameterStruct_t mShellIqSet = {"iq_set", &mCtrl.iq_set};

static const usbcdcParameterStruct_t mShellfcf = {"fc_f", &mCtrl.fc_f};
static const usbcdcParameterStruct_t mShellfcvd = {"fc_vd", &mCtrl.fc_vd};
static const usbcdcParameterStruct_t mShellfcvq = {"fc_vq", &mCtrl.fc_vq};

static const usbcdcParameterStruct_t mShellL = {"ls", &mMotParms.Ls};
static const usbcdcParameterStruct_t mShellR = {"rs", &mMotParms.Rs};
static const usbcdcParameterStruct_t mShellPSI = {"psi", &mMotParms.psi};
static const usbcdcParameterStruct_t mShellLambda = {"lambda", &mFOCParms.obsGain};

static const usbcdcParameterStruct_t* mShellVars[] = 
{
  &mShellSpeedObsKp,
  &mShellSpeedObsKi,
  &mShellcurr_dKp,
  &mShellcurr_dKi,
  &mShellcurr_qKp,
  &mShellcurr_qKi,
  &mShellObsGain,
  &mShellwSet,
  &mShellSpeedKp,
  &mShellSpeedKi,
  &mShellIdSet,
  &mShellIqSet,
  &mShellL,
  &mShellR,
  &mShellPSI,
  &mShellLambda,
  &mShellfcf,
  &mShellfcvd,
  &mShellfcvq,
  NULL
};

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
 * Effective deadtime, used for deadtime compensation
 */
#define FOC_PWM_DEADTIME_US (0.08 + 1.0e6*FOC_PWM_DEADTIME_CYCLES/SYSTEM_CORE_CLOCK)
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

static void svm (float a, float b, uint16_t* da, uint16_t* db, uint16_t* dc);
static void svm2(float alpha, float beta, uint32_t PWMHalfPeriod,
    uint32_t* tAout, uint32_t* tBout, uint32_t* tCout);

static void clark (float* va, float* vb, float* vc, float* a, float* b);
static void park (float* a, float* b, float* theta, float* d, float* q );
static void invclark (float* a, float* b, float* va, float* vb, float* vc);
static void invpark (float* d, float* q, float* theta, float* a, float* b);
static float piController(piStruct_t* s, float sample, float* dt);

static void runPositionObserver(float dt);
static void runSpeedObserver (float dt);
static void runSpeedController (float *dt);
static void runCurrentController (float dt);
static void runOutputs(void);
static void runOutputsWithoutObserver(float theta);
static void forcedCommutation (void);

static void intHandler(void);

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
  usbcdcSetShellVars(mShellVars);

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

  /******* TIM12 various time measurements *******/  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);
  // Time base configuration
  tim_tbs.TIM_Period = 0xFFFFFFFF;
  tim_tbs.TIM_Prescaler = (uint16_t)(((SYSTEM_CORE_CLOCK / 2) / 10000000) - 1);
  tim_tbs.TIM_ClockDivision = 0;
  tim_tbs.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM15, &tim_tbs);
  TIM_Cmd(TIM15, ENABLE);

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
/**
 * @brief      Starts the sampling routine
 */
void mcfStartSample(void)
{
  mStartStore = 1;
}
/**
 * @brief      Routine to measure the resistance from fet, motor and cables
 * @note       Runs the motor in openloop, measures d-q-currents and supply voltage then
 * calculates the resistance
 */
void mcfMeasureResistance(void)
{
  float curr, volt;
  uint16_t ctr = 0;
  mMeasuredResistance = 0.0;

  // Spin up motor in forced commutation
  mMode = MC_OPEN_LOOP;

  // Wait for motor to spin
  chThdSleepMilliseconds(3000);

  // Start sampling
  memset(&mSample, 0, sizeof(sample_t));
  nvicEnableVector(ADC1_2_IRQn, 8);

  while((mSample.nCurrSamples < FOC_MEASURE_RES_NUM_SAMPLES) || 
    (mSample.nVoltSamples < FOC_MEASURE_RES_NUM_SAMPLES))
  {
    chThdSleepMilliseconds(10);
    // timeout
    if(++ctr > 500) break;
  }
    
  mMode = MC_HALT;
  nvicDisableVector(ADC1_2_IRQn);

  curr = mSample.curr_sum / FOC_MEASURE_RES_NUM_SAMPLES;
  volt = (mSample.volt_sum / FOC_MEASURE_RES_NUM_SAMPLES) * 
    sqrtf(mCtrl.fc_vd*mCtrl.fc_vd + mCtrl.fc_vq*mCtrl.fc_vq);

  mMeasuredResistance = (volt / curr) * (2.0 / 3.0);

  DBG("Avg curr = %f\r\nAvg volt = %f\r\n--->Rs=%f Ohm\r\n", curr, volt, mMeasuredResistance);
}

/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/
/**
 * @brief      FOC statemachine
 *
 * @param[in]  <unnamed>  thread pointer
 * @param[in]  <unnamed>  arguments
 */
static THD_FUNCTION(mcfocMainThread, arg) 
{
  (void)arg;
  chRegSetThreadName(DEFS_THD_MCFOC_MAIN_NAME);

  mMode = MC_OPEN_LOOP;
  chThdSleepMilliseconds(3000);

  mMode = MC_CONTROL_MODE_CURRENT;
  mState = MC_STATE_RUNNING;

  // DBG3("Starting Res measurement\r\n");
  // mcfMeasureResistance();
  mCtrl.iq_set = 3.0;
  mMode = MC_CLOSED_LOOP_CURRENT;
  
  while (true) 
  {
    float openloop_rpm = utilMap(fabsf(mCtrl.iq_set),
        0.0, mFOCParms.currMax,
        0.0, mFOCParms.openloop_rpm);

    const float dt = 0.001;
    const float min_rads = (openloop_rpm * 2.0 * PI) / 60.0;
    static float min_rpm_hyst_timer = 0.0;
    static float min_rpm_timer = 0.0;

    float add_min_speed = 0.0;
    if (mCtrl.duty_now > 0.0) {
      add_min_speed = min_rads * dt;
    } else {
      add_min_speed = -min_rads * dt;
    }

    // Output a minimum speed from the observer
    if (fabsf(mObs.omega_m) < min_rads) {
      min_rpm_hyst_timer += dt;
    } else if (min_rpm_hyst_timer > 0.0) {
      min_rpm_hyst_timer -= dt;
    }

    // Don't use this in brake mode.
    if (mMode == MC_CONTROL_MODE_CURRENT_BRAKE || fabsf(mCtrl.duty_now) < 0.001) {
      min_rpm_hyst_timer = 0.0;
      mPhaseObserverOverride = false;
    }

    if (min_rpm_hyst_timer > mFOCParms.openloop_hyst && min_rpm_timer <= 0.0001) {
      min_rpm_timer = mFOCParms.openloop_time;
    }

    if (min_rpm_timer > 0.0) {
      mPhaseNowObserverOverride += add_min_speed;
      utils_norm_angle_rad((float*)&mPhaseNowObserverOverride);
      mPhaseObserverOverride = true;
      min_rpm_timer -= dt;
      min_rpm_hyst_timer = 0.0;
    } else {
      mPhaseNowObserverOverride = mObs.theta;
      mPhaseObserverOverride = false;
    }
    chThdSleepMilliseconds(1);
  }
}

/**
 * @brief      Secondary FOC thread, used for debugging only
 *
 * @param[in]  <unnamed>  thread pointer
 * @param[in]  <unnamed>  arguments
 */
static THD_FUNCTION(mcfocSecondaryThread, arg)
{
  (void)arg;
  chRegSetThreadName(DEFS_THD_MCFOC_SECOND_NAME);
  uint16_t i;
  
  while(true)
  {
    #ifdef DEBUG_ADC
      while(!mStartStore) chThdSleepMilliseconds(10);
      mStartStore = 0;
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
      while(!mStartStore) chThdSleepMilliseconds(10);
      mStartStore = 0;
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
      while(!mStartStore) chThdSleepMilliseconds(10);
      mStartStore = 0;
      mStoreController = 1;
      mMode = MC_CLOSED_LOOP_CURRENT;
      chThdSleepMilliseconds(1);
      while(mStoreController) chThdSleepMilliseconds(1);
      mMode = MC_OPEN_LOOP;
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
  mFOCParms.obsSpeed_ceil = FOC_PARAM_DEFAULT_OBS_SPEED_ITERM_MAX;
  mFOCParms.obsSpeed_floor = FOC_PARAM_DEFAULT_OBS_SPEED_ITERM_MIN;
  mFOCParms.curr_d_kp = FOC_PARAM_DEFAULT_CURR_D_KP;
  mFOCParms.curr_d_ki = FOC_PARAM_DEFAULT_CURR_D_KI;
  mFOCParms.curr_q_kp = FOC_PARAM_DEFAULT_CURR_Q_KP;
  mFOCParms.curr_q_ki = FOC_PARAM_DEFAULT_CURR_Q_KI;
  mFOCParms.speed_kp = FOC_PARAM_DEFAULT_SPEED_KP;
  mFOCParms.iTermCeil = FOC_PARAM_DEFAULT_ITERM_CEIL;
  mFOCParms.iTermFloor = FOC_PARAM_DEFAULT_ITERM_FLOOR;
  mFOCParms.obsSpeed_ceil = FOC_PARAM_DEFAULT_OBS_SPEED_ITERM_MAX;
  mFOCParms.dutyMax = FOC_MAX_DUTY;
  mFOCParms.minErpm = FOC_PARAM_DEFAULT_MIN_RPM;
  mFOCParms.currMin = FOC_PARAM_DEFAULT_CURR_MIN;
  mFOCParms.currMax = FOC_PARAM_DEFAULT_CURR_MAX;
  mFOCParms.currInMin = FOC_PARAM_DEFAULT_CURR_IN_MIN;
  mFOCParms.currInMax = FOC_PARAM_DEFAULT_CURR_IN_MAX;
  mFOCParms.duty_dowmramp_kp = FOC_PARAM_DEFAULT_DUTY_DOWNRAMP_KP;
  mFOCParms.duty_dowmramp_ki = FOC_PARAM_DEFAULT_DUTY_DOWNRAMP_KI;
  mFOCParms.dCurrInjDuty = FOC_DEFAULT_PARAM_D_CURRENT_INJ_DUTY;
  mFOCParms.dCurrInjFactor = FOC_DEFAULT_PARAM_D_CURRENT_INJ_FACTOR;
  mFOCParms.openloop_rpm = FOC_DEFAULT_PARAM_OPENLOOP_RPM;
  mFOCParms.openloop_hyst = FOC_DEFAULT_PARAM_OPENLOOP_HYST;
  mFOCParms.openloop_time = FOC_DEFAULT_PARAM_OPENLOOP_TIME;

  memset(&mObs, 0, sizeof(mcfObs_t));
  memset(&mCtrl, 0, sizeof(mcfController_t));

  mCtrl.fc_f = FOC_FORCED_COMM_FREQ;
  mCtrl.fc_vd = FOC_FORCED_COMM_VD;
  mCtrl.fc_vq = FOC_FORCED_COMM_VQ;

  // PID Controllers
  mpiId.kp = mFOCParms.curr_d_kp;
  mpiId.ki = mFOCParms.curr_d_ki;
  mpiId.istate = 0;
  mpiId.iceil = mFOCParms.iTermCeil;
  mpiId.ifloor = mFOCParms.iTermFloor;

  mpiIq.kp = mFOCParms.curr_q_kp;
  mpiIq.ki = mFOCParms.curr_q_ki;
  mpiIq.istate = 0;
  mpiIq.iceil = mFOCParms.iTermCeil;
  mpiIq.ifloor = mFOCParms.iTermFloor;

  mpiSpeedObs.kp = mFOCParms.obsSpeed_kp;
  mpiSpeedObs.ki = mFOCParms.obsSpeed_ki;
  mpiSpeedObs.istate = 0;
  mpiSpeedObs.iceil = mFOCParms.obsSpeed_ceil;
  mpiSpeedObs.ifloor = mFOCParms.obsSpeed_floor;

  mpiSpeed.kp = mFOCParms.speed_kp;
  mpiSpeed.ki = mFOCParms.speed_ki;
  mpiSpeed.istate = 0;
  mpiSpeed.iceil = mFOCParms.iTermCeil;
  mpiSpeed.ifloor = mFOCParms.iTermFloor;

  memset(&mSample, 0, sizeof(sample_t));
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
static void svm (float a, float b, uint16_t* da, uint16_t* db, uint16_t* dc)
{
  float pwmHalfPeriod = TIM1->ARR;

  float Ta, Tb, T0;
  float ta, tb, tc;

  float va = a;
  //TODO: Why?
  float vb = b*ONE_BY_SQRT_3;

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
        tc = (T0/2);
        tb = (T0/2+Tb);
        ta = (T0/2+Tb+Ta);
      }
      else
      {
        // sector 3
        ta = (T0/2);
        tc = (T0/2+Ta);
        tb = (T0/2+Tb+Ta);
      }
    }
    else if (va >= 0)
    {
      // sector 6
      tb = (T0/2);
      tc = (T0/2+Tb);
      ta = (T0/2+Tb+Ta);
    }
    else
    {
      // sector 4
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
      tc = (T0/2);
      ta = (T0/2+Ta);
      tb = (T0/2+Tb+Ta);
    }
    else
    {
      // sector 5
      tb = (T0/2);
      ta = (T0/2+Tb);
      tc = (T0/2+Tb+Ta);
    }
  }

  *da = (uint16_t)(pwmHalfPeriod*(ta+0.5));
  *db = (uint16_t)(pwmHalfPeriod*(tb+0.5));
  *dc = (uint16_t)(pwmHalfPeriod*(tc+0.5));        
}

// Magnitude must not be larger than sqrt(3)/2, or 0.866
static void svm2(float alpha, float beta, uint32_t PWMHalfPeriod,
    uint32_t* tAout, uint32_t* tBout, uint32_t* tCout) 
{
  uint32_t sector;

  if (beta >= 0.0f) 
  {
    if (alpha >= 0.0f) 
    {
      //quadrant I
      if (ONE_BY_SQRT_3 * beta > alpha)
        sector = 2;
      else
        sector = 1;
    } else {
      //quadrant II
      if (-ONE_BY_SQRT_3 * beta > alpha)
        sector = 3;
      else
        sector = 2;
    }
  } else {
    if (alpha >= 0.0f) 
    {
      //quadrant IV5
      if (-ONE_BY_SQRT_3 * beta > alpha)
        sector = 5;
      else
        sector = 6;
    } else {
      //quadrant III
      if (ONE_BY_SQRT_3 * beta > alpha)
        sector = 4;
      else
        sector = 5;
    }
  }

  // PWM timings
  uint32_t tA, tB, tC;

  switch (sector) 
  {

  // sector 1-2
  case 1: {
    // Vector on-times
    uint32_t t1 = (alpha - ONE_BY_SQRT_3 * beta) * PWMHalfPeriod;
    uint32_t t2 = (TWO_BY_SQRT_3 * beta) * PWMHalfPeriod;

    // PWM timings
    tA = (PWMHalfPeriod - t1 - t2) / 2;
    tB = tA + t1;
    tC = tB + t2;

    break;
  }

  // sector 2-3
  case 2: {
    // Vector on-times
    uint32_t t2 = (alpha + ONE_BY_SQRT_3 * beta) * PWMHalfPeriod;
    uint32_t t3 = (-alpha + ONE_BY_SQRT_3 * beta) * PWMHalfPeriod;

    // PWM timings
    tB = (PWMHalfPeriod - t2 - t3) / 2;
    tA = tB + t3;
    tC = tA + t2;

    break;
  }

  // sector 3-4
  case 3: {
    // Vector on-times
    uint32_t t3 = (TWO_BY_SQRT_3 * beta) * PWMHalfPeriod;
    uint32_t t4 = (-alpha - ONE_BY_SQRT_3 * beta) * PWMHalfPeriod;

    // PWM timings
    tB = (PWMHalfPeriod - t3 - t4) / 2;
    tC = tB + t3;
    tA = tC + t4;

    break;
  }

  // sector 4-5
  case 4: {
    // Vector on-times
    uint32_t t4 = (-alpha + ONE_BY_SQRT_3 * beta) * PWMHalfPeriod;
    uint32_t t5 = (-TWO_BY_SQRT_3 * beta) * PWMHalfPeriod;

    // PWM timings
    tC = (PWMHalfPeriod - t4 - t5) / 2;
    tB = tC + t5;
    tA = tB + t4;

    break;
  }

  // sector 5-6
  case 5: {
    // Vector on-times
    uint32_t t5 = (-alpha - ONE_BY_SQRT_3 * beta) * PWMHalfPeriod;
    uint32_t t6 = (alpha - ONE_BY_SQRT_3 * beta) * PWMHalfPeriod;

    // PWM timings
    tC = (PWMHalfPeriod - t5 - t6) / 2;
    tA = tC + t5;
    tB = tA + t6;

    break;
  }

  // sector 6-1
  case 6: {
    // Vector on-times
    uint32_t t6 = (-TWO_BY_SQRT_3 * beta) * PWMHalfPeriod;
    uint32_t t1 = (alpha + ONE_BY_SQRT_3 * beta) * PWMHalfPeriod;

    // PWM timings
    tA = (PWMHalfPeriod - t6 - t1) / 2;
    tC = tA + t1;
    tB = tC + t6;

    break;
  }
  }

  *tAout = tA;
  *tBout = tB;
  *tCout = tC;
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
static void runPositionObserver(float dt)
{
  static float x1 = 0;
  static float x2 = 0;
  const float L = (3.0 / 2.0) * mMotParms.Ls;
  const float R = (3.0 / 2.0) * mMotParms.Rs;
  const float gamma = mFOCParms.obsGain;
  const float linkage = mMotParms.psi;

  const float Lia = L * mCtrl.ia_is;
  const float Lib = L * mCtrl.ib_is;

  float k1 = (linkage * linkage) - ((x1 - Lia) * (x1 - Lia) + (x2 - Lib) * (x2 - Lib));
  float x1_dot = 0.0;
  float x2_dot = 0.0;

  // TODO: But why???
  float v_alpha = 0.0;
  float v_beta = 0.0;
  x1_dot = -R * mCtrl.ia_is + v_alpha + ((gamma / 2.0) * (x1 - Lia)) * k1;
  x2_dot = -R * mCtrl.ib_is + v_beta + ((gamma / 2.0) * (x2 - Lib)) * k1;
  x1 += x1_dot * dt;
  x2 += x2_dot * dt;

  if (fabsf(x1) > 1e20 || IS_NAN(x1)) 
  {
    x1 = 0.0;
  }

  if (fabsf(x2) > 1e20 || IS_NAN(x2)) 
  {
    x2 = 0.0;
  }

  mObs.theta = utilFastAtan2(x2 - L * mCtrl.ib_is, x1 - L * mCtrl.ia_is);

  // static float pos_error;
  // static float xp[2];

  // mObs.eta[0] = mObs.x[0] - mMotParms.Ls*(mCtrl.ia_is);
  // mObs.eta[1] = mObs.x[1] - mMotParms.Ls*(mCtrl.ib_is);

  // mObs.y[0] = -mMotParms.Rs*(mCtrl.ia_is) + (mCtrl.va_set);
  // mObs.y[1] = -mMotParms.Rs*(mCtrl.ib_is) + (mCtrl.vb_set);

  // pos_error =  mMotParms.psi*mMotParms.psi - 
  //   (mObs.eta[0]*mObs.eta[0] + mObs.eta[1]*mObs.eta[1]);

  // xp[0] = mObs.y[0] + 0.5f*mFOCParms.obsGain*mObs.eta[0]*pos_error;
  // xp[1] = mObs.y[1] + 0.5f*mFOCParms.obsGain*mObs.eta[1]*pos_error;
        
  // mObs.x[0] += xp[0]*(*dt);
  // mObs.x[1] += xp[1]*(*dt);

  // mObs.theta = utilFastAtan2((mObs.x[1] - mMotParms.Ls*(mCtrl.ib_is)), 
  //   (mObs.x[0] - mMotParms.Ls*(mCtrl.ia_is) ));
}
/**
 * @brief      Estimates the rotor speed using a PLL
 * @note       Based on IEEE 2010 Position Estimator using a Nonlinear Observer
 *
 * @param      dt    in: time delta since last call
 */
static void runSpeedObserver (float dt)
{
  static float err, wm;

  err = mObs.theta - mObs.theta_var;
  utils_norm_angle_rad(&err);
  // mObs.omega_e = arm_pid_f32(&mObs.speedPID, err);
  mObs.omega_e = piController(&mpiSpeedObs, err, &dt);
  mObs.theta_var += mObs.omega_e * dt;
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
static void runSpeedController (float *dt)
{
  static float err;

  err = mCtrl.w_set - mObs.omega_m;
  mCtrl.id_set = 0.0;
  // mCtrl.iq_set = arm_pid_f32(&mCtrl.speedPID, err);
  mCtrl.iq_set = FOC_PARAPM_DEFAULT_O_CURRENT_MAX * piController(&mpiSpeed, err, dt);
}
/**
 * @brief      Runs the current controller. Calculates vd and vq
 */
static void runCurrentController (float dt)
{
  static float d_err, q_err;
  float c,s,id,iq;

  s = arm_sin_f32(mObs.theta);
  c = arm_cos_f32(mObs.theta);

  // park(&mCtrl.ia_is, &mCtrl.ib_is, &mObs.theta, &id, &iq);
  id = c * mCtrl.ia_is + s * mCtrl.ib_is;
  iq = c * mCtrl.ib_is  - s * mCtrl.ia_is;
  UTIL_LP_FAST(mCtrl.id_is, id, FOC_LP_FAST_CONSTANT);
  UTIL_LP_FAST(mCtrl.iq_is, iq, FOC_LP_FAST_CONSTANT);

  d_err = mCtrl.id_set - mCtrl.id_is;
  q_err = mCtrl.iq_set - mCtrl.iq_is;

  float Vd = mCtrl.vd_set + d_err * mFOCParms.curr_d_kp;
  float Vq = mCtrl.vq_set + q_err * mFOCParms.curr_q_kp;
  mCtrl.vd_set += d_err * (mFOCParms.curr_d_ki * dt);
  mCtrl.vq_set += q_err * (mFOCParms.curr_q_kp * dt);

  mCtrl.mod_d = Vd / ((2.0 / 3.0) * mCtrl.vsupply);
  mCtrl.mod_q = Vq / ((2.0 / 3.0) * mCtrl.vsupply);

  // Windup protection and saturation
  utils_saturate_vector_2d(&mCtrl.mod_d, &mCtrl.mod_q,
      SQRT_3_BY_2 * FOC_MAX_DUTY);
  utils_saturate_vector_2d(&mCtrl.vd_set, &mCtrl.vq_set,
      (2.0 / 3.0) * FOC_MAX_DUTY * SQRT_3_BY_2 * mCtrl.vsupply);

  float mod_alpha = c * mCtrl.mod_d - s * mCtrl.mod_q;
  float mod_beta  = c * mCtrl.mod_q + s * mCtrl.mod_d;

  // Deadtime compensation
  const float i_alpha_set = c * mCtrl.id_set - s * mCtrl.iq_set;
  const float i_beta_set = c * mCtrl.iq_set + s * mCtrl.id_set;
  const float ia_set = i_alpha_set;
  const float ib_set = -0.5 * i_alpha_set + SQRT_3_BY_2 * i_beta_set;
  const float ic_set = -0.5 * i_alpha_set - SQRT_3_BY_2 * i_beta_set;
  const float mod_alpha_set_sgn = (2.0 / 3.0) * SIGN(ia_set) - (1.0 / 3.0) * SIGN(ib_set) - (1.0 / 3.0) * SIGN(ic_set);
  const float mod_beta_set_sgn = ONE_BY_SQRT_3 * SIGN(ib_set) - ONE_BY_SQRT_3 * SIGN(ic_set);
  const float mod_comp_fact = FOC_PWM_DEADTIME_US * 1e-6 * FOC_F_SW;
  const float mod_alpha_comp = mod_alpha_set_sgn * mod_comp_fact;
  const float mod_beta_comp = mod_beta_set_sgn * mod_comp_fact;

  // Apply compensation here so that 0 duty cyle has no glitches.
  mCtrl.va_set = (mod_alpha - mod_alpha_comp) * (2.0 / 3.0) * mCtrl.vsupply;
  mCtrl.vb_set = (mod_beta - mod_beta_comp) * (2.0 / 3.0) * mCtrl.vsupply;

  // uint16_t dutya, dutyb, dutyc;
  // svm(mod_alpha, mod_beta, &dutya, &dutyb, &dutyc);
  // TIMER_UPDATE_DUTY(dutyc, dutyb, dutya);
  // set output
  uint32_t duty1, duty2, duty3, top;
  top = TIM1->ARR;
  svm2(-mod_alpha, -mod_beta, top, &duty1, &duty2, &duty3);
  TIMER_UPDATE_DUTY(duty1, duty2, duty3);

  // d_err = mCtrl.id_set - mCtrl.id_is;
  // q_err = mCtrl.iq_set - mCtrl.iq_is;

  // // mCtrl.vd_set = arm_pid_f32(&mCtrl.idPID, d_err);
  // // mCtrl.vq_set = arm_pid_f32(&mCtrl.iqPID, q_err);
  // mCtrl.vd_set = piController(&mpiId, d_err, dt);
  // mCtrl.vq_set = piController(&mpiIq, q_err, dt);

  // // mCtrl.vd_set -= mObs.omega_e * mMotParms.Ls;
  // // mCtrl.vq_set += mObs.omega_e * mMotParms.Ls;
  // // mCtrl.vq_set += mObs.omega_e * mMotParms.psi;

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
  svm(mCtrl.va_set, mCtrl.vb_set, &dutya, &dutyb, &dutyc);
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
  svm(mCtrl.va_set, mCtrl.vb_set, &dutya, &dutyb, &dutyc); // 3.738us
  // set output
  TIMER_UPDATE_DUTY(dutyc, dutyb, dutya); // 0.993us
}
/**
 * @brief      Runs output in forced commutation mode
 */
static void forcedCommutation (void)
{
  static float t = 0.0;
  static float theta;

  theta = 2*PI*mCtrl.fc_f*(t); //800ns
  mCtrl.vd_set = mCtrl.fc_vd;
  mCtrl.vq_set = mCtrl.fc_vq;
  t += ((float)FOC_CURRENT_CONTROLLER_SLOWDOWN / FOC_F_SW);
  runOutputsWithoutObserver(theta);
}

/*===========================================================================*/
/* Interrupt handlers                                                        */
/*===========================================================================*/
/**
 * @brief      ADC1_2 IRQ handler
 */
CH_IRQ_HANDLER(Vector88) 
{
  static uint16_t ctr = 0;
  static float vd, vq;
  static uint16_t voltmeasSlowDownCtr = 0;
  CH_IRQ_PROLOGUE();
  ADC_ClearITPendingBit(ADC1, ADC_IT_EOS);
  ADC1->CR |= ADC_CR_ADSTART;

  UTIL_LP_FAST(mCtrl.vsupply, ADC_VOLT(ADC_CH_SUPPL), FOC_LP_FAST_CONSTANT);

  if(++voltmeasSlowDownCtr == FOC_VOLT_MEAS_SLOWDOWN)
  {
    voltmeasSlowDownCtr = 0;
    mSample.volt_sum += ADC_VOLT(ADC_CH_SUPPL);
    mSample.nVoltSamples++;
  }
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
CH_IRQ_HANDLER(VectorFC) 
{
  static uint16_t currSlowDownCtr = 0;
  static uint16_t speedSlowDownCtr = 0;
  static float dt = 0;
  static float id, iq;
  static float dtspeed, dtcurrent;


  CH_IRQ_PROLOGUE();
  palSetPad(GPIOE,14);
  ADC_ClearITPendingBit(ADC3, ADC_IT_EOS);
  intHandler();
  ADC3->CR |= ADC_CR_ADSTART;
  ADC1->CR |= ADC_CR_ADSTART;
  palClearPad(GPIOE,14);
  CH_IRQ_EPILOGUE();
  return;


  CH_IRQ_PROLOGUE();



  palSetPad(GPIOE,14);

  ADC_ClearITPendingBit(ADC3, ADC_IT_EOS);
  ADC3->CR |= ADC_CR_ADSTART;
  ADC1->CR |= ADC_CR_ADSTART;

  dt = 1.0/((float)FOC_F_SW);

  // Current calculation time: 1.944us
  mCtrl.ipa_is = ADC_CURR_A();
  mCtrl.ipb_is = ADC_CURR_B();
  mCtrl.ipc_is = -mCtrl.ipa_is -mCtrl.ipb_is;
  UTIL_LP_FAST(mCtrl.vsupply, ADC_VOLT(ADC_CH_SUPPL), FOC_LP_FAST_CONSTANT);
  clark(&mCtrl.ipa_is, &mCtrl.ipb_is, &mCtrl.ipc_is, &mCtrl.ia_is, &mCtrl.ib_is); //1.727us
  runPositionObserver(dt); // 8.765us
  runSpeedObserver(dt); // 2.895us
#ifdef DEBUG_CONTROLLERS
  if((mControllerDebugCtr < ((uint16_t)(CONT_STORE_DEPTH/2))) || (!mStoreController))
  {
    // mCtrl.iq_set = 2.0;
  }
  else
  {
    // mCtrl.iq_set = 4.0;
  }
#endif
#ifdef DEBUG_OBSERVER
  if((mObsDebugCounter < ((uint16_t)(OBS_STORE_DEPTH/2))) || (!mStoreObserver))
  {
    // mMode = MC_OPEN_LOOP;
  }
  else
  {
    // mMode = MC_CLOSED_LOOP_CURRENT;
  }
#endif


  if(++speedSlowDownCtr == FOC_SPEED_CONTROLLER_SLOWDOWN)
  {
    speedSlowDownCtr = 0;
    dtspeed = dt * FOC_SPEED_CONTROLLER_SLOWDOWN;
    // Force a step for the current controller
    #ifdef DEBUG_CONTROLLERS
    if((mControllerDebugCtr < (CONT_STORE_DEPTH/3)) || (!mStoreController))
    #endif
    #ifdef DEBUG_OBSERVER
    if((mObsDebugCounter < (OBS_STORE_DEPTH/2)) || (!mStoreObserver))
    #endif
    {
      mCtrl.w_set = 150;
    }
    else
    {
      mCtrl.w_set = 250;
    }
    if(mMode == MC_CLOSED_LOOP_SPEED)
    {
      // runSpeedController(&dtspeed);
    }
  }

  if(++currSlowDownCtr == FOC_CURRENT_CONTROLLER_SLOWDOWN)
  {
    currSlowDownCtr = 0;
    dtcurrent = dt * FOC_CURRENT_CONTROLLER_SLOWDOWN;
    // palSetPad(GPIOE,14);
    // chSysLockFromISR();
    // chBSemSignalI(&mIstSem);
    // chSysUnlockFromISR(); 

    // park(&mCtrl.ia_is, &mCtrl.ib_is, &mObs.theta, &id, &iq);
    // UTIL_LP_FAST(mCtrl.id_is, id, FOC_LP_FAST_CONSTANT);
    // UTIL_LP_FAST(mCtrl.iq_is, iq, FOC_LP_FAST_CONSTANT);

    // mCtrl.vd_set = FOC_FORCED_COMM_VD; // override
    // mCtrl.vq_set = FOC_FORCED_COMM_VQ;
    if(mMode == MC_HALT)
    {
      TIMER_UPDATE_DUTY(0,0,0);
    }
    else if(mMode == MC_OPEN_LOOP)
    {
      forcedCommutation();
    }
    else if(mMode == MC_CLOSED_LOOP_CURRENT)
    {
      runCurrentController(dtcurrent);
    }

    mSample.curr_sum += sqrtf(mCtrl.id_is * mCtrl.id_is + mCtrl.iq_is * mCtrl.iq_is);
    mSample.nCurrSamples++;
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

/**
 * @brief      ADC Interrupt handler
 */
static void intHandler(void)
{
  TIM15->CNT = 0;

  const float dt = 1.0 / (FOC_F_SW / 2.0);
  float ia = ADC_CURR_A();
  float ib = ADC_CURR_B();
  mCtrl.vsupply = ADC_VOLT(ADC_CH_SUPPL);

  float enc_ang = 0;

  static float phase_before = 0.0;
  const float phase_diff = angle_difference(mCtrl.phase, phase_before);
  phase_before = mCtrl.phase;

  if (mState == MC_STATE_RUNNING) {
    // Clarke transform assuming balanced currents
    mCtrl.ia_is = ia;
    mCtrl.ib_is = ONE_BY_SQRT_3 * ia + TWO_BY_SQRT_3 * ib;

    const float duty_abs = fabsf(mCtrl.duty_now);
    float id_set_tmp = mCtrl.id_set;
    float iq_set_tmp = mCtrl.iq_set;
    mCtrl.max_duty = mFOCParms.dutyMax;

    static float duty_filtered = 0.0;
    UTIL_LP_FAST(duty_filtered, mCtrl.duty_now, 0.1);
    truncate_number(&duty_filtered, -1.0, 1.0);

    float duty_set = mCtrl.duty_set;
    bool control_duty = mMode == MC_CONTROL_MODE_DUTY;

    // When the filtered duty cycle in sensorless mode becomes low in brake mode, the
    // observer has lost tracking. Use duty cycle control with the lowest duty cycle
    // to get as smooth braking as possible.
    if (mMode == MC_CONTROL_MODE_CURRENT_BRAKE
//        && (m_conf->foc_sensor_mode != FOC_SENSOR_MODE_ENCODER) // Don't use this with encoderss
        && fabsf(duty_filtered) < 0.03) {
      control_duty = true;
      duty_set = 0.0;
    }

    // Brake when set ERPM is below min ERPM
    if (mMode == MC_CONTROL_MODE_SPEED &&
        fabsf(mCtrl.w_set) < mFOCParms.minErpm) {
      control_duty = true;
      duty_set = 0.0;
    }

    if (control_duty) {
      // Duty cycle control
      static float duty_i_term = 0.0;
      if (fabsf(duty_set) < (duty_abs - 0.05) ||
          (SIGN(mCtrl.vq_is) * mCtrl.iq_is) < mFOCParms.currMin) {
        // Truncating the duty cycle here would be dangerous, so run a PID controller.

        // Compensation for supply voltage variations
        float scale = 1.0 / mCtrl.vsupply;

        // Compute error
        float error = duty_set - mCtrl.duty_now;

        // Compute parameters
        float p_term = error * mFOCParms.duty_dowmramp_kp * scale;
        duty_i_term += error * (mFOCParms.duty_dowmramp_ki * dt) * scale;

        // I-term wind-up protection
        truncate_number(&duty_i_term, -1.0, 1.0);

        // Calculate output
        float output = p_term + duty_i_term;
        truncate_number(&output, -1.0, 1.0);
        iq_set_tmp = output * mFOCParms.currMax;
      } else {
        // If the duty cycle is less than or equal to the set duty cycle just limit
        // the modulation and use the maximum allowed current.
        duty_i_term = 0.0;
        mFOCParms.dutyMax = duty_set;
        if (duty_set > 0.0) {
          iq_set_tmp = mFOCParms.currMax;
        } else {
          iq_set_tmp = -mFOCParms.currMax;
        }
      }
    } else if (mMode == MC_CONTROL_MODE_CURRENT_BRAKE) {
      // Braking
      iq_set_tmp = fabsf(iq_set_tmp);

      if (phase_diff > 0.0) {
        iq_set_tmp = -iq_set_tmp;
      } else if (phase_diff == 0.0) {
        iq_set_tmp = 0.0;
      }
    }

    // Run observer
    if (!mPhaseOverride) {
      runPositionObserver(dt);
    }

    if (mPhaseObserverOverride) {
      mCtrl.phase = mPhaseNowObserverOverride;
    } else {
      mCtrl.phase = mObs.theta;
    }

    // Inject D axis current at low speed to make the observer track
    // better. This does not seem to be necessary with dead time
    // compensation.
    // Note: this is done at high rate prevent noise.
    if (!mPhaseOverride) {
      if (duty_abs < mFOCParms.dCurrInjDuty) {
        id_set_tmp = utilMap(duty_abs, 0.0, mFOCParms.dCurrInjDuty,
            fabsf(mCtrl.iq_set) * mFOCParms.dCurrInjFactor, 0.0);
      } else {
        id_set_tmp = 0.0;
      }
    }

    if (mPhaseOverride) {
      mCtrl.phase = mPhaseNowOverride;
    }

    // Apply current limits
    const float mod_q = mCtrl.mod_q;
    truncate_number(&iq_set_tmp, mFOCParms.currMin, mFOCParms.currMax);
    utils_saturate_vector_2d(&id_set_tmp, &iq_set_tmp, mFOCParms.currMax);
    if (mod_q > 0.001) {
      truncate_number(&iq_set_tmp, mFOCParms.currInMin / mod_q, mFOCParms.currInMax / mod_q);
    } else if (mod_q < -0.001) {
      truncate_number(&iq_set_tmp, mFOCParms.currInMax / mod_q, mFOCParms.currInMin / mod_q);
    }

    mCtrl.id_set = id_set_tmp;
    mCtrl.iq_set = iq_set_tmp;

    runCurrentController(dt);
  } else {
    // Track back emf
    float Va = ADC_VOLT(ADC_CH_PH_A);
    float Vb = ADC_VOLT(ADC_CH_PH_B);
    float Vc = ADC_VOLT(ADC_CH_PH_C);

    // Clarke transform
    mCtrl.va_is = (2.0 / 3.0) * Va - (1.0 / 3.0) * Vb - (1.0 / 3.0) * Vc;
    mCtrl.vb_is = ONE_BY_SQRT_3 * Vb - ONE_BY_SQRT_3 * Vc;

    // Park transform
    float c,s;
    s = arm_sin_f32(mCtrl.phase);
    c = arm_cos_f32(mCtrl.phase);
    mCtrl.vd_is = c * mCtrl.va_is + s * mCtrl.vb_is;
    mCtrl.vq_is = c * mCtrl.vb_is  - s * mCtrl.va_is;

    // Update corresponding modulation
    mCtrl.mod_d = mCtrl.vd_is / ((2.0 / 3.0) * mCtrl.vsupply);
    mCtrl.mod_q = mCtrl.vq_is / ((2.0 / 3.0) * mCtrl.vsupply);

    // The current is 0 when the motor is undriven
    mCtrl.ia_is = 0.0;
    mCtrl.ib_is = 0.0;
    mCtrl.id_is = 0.0;
    mCtrl.id_is = 0.0;
    mCtrl.id_is_filter = 0.0;
    mCtrl.id_is_filter = 0.0;
    mCtrl.isupply = 0.0;
    mCtrl.i_abs = 0.0;
    mCtrl.i_abs_filter = 0.0;

    // Run observer
    runPositionObserver(dt);
    mCtrl.phase = mObs.theta;
  }

  // Calculate duty cycle
  mCtrl.duty_now = SIGN(mCtrl.vq_is) *
      sqrtf(mCtrl.mod_d * mCtrl.mod_d +
          mCtrl.mod_q * mCtrl.mod_q) / SQRT_3_BY_2;

  // Run PLL for speed estimation
  runSpeedObserver(dt);

  // Update tachometer
  static float phase_last = 0.0;
  int diff = (int)((angle_difference_rad(mCtrl.phase, phase_last) * (180.0 / PI)) / 60.0);
  if (diff != 0) {
    mTachometer += diff;
    mTachometerAbs += fabsf(diff);
    phase_last = mCtrl.phase;
  }

  // // Track position control angle
  // // TODO: Have another look at this.
  // float angle_now = 0.0;
  // angle_now = mCtrl.phase * (180.0 / PI); 

  // if (m_conf->p_pid_ang_div > 0.98 && m_conf->p_pid_ang_div < 1.02) {
  //   m_pos_pid_now = angle_now;
  // } else {
  //   static float angle_last = 0.0;
  //   float diff_f = utils_angle_difference(angle_now, angle_last);
  //   angle_last = angle_now;
  //   m_pos_pid_now += diff_f / m_conf->p_pid_ang_div;
  //   utils_norm_angle((float*)&m_pos_pid_now);
  // }

  // // Run position control
  // if (m_state == MC_STATE_RUNNING) {
  //   run_pid_control_pos(m_pos_pid_now, m_pos_pid_set, dt);
  // }

  // // MCIF handler
  // mc_interface_mc_timer_isr();

  mLastISRDuration = (float) TIM15->CNT / 10000000.0;
}


/** @} */
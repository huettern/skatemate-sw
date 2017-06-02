/*
   SKATEMATE - Copyright (C) 2017 FHNW Project 4 Team 2
 */

/**
 * @file       main.c
 * @brief      Main routine and LED control
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       23 March 2017
 * 
 *
 * @addtogroup MAIN
 * @brief main routine and UX
 * @{
 */
// #include <core_cm4_simd.h>

#include "ch.h"
#include "hal.h"

#include "shell.h"

#include "defs.h"
// #include "chprintf.h"
#include "utelemetry.h"
#include "drv8301.h"
#include "mc_foc.h"

#include "usbcdc.h"
#include "usbcfg.h"
#include "chprintf.h"

#include "stm32f30x_conf.h"

// #include "core_cm4_simd.h"
#include "arm_math.h"

/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/

/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/
uint8_t ydata[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
uint8_t xdata[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

static void icuwidthcb(ICUDriver *icup);
static void icuperiodcb(ICUDriver *icup);
icucnt_t last_width, last_period;
#define PWM_IN_LOW 763
#define PWM_IN_MID 1594
#define PWM_IN_HIGH 2100
static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_HIGH,
  1000000,                                    /* 10kHz ICU clock frequency.   */
  icuwidthcb,
  icuperiodcb,
  NULL,
  ICU_CHANNEL_2,
  0
};

/*===========================================================================*/
/* prototypes                                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* callbacks                                                                 */
/*===========================================================================*/
static void icuwidthcb(ICUDriver *icup) {
  last_width = icuGetWidthX(icup);
}

static void icuperiodcb(ICUDriver *icup) {
  last_period = icuGetPeriodX(icup);
}

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/*
 * Application entry point.
 */
int main(void) 
{
  /*
  * System initializations.
  */
  halInit();
  chSysInit();

  /**
  * User init
  // */
  // usbcdcInit();

  /**
   * ICU for PWM capture
   */
  icuStart(&ICUD3, &icucfg);
  palSetPadMode(GPIOB, 5, PAL_MODE_ALTERNATE(2));
  icuStartCapture(&ICUD3);
  icuEnableNotifications(&ICUD3);

  /*
   * Shell manager initialization.
   */
  shellInit();

  /**
   * Idle thread
   */
  chRegSetThreadName(DEFS_THD_IDLE_NAME);

  /*
   * USART TX/RX for shell
   */
  palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7)); // used function : USART3_TX
  palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7)); // used function : USART3_RX
  sdStart(&SD3, NULL);

  /*
   * Onboard LEDs
   */
  palSetPadMode(GPIOC, 13, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOC, 14, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOC, 15, PAL_MODE_OUTPUT_PUSHPULL);

  /**
   * Start Motor driver
   */
  drvInit();
  mcfInit();

  chThdSleepMilliseconds(2000);
  mcfSetForcedCommutationFrequency(-5.0);
  chThdSleepMilliseconds(2000);
  // mcfSetCurrentFactor(0.2);
  // mcfSetDuty(900,900,900);
  while (true) 
  {

    // chThdSleepMilliseconds(200);
    // palSetPad(GPIOC, 13);
    // palClearPad(GPIOC, 15);
    // chThdSleepMilliseconds(200);
    // palSetPad(GPIOC, 14);
    // palClearPad(GPIOC, 13);
    // chThdSleepMilliseconds(200);
    // palSetPad(GPIOC, 15);
    // palClearPad(GPIOC, 14);

    chThdSleepMilliseconds(1);
    float input_val;
    uint8_t ctr;
    if(++ctr > 100)
    {
      input_val = ((float)last_width - PWM_IN_MID)/((float)PWM_IN_HIGH - PWM_IN_MID);
      if(input_val > 1.0) input_val = 1.0;
      if(input_val < -1.0) input_val = -1.0;
      // mcfSetCurrentFactor(input_val);
      ctr = 0;
    }
    
    usbcdcHandleShell();
  }
}

void assert_failed(uint8_t* file, uint32_t line) {
  (void)file; (void)line;
  // DBG("Assert fail at File %s Line %d", file, (int)line);
  while(1) palSetPad(GPIOC, 15);;   // hang here (behavior is our your choice)  
} 

/** @} */

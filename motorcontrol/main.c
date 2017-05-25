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
  */
  // usbcdcInit();

  /**
   * ICU for PWM capture
   */
  // icuStart(&ICUD3, &icucfg);
  // palSetPadMode(GPIOB, 5, PAL_MODE_ALTERNATE(2));
  // icuStartCapture(&ICUD3);
  // icuEnableNotifications(&ICUD3);

  /*
   * Shell manager initialization.
   */
  // shellInit();

  /**
   * Idle thread
   */
  chRegSetThreadName(DEFS_THD_IDLE_NAME);

  // shellInit();
  // drvInit();
  // mcfInit();
  palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7)); // used function : USART3_TX
  palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7)); // used function : USART3_RX
  // sdStart(&SD3, NULL);

  palSetPadMode(GPIOC, 13, PAL_MODE_OUTPUT_PUSHPULL);
  while (true) 
  {
    chThdSleepMilliseconds(200);
    palSetPad(GPIOC, 13);
    chThdSleepMilliseconds(200);
    palClearPad(GPIOC, 13);

    
    // chThdSleepMilliseconds(1);
    // if(++ctr > 100)
    // {
    //   input_val = ((float)last_width - PWM_IN_MID)/((float)PWM_IN_HIGH - PWM_IN_MID);
    //   if(input_val > 1.0) input_val = 1.0;
    //   if(input_val < -1.0) input_val = -1.0;
    //   mcfSetCurrentFactor(input_val);
    //   ctr = 0;
    // }
    
    
    // usbcdcHandleShell();
  }
  
  
  // CMSIS benchmark
  // systime_t ticks, pidtime, sintime;
  // arm_pid_instance_f32 pid;
  // float32_t a, b;
  // arm_pid_init_f32(&pid, 1);
  // while(true)
  // {
  //   ticks = chVTGetSystemTimeX();
  //   for(op1 = 0; op1 < 1000; op1++) arm_pid_f32(&pid,10.0f);
  //   pidtime = chVTGetSystemTimeX();
  //   for(op1 = 0; op1 < 1000; op1++) arm_sin_cos_f32(29.8, &a, &b);
  //   sintime = chVTGetSystemTimeX();


  //   chprintf(bssusb, "PID time: %fus \r\n", (float)(pidtime-ticks)/CH_CFG_ST_FREQUENCY*1000);
  //   chprintf(bssusb, "SINCOS time: %fus \r\n", (float)(sintime-pidtime)/CH_CFG_ST_FREQUENCY*1000);

  //   chThdSleepMilliseconds(2000);
  // }

  
  // chThdSleepMilliseconds(10000);
  // utlmEnable(true);
  // while (true) 
  // {
  //   chThdSleepMilliseconds(1000);
  //   utlmSend(0, 10, xdata, ydata);
  //   for(i = 0; i < 10; i ++)
  //   {
  //     xdata[i] = xdata[i] + 10;
  //   }
    
  //   xda++;
  //   yda = 1.0;
  //   utlmSend(1, 1, &xda, &yda);


  //   palTogglePad(GPIOE,14);


  //   // usbcdcHandleShell();
  // }
}

void assert_failed(uint8_t* file, uint32_t line) {
  // DBG("Assert fail at File %s Line %d", file, (int)line);
  while(1) palSetPad(GPIOC, 15);;   // hang here (behavior is our your choice)  
} 

/** @} */

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

#include "usbcdc.h"
#include "usbcfg.h"
#include "chprintf.h"

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


/*===========================================================================*/
/* prototypes                                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* callbacks                                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/*
 * Application entry point.
 */
int main(void) 
{
  static int i;
  static uint64_t xda=0;
  static float yda;

  uint32_t op1,op2,res;

  /*
  * System initializations.
  */
  halInit();
  chSysInit();

  /**
  * User init
  */
  usbcdcInit();

  /*
   * Shell manager initialization.
   */
  // shellInit();

  palSetPadMode(GPIOE, 14, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(GPIOE,14);

  /**
   * Idle thread
   */
  chRegSetThreadName(DEFS_THD_IDLE_NAME);

  op1 = 10;
  op2 = 20;
  res = __UADD8(op1,op2);

  // wait 10sec
  shellInit();
  drvInit();
  while (true) 
  {
    chThdSleepMilliseconds(1);
    // drvDumpStatus();
    usbcdcHandleShell();
    palTogglePad(GPIOE,14);
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

/** @} */

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

#include "ch.h"
#include "hal.h"

#include "shell.h"

#include "defs.h"
// #include "chprintf.h"
#include "utelemetry.h"

#include "usbcdc.h"
#include "usbcfg.h"

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

  // wait 10sec
  chThdSleepMilliseconds(10000);
  utlmEnable(true);

  while (true) 
  {
    chThdSleepMilliseconds(1000);
    utlmSend(0, 10, xdata, ydata);
    for(i = 0; i < 10; i ++)
    {
      xdata[i] = xdata[i] + 10;
    }
    
    xda++;
    yda = 1.0;
    utlmSend(1, 1, &xda, &yda);


    palTogglePad(GPIOE,14);


    // usbcdcHandleShell();
  }
}

/** @} */

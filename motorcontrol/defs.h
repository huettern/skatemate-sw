/*
   SKATEMATE - Copyright (C) 2016 FHNW Project 3 Team 2
 */

/**
 * @file       defs.h
 * @brief      Thread and debugging definitions
 * @details 
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       23 March 2017
 * 
 *
 * @addtogroup MAIN
 * @{
 */

#ifndef _DEFS_H
#define _DEFS_H

#include "chprintf.h"
#include "usbcfg.h"


#define DEFS_THD_IDLE_WA_SIZE 			0x500
#define DEFS_THD_SHELL_WA_SIZE 			2048
#define DEFS_THD_MCFOCMAIN_WA_SIZE		1024
#define DEFS_THD_MCFOCSECOND_WA_SIZE	1024

#define DEFS_THD_IDLE_NAME 				"main"
#define DEFS_THD_SHELL_NAME 			"shell"
#define DEFS_THD_MCFOC_MAIN_NAME		"mcfoc_main"
#define DEFS_THD_MCFOC_SECOND_NAME		"mcfoc_sec"

#define SYSTEM_CORE_CLOCK	72000000 //Hz
#define APB1_CLOCK	36000000 //Hz
#define APB2_CLOCK	72000000 //Hz
#define AHB_CLOCK	72000000 //Hz
#define PLL_CLOCK	72000000 //Hz

#define DEFS_SHELL_STREAM SD3

// DBG for usb debugging messages
#define DBG(X, ...) chprintf((BaseSequentialStream *)&DEFS_SHELL_STREAM, X, ##__VA_ARGS__ )

// DBG2 for slow UART debugging
// #define DBG2(X, ...) chprintf((BaseSequentialStream *)&SD3, X, ##__VA_ARGS__ )
#define DBG2(X, ...)

// DBG3 for fast UART debugging
#define DBG3(X, ...) chprintf((BaseSequentialStream *)&SD3, X, ##__VA_ARGS__ )
// #define DBG3(X, ...)

#define LED_RED_ON() palSetPad(GPIOC, 15)
#define LED_RED_OFF() palClearPad(GPIOC, 15)
#define LED_GRN_ON() palSetPad(GPIOC, 13)
#define LED_GRN_OFF() palClearPad(GPIOC, 13)
#define LED_BLU_ON() palSetPad(GPIOC, 14)
#define LED_BLU_OFF() palClearPad(GPIOC, 14)

/*===========================================================================*/
/* Test defines. Always uncomment for production                             */
/*===========================================================================*/


#endif

/** @} */
